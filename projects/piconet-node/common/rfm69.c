#include "rfm69.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "pico/time.h"
#include <stdio.h>
#include <string.h>

#define SPI_FREQ_HZ     1000000  // 1 MHz

static uint8_t _node_address = 0x00;
static bool _is_sender_mode = false;  // Track if we're in sender mode
static rfm69_reliability_config_t _rel_cfg = {
    .max_retries = 0,
    .ack_timeout_ms = 100,
    .auto_ack_enabled = true,
    .duplicate_suppression = true,
    .csma_enabled = false,
    .csma_rssi_threshold_dbm = -95,
    .csma_listen_time_ms = 5,
    .csma_max_backoff_ms = 20,
};
static uint8_t _next_seq = 1; // 0 reserved

typedef struct {
    uint8_t src;
    uint8_t seq;
} dup_entry_t;

#define DUP_CACHE_SIZE 8
static dup_entry_t _dup_cache[DUP_CACHE_SIZE];
static uint8_t _dup_idx = 0;

// Chip select control
static void cs_select(void) {
    gpio_put(SPI_CS_PIN, 0);
}

static void cs_deselect(void) {
    gpio_put(SPI_CS_PIN, 1);
}

// Hardware reset
void rfm69_reset(void) {
    gpio_put(RESET_PIN, 1);
    sleep_ms(10);
    gpio_put(RESET_PIN, 0);
    sleep_ms(10);
}

// Write single register
void rfm69_write_register(uint8_t reg, uint8_t value) {
    uint8_t buf[2];
    buf[0] = 0x80 | reg;
    buf[1] = value;
    
    cs_select();
    spi_write_blocking(SPI_PORT, buf, 2);
    cs_deselect();
    sleep_us(50);
}

// Read single register
uint8_t rfm69_read_register(uint8_t reg) {
    uint8_t buf[2] = {reg & 0x7F, 0xFF};
    uint8_t rx[2];
    
    cs_select();
    spi_write_read_blocking(SPI_PORT, buf, rx, 2);
    cs_deselect();
    
    return rx[1];
}

// Set operating mode
void rfm69_set_mode(uint8_t mode) {
    rfm69_write_register(REG_OPMODE, mode);
    sleep_ms(1);
}

// Check if RFM69 is connected
bool rfm69_check_connection(void) {
    for (int i = 0; i < 3; i++) {
        uint8_t version = rfm69_read_register(REG_VERSION);
        if (version == 0x24) {
            return true;
        }
        sleep_ms(10);
    }
    return false;
}

// Get RSSI value
int8_t rfm69_get_rssi(void) {
    return -(rfm69_read_register(REG_RSSIVAL) / 2);
}

static bool channel_is_busy(void) {
    // Lower RSSI value (more negative) means quieter; busy if RSSI >= threshold
    int8_t rssi = rfm69_get_rssi();
    bool busy = rssi >= _rel_cfg.csma_rssi_threshold_dbm;
    if (busy) {
        printf("    Channel busy: RSSI=%d >= threshold=%d\n", rssi, _rel_cfg.csma_rssi_threshold_dbm);
    }
    return busy;
}

void rfm69_set_reliability_config(const rfm69_reliability_config_t *cfg) {
    if (cfg != NULL) {
        _rel_cfg = *cfg;
    }
}

// Initialize RFM69HCW with addressing
bool rfm69_init(uint8_t node_addr, uint8_t filter_mode) {
    printf("DEBUG: rfm69_init starting...\n");
    // Validate filter mode
    if (filter_mode > ADDR_FILTER_BOTH) {
        printf("DEBUG: Invalid filter mode\n");
        return false;
    }
    
    _node_address = node_addr;
    printf("DEBUG: Setting node address to 0x%02X\n", node_addr);
    
    // Initialize SPI
    printf("DEBUG: Initializing SPI...\n");
    spi_init(SPI_PORT, SPI_FREQ_HZ);
    gpio_set_function(SPI_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI_MOSI_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI_MISO_PIN, GPIO_FUNC_SPI);
    printf("DEBUG: SPI GPIO setup complete\n");
    
    // Initialize CS pin
    gpio_init(SPI_CS_PIN);
    gpio_set_dir(SPI_CS_PIN, GPIO_OUT);
    cs_deselect();
    printf("DEBUG: CS pin initialized\n");
    
    // Initialize RESET pin
    gpio_init(RESET_PIN);
    gpio_set_dir(RESET_PIN, GPIO_OUT);
    gpio_put(RESET_PIN, 0);
    printf("DEBUG: RESET pin initialized\n");
    
    // Initialize DIO0 pin
    gpio_init(DIO0_PIN);
    gpio_set_dir(DIO0_PIN, GPIO_IN);
    gpio_pull_down(DIO0_PIN);
    printf("DEBUG: DIO0 pin initialized\n");
    
    // Configure SPI format
    spi_set_format(SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    printf("DEBUG: SPI format configured\n");
    
    // Reset the module
    printf("DEBUG: Resetting RFM69...\n");
    rfm69_reset();
    sleep_ms(10);
    printf("DEBUG: Reset complete\n");
    
    // Check connection
    printf("DEBUG: Checking connection...\n");
    if (!rfm69_check_connection()) {
        printf("DEBUG: Connection check failed!\n");
        return false;
    }
    printf("DEBUG: Connection check passed!\n");
    
    // Configuration with addressing
    printf("DEBUG: Starting configuration...\n");
    const uint8_t config[][2] = {
        {REG_OPMODE, MODE_STANDBY},        // Standby mode
        {REG_DATAMODUL, 0x00},             // FSK, packet mode
        {REG_BITRATEMSB, 0x1A},            // 4800 bps
        {REG_BITRATELSB, 0x0B},
        {REG_FDEVMSB, 0x00},               // 5kHz frequency deviation
        {REG_FDEVLSB, 0x52},
        {REG_FRFMSB, 0x6C},                // 433 MHz
        {REG_FRFMID, 0x40},
        {REG_FRFLSB, 0x00},
        {REG_PALEVEL, 0x9F},               // PA0, max power
        {REG_OCP, 0x1A},                   // OCP enabled
        {REG_RXBW, 0x55},                  // RX bandwidth
        {REG_RSSITHRESH, 0xE4},            // RSSI threshold -114 dBm (0xE4 = 228, -228/2 = -114)
        {REG_PREAMBLEMSB, 0x00},           // 4 byte preamble
        {REG_PREAMBLELSB, 0x04},
        {REG_SYNCCONFIG, 0x88},            // Sync on, 2 bytes
        {REG_SYNCVALUE1, 0x2D},            // Network ID
        {REG_SYNCVALUE2, 0xD4},
        {REG_NODEADRS, node_addr},         // Node address
        {REG_BROADCASTADRS, BROADCAST_ADDR}, // Broadcast address
        {REG_FIFOTHRESH, 0x8F},            // FIFO threshold
        {REG_DIOMAPPING1, 0x40},           // DIO0 = PayloadReady/PacketSent
        {REG_DIOMAPPING2, 0x07}
    };
    
    // Calculate PACKETCONFIG1 value based on filter mode
    // 0x90 => VarLen (bit7) + CRC On (bit4)
    // Address filtering uses bits [2:1]: 01=Node, 10=Node+Bcast
    uint8_t packet_config1 = 0x90;  // Variable length + CRC on, no filtering
    if (filter_mode == ADDR_FILTER_NODE) {
        packet_config1 |= 0x02;     // Node filtering (bits[2:1] = 01)
    } else if (filter_mode == ADDR_FILTER_BOTH) {
        packet_config1 |= 0x04;     // Node + Broadcast (bits[2:1] = 10)
    }
    
    // Write configuration
    for (int i = 0; i < sizeof(config) / sizeof(config[0]); i++) {
        rfm69_write_register(config[i][0], config[i][1]);
    }
    
    // Set packet config with addressing
    rfm69_write_register(REG_PACKETCONFIG1, packet_config1);
    
    return true;
}

// Send packet with addressing
bool rfm69_send_packet_addressed(uint8_t dest_addr, uint8_t src_addr, uint8_t *data, uint8_t length) {
    if (length > MAX_PAYLOAD_SIZE - 2 || data == NULL) {  // -2 for dest and src addresses
        return false;
    }
    
    // Ensure we're in standby mode
    rfm69_set_mode(MODE_STANDBY);
    
    // Clear FIFO
    rfm69_read_register(REG_IRQFLAGS2);
    
    // Write to FIFO
    cs_select();
    uint8_t fifo_addr = REG_FIFO | 0x80;
    spi_write_blocking(SPI_PORT, &fifo_addr, 1);
    
    // Write length (includes dest addr + src addr + payload)
    uint8_t total_length = length + 2;  // +1 for dest, +1 for src
    spi_write_blocking(SPI_PORT, &total_length, 1);
    
    // Write destination address
    spi_write_blocking(SPI_PORT, &dest_addr, 1);
    
    // Write source address as first byte of payload
    spi_write_blocking(SPI_PORT, &src_addr, 1);
    
    // Write actual payload
    spi_write_blocking(SPI_PORT, data, length);
    
    cs_deselect();
    
    // Start transmission
    rfm69_set_mode(MODE_TX);
    
    // Wait for packet sent
    absolute_time_t timeout = make_timeout_time_ms(1000);
    while (!time_reached(timeout)) {
        if (rfm69_read_register(REG_IRQFLAGS2) & 0x08) {  // PacketSent
            rfm69_set_mode(MODE_STANDBY);
            return true;
        }
        sleep_us(100);
    }
    
    rfm69_set_mode(MODE_STANDBY);
    return false;
}

// Internal helper to build and send a raw addressed frame (with provided payload bytes)
static bool _rfm69_send_frame(uint8_t dest_addr, const uint8_t *payload, uint8_t payload_len) {
    if (payload_len > MAX_PAYLOAD_SIZE || payload == NULL) return false;
    rfm69_set_mode(MODE_STANDBY);
    rfm69_read_register(REG_IRQFLAGS2);

    cs_select();
    uint8_t fifo_addr = REG_FIFO | 0x80;
    spi_write_blocking(SPI_PORT, &fifo_addr, 1);
    uint8_t total_length = payload_len + 1; // +1 for dest
    spi_write_blocking(SPI_PORT, &total_length, 1);
    spi_write_blocking(SPI_PORT, &dest_addr, 1);
    spi_write_blocking(SPI_PORT, payload, payload_len);
    cs_deselect();

    rfm69_set_mode(MODE_TX);
    absolute_time_t timeout = make_timeout_time_ms(1000);
    while (!time_reached(timeout)) {
        if (rfm69_read_register(REG_IRQFLAGS2) & 0x08) {  // PacketSent
            rfm69_set_mode(MODE_STANDBY);
            return true;
        }
        sleep_us(100);
    }
    rfm69_set_mode(MODE_STANDBY);
    return false;
}

// Reliable send with CSMA and ACK/Retry
bool rfm69_send_with_ack(uint8_t dest_addr, uint8_t src_addr, const uint8_t *data, uint8_t length) {
    if (length > (MAX_PAYLOAD_SIZE - 3) || data == NULL) { // src + seq + flags + data must fit
        return false;
    }

    // Build link-layer payload: [SRC][SEQ][FLAGS][DATA...]
    uint8_t buf[MAX_PAYLOAD_SIZE];
    uint8_t seq = _next_seq++;
    if (_next_seq == 0) _next_seq = 1; // avoid 0
    buf[0] = src_addr;
    buf[1] = seq;
    buf[2] = RFM69_LL_FLAG_ACK_REQ;
    memcpy(&buf[3], data, length);
    uint8_t payload_len = 3 + length;

    int attempts = 0;
    int csma_attempts = 0;
    const int max_csma_attempts = 50; // Prevent infinite CSMA loop
    
    for (;;) {
        // CSMA/LBT: listen and backoff if necessary
        if (_rel_cfg.csma_enabled) {
            // Enter RX to measure channel activity
            rfm69_set_mode(MODE_RX);
            absolute_time_t listen_deadline = make_timeout_time_ms(_rel_cfg.csma_listen_time_ms);
            bool busy = false;
            while (!time_reached(listen_deadline)) {
                if (channel_is_busy()) { busy = true; break; }
                sleep_ms(1);
            }
            rfm69_set_mode(MODE_STANDBY);
            if (busy) {
                csma_attempts++;
                printf("    CSMA: Channel busy (attempt %d/%d)\n", csma_attempts, max_csma_attempts);
                if (csma_attempts >= max_csma_attempts) {
                    // Give up after too many CSMA failures
                    printf("    CSMA: Too many busy attempts, giving up\n");
                    return false;
                }
                if (_rel_cfg.csma_max_backoff_ms) {
                    uint32_t backoff = (to_ms_since_boot(get_absolute_time()) ^ seq) % (_rel_cfg.csma_max_backoff_ms + 1);
                    sleep_ms(backoff);
                } else {
                    sleep_ms(5);
                }
                // retry LBT loop without consuming a retry attempt
                continue;
            }
        }

        // Transmit
        printf("    Attempt %d: Transmitting...\n", attempts + 1);
        bool tx_ok = _rfm69_send_frame(dest_addr, buf, payload_len);
        if (!tx_ok) {
            printf("    TX failed!\n");
        }

        // For broadcast, no ACK expected
        if (dest_addr == BROADCAST_ADDR || _rel_cfg.max_retries == 0) {
            return tx_ok;
        }

        // Wait for ACK
        printf("    Waiting for ACK (timeout: %d ms)...\n", _rel_cfg.ack_timeout_ms);
        absolute_time_t wait_ack_deadline = make_timeout_time_ms(_rel_cfg.ack_timeout_ms);
        rfm69_packet_t pkt;
        bool ack_received = false;
        _is_sender_mode = true;  // Set sender mode to disable auto-ACK
        while (!time_reached(wait_ack_deadline) && !ack_received) {
            if (rfm69_receive_packet(&pkt, 10)) {  // Check every 10ms for ACK
                printf("    Received packet: src=0x%02X, dest=0x%02X, seq=%d, flags=0x%02X\n", 
                       pkt.src_addr, pkt.dest_addr, pkt.seq, pkt.flags);
                // Expect: src=dest_addr, dest=_node_address, flags has ACK and seq matches
                // Parse flags & seq if available
                uint8_t got_flags = 0;
                uint8_t got_seq = 0;
                if (pkt.length >= 4) { // dest + (src,seq,flags) at least
                    got_seq = pkt.seq;
                    got_flags = pkt.flags;
                }
                if (pkt.src_addr == dest_addr && pkt.dest_addr == _node_address && 
                    (got_flags & RFM69_LL_FLAG_ACK) && got_seq == seq) {
                    printf("    ACK received!\n");
                    ack_received = true;
                } else {
                    printf("    Not matching ACK (expected: src=0x%02X, dest=0x%02X, seq=%d, flags with ACK)\n",
                           dest_addr, _node_address, seq);
                }
                // else ignore and keep waiting until timeout
            }
        }
        
        _is_sender_mode = false;  // Reset sender mode
        if (ack_received) {
            return true; // ACKed
        }

        if (attempts++ >= _rel_cfg.max_retries) {
            printf("    No ACK after %d attempts, giving up\n", _rel_cfg.max_retries + 1);
            return false; // no ACK after retries
        }
        printf("    No ACK, retrying...\n");
        // small retry backoff
        sleep_ms(10 + (seq % 10));
    }
}

// Receive packet
bool rfm69_receive_packet(rfm69_packet_t *packet, uint32_t timeout_ms) {
    if (packet == NULL) {
        return false;
    }
    
    // Initialize packet structure
    memset(packet, 0, sizeof(rfm69_packet_t));
    
    // Enter receive mode
    rfm69_set_mode(MODE_RX);
    
    absolute_time_t timeout = make_timeout_time_ms(timeout_ms);
    
    // Wait for packet or timeout
    while (!time_reached(timeout)) {
        // Check PayloadReady flag
        uint8_t irq_flags2 = rfm69_read_register(REG_IRQFLAGS2);
        if (irq_flags2 & 0x04) {  // PayloadReady bit
            // Get RSSI
            packet->rssi = rfm69_get_rssi();
            
            // Read from FIFO
            cs_select();
            uint8_t fifo_addr = REG_FIFO & 0x7F;  // Read from FIFO
            spi_write_blocking(SPI_PORT, &fifo_addr, 1);
            
            // Read length
            uint8_t length_buf;
            spi_read_blocking(SPI_PORT, 0xFF, &length_buf, 1);
            packet->length = length_buf;
            
            // Validate length to prevent buffer overflow
            if (packet->length == 0 || packet->length > MAX_PAYLOAD_SIZE + 1) {
                cs_deselect();
                // Stay in RX to avoid missing subsequent frames
                return false;
            }
            
            // Read destination address
            spi_read_blocking(SPI_PORT, 0xFF, &packet->dest_addr, 1);
            
            // Read remaining payload (source address + data)
            uint8_t payload_length = packet->length - 1;  // -1 for dest addr
            
            // Declare saved variables outside the if block
            uint8_t saved_src = 0;
            uint8_t saved_seq = 0;
            uint8_t saved_flags = 0;
            uint8_t saved_dest = packet->dest_addr;
            
            if (payload_length > 0 && payload_length <= MAX_PAYLOAD_SIZE) {
                spi_read_blocking(SPI_PORT, 0xFF, packet->payload, payload_length);
                
                // IMMEDIATELY capture sequence number before ANY other processing
                uint8_t raw_seq_byte = packet->payload[1];
                
                packet->src_addr = packet->payload[0];  // First byte is source
                packet->seq = (payload_length >= 2) ? packet->payload[1] : 0;
                packet->flags = (payload_length >= 3) ? packet->payload[2] : 0;
                
                // IMMEDIATELY save critical values to avoid corruption
                saved_src = packet->src_addr;
                saved_seq = raw_seq_byte;  // Use the raw byte captured immediately
                saved_flags = packet->flags;
                
                printf("    DEBUG RX: raw_seq_byte=%d, packet seq=%d, saved_seq=%d\n", 
                       raw_seq_byte, packet->seq, saved_seq);
            } else {
                // Invalid payload length
                cs_deselect();
                // Stay in RX
                return false;
            }
            
            cs_deselect();
            
            // Duplicate suppression (for data frames, not ACKs)
            if (_rel_cfg.duplicate_suppression && !(packet->flags & RFM69_LL_FLAG_ACK) && packet->seq != 0) {
                for (int i = 0; i < DUP_CACHE_SIZE; ++i) {
                    if (_dup_cache[i].src == packet->src_addr && _dup_cache[i].seq == packet->seq) {
                        // Keep RX
                        return false; // drop duplicate
                    }
                }
                // Add to duplicate cache
                _dup_cache[_dup_idx].src = packet->src_addr;
                _dup_cache[_dup_idx].seq = packet->seq;
                _dup_idx = (_dup_idx + 1) % DUP_CACHE_SIZE;
            }

            printf("    DEBUG DUP: After dup check, packet->seq=%d\n", packet->seq);

            // Auto-ACK for unicast when requested (use saved values to avoid corruption)
            if (_rel_cfg.auto_ack_enabled && !_is_sender_mode &&
                saved_dest != BROADCAST_ADDR &&
                (saved_flags & RFM69_LL_FLAG_ACK_REQ)) {
                printf("    Sending ACK to Node 0x%02X (seq=%d)\n", saved_src, saved_seq);
                
                // Small delay to ensure sender is ready to receive ACK
                sleep_ms(2);
                
                uint8_t ack_payload[3];
                ack_payload[0] = _node_address; // SRC = me
                ack_payload[1] = saved_seq;     // Use saved sequence number
                ack_payload[2] = RFM69_LL_FLAG_ACK; // ACK flag
                printf("    ACK payload: [0x%02X, 0x%02X, 0x%02X] -> dest=0x%02X\n", 
                       ack_payload[0], ack_payload[1], ack_payload[2], saved_src);
                bool ack_sent = _rfm69_send_frame(saved_src, ack_payload, 3);
                printf("    ACK transmission: %s\n", ack_sent ? "OK" : "FAILED");
                
                // After TX, re-enter RX quickly
                rfm69_set_mode(MODE_RX);
                sleep_ms(1);
            }

            return true;
        }
        
        sleep_us(100);
    }
    
    // Timeout - keep listening in RX to avoid gaps
    return false;
}