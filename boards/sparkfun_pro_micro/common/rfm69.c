#include "rfm69.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "pico/time.h"
#include <stdio.h>
#include <string.h>

#define SPI_FREQ_HZ     1000000  // 1 MHz

static uint8_t _node_address = 0x00;

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

// Initialize RFM69HCW with addressing
bool rfm69_init(uint8_t node_addr, uint8_t filter_mode) {
    _node_address = node_addr;
    
    // Initialize SPI
    spi_init(SPI_PORT, SPI_FREQ_HZ);
    gpio_set_function(SPI_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI_MOSI_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI_MISO_PIN, GPIO_FUNC_SPI);
    
    // Initialize CS pin
    gpio_init(SPI_CS_PIN);
    gpio_set_dir(SPI_CS_PIN, GPIO_OUT);
    cs_deselect();
    
    // Initialize RESET pin
    gpio_init(RESET_PIN);
    gpio_set_dir(RESET_PIN, GPIO_OUT);
    gpio_put(RESET_PIN, 0);
    
    // Initialize DIO0 pin
    gpio_init(DIO0_PIN);
    gpio_set_dir(DIO0_PIN, GPIO_IN);
    gpio_pull_down(DIO0_PIN);
    
    // Configure SPI format
    spi_set_format(SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    
    // Reset the module
    rfm69_reset();
    sleep_ms(10);
    
    // Check connection
    if (!rfm69_check_connection()) {
        return false;
    }
    
    // Configuration with addressing
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
    uint8_t packet_config1 = 0x90;  // Variable length, CRC on, no filtering
    if (filter_mode == ADDR_FILTER_NODE) {
        packet_config1 = 0x91;  // Add node filtering
    } else if (filter_mode == ADDR_FILTER_BOTH) {
        packet_config1 = 0x92;  // Add node+broadcast filtering
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
    if (length > MAX_PAYLOAD_SIZE - 1) {  // -1 for source address
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

// Receive packet
bool rfm69_receive_packet(rfm69_packet_t *packet, uint32_t timeout_ms) {
    // Enter receive mode
    rfm69_set_mode(MODE_RX);
    
    absolute_time_t timeout = make_timeout_time_ms(timeout_ms);
    
    // Wait for packet or timeout
    while (!time_reached(timeout)) {
        // Check PayloadReady flag
        if (rfm69_read_register(REG_IRQFLAGS2) & 0x04) {
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
            
            // Read destination address
            spi_read_blocking(SPI_PORT, 0xFF, &packet->dest_addr, 1);
            
            // Read remaining payload (source address + data)
            uint8_t payload_length = packet->length - 1;  // -1 for dest addr
            if (payload_length > 0 && payload_length <= MAX_PAYLOAD_SIZE) {
                spi_read_blocking(SPI_PORT, 0xFF, packet->payload, payload_length);
                packet->src_addr = packet->payload[0];  // First byte is source
            }
            
            cs_deselect();
            
            // Return to standby
            rfm69_set_mode(MODE_STANDBY);
            return true;
        }
        
        sleep_us(100);
    }
    
    // Timeout - return to standby
    rfm69_set_mode(MODE_STANDBY);
    return false;
}