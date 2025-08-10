#ifndef RFM69_H
#define RFM69_H

#include <stdint.h>
#include <stdbool.h>

// Pin definitions
#define SPI_PORT        spi0
#define SPI_SCK_PIN     18
#define SPI_MOSI_PIN    19
#define SPI_MISO_PIN    16
#define SPI_CS_PIN      17
#define RESET_PIN       20
#define DIO0_PIN        21

// RFM69 Register Addresses
#define REG_FIFO            0x00
#define REG_OPMODE          0x01
#define REG_DATAMODUL       0x02
#define REG_BITRATEMSB      0x03
#define REG_BITRATELSB      0x04
#define REG_FDEVMSB         0x05
#define REG_FDEVLSB         0x06
#define REG_FRFMSB          0x07
#define REG_FRFMID          0x08
#define REG_FRFLSB          0x09
#define REG_VERSION         0x10
#define REG_PALEVEL         0x11
#define REG_OCP             0x13
#define REG_RXBW            0x19
#define REG_DIOMAPPING1     0x25
#define REG_DIOMAPPING2     0x26
#define REG_IRQFLAGS1       0x27
#define REG_IRQFLAGS2       0x28
#define REG_RSSITHRESH      0x29
#define REG_PREAMBLEMSB     0x2C
#define REG_PREAMBLELSB     0x2D
#define REG_SYNCCONFIG      0x2E
#define REG_SYNCVALUE1      0x2F
#define REG_SYNCVALUE2      0x30
#define REG_PACKETCONFIG1   0x37
#define REG_PACKETCONFIG2   0x38
#define REG_NODEADRS        0x39
#define REG_BROADCASTADRS   0x3A
#define REG_FIFOTHRESH      0x3C
#define REG_RSSIVAL         0x24

// Operating modes
#define MODE_SLEEP          0x00
#define MODE_STANDBY        0x04
#define MODE_FS             0x08
#define MODE_TX             0x0C
#define MODE_RX             0x10

// Address filtering modes
#define ADDR_FILTER_NONE    0x00
#define ADDR_FILTER_NODE    0x01
#define ADDR_FILTER_BOTH    0x02

// Network configuration
#define BROADCAST_ADDR      0xFF
#define MAX_PAYLOAD_SIZE    60

// Link-layer flags (stored after src_addr in payload)
#define RFM69_LL_FLAG_ACK_REQ   0x01
#define RFM69_LL_FLAG_ACK       0x80

// Reliability configuration
typedef struct {
    // ACK/Retry
    uint8_t  max_retries;         // Number of retransmissions for reliable send
    uint16_t ack_timeout_ms;      // How long to wait for ACK after each TX
    bool     auto_ack_enabled;    // Receiver auto-sends ACK for unicast frames with ACK_REQ
    bool     duplicate_suppression; // Drop duplicate data frames (same src+seq)

    // CSMA/LBT
    bool     csma_enabled;        // Perform listen-before-talk
    int8_t   csma_rssi_threshold_dbm; // Consider channel busy if RSSI >= this (e.g., -90)
    uint16_t csma_listen_time_ms; // Time to listen before TX
    uint16_t csma_max_backoff_ms; // Random backoff window if busy
} rfm69_reliability_config_t;

// Packet structure
typedef struct {
    uint8_t length;         // Total length including address
    uint8_t dest_addr;      // Destination address
    uint8_t src_addr;       // Source address (first byte of payload)
    uint8_t payload[MAX_PAYLOAD_SIZE];
    int8_t rssi;           // RSSI of received packet
    // Link-layer metadata (parsed when available)
    uint8_t seq;            // Sequence number (payload[1])
    uint8_t flags;          // Flags (payload[2])
} rfm69_packet_t;

// Function prototypes
bool rfm69_init(uint8_t node_addr, uint8_t filter_mode);
void rfm69_reset(void);
void rfm69_write_register(uint8_t reg, uint8_t value);
uint8_t rfm69_read_register(uint8_t reg);
bool rfm69_send_packet_addressed(uint8_t dest_addr, uint8_t src_addr, uint8_t *data, uint8_t length);
bool rfm69_receive_packet(rfm69_packet_t *packet, uint32_t timeout_ms);
void rfm69_set_mode(uint8_t mode);
bool rfm69_check_connection(void);
int8_t rfm69_get_rssi(void);

// Reliability APIs
void rfm69_set_reliability_config(const rfm69_reliability_config_t *cfg);
bool rfm69_send_with_ack(uint8_t dest_addr, uint8_t src_addr, const uint8_t *data, uint8_t length);

#endif