# PicoNet Node - RFM69 Wireless Communication

A wireless communication system built for Raspberry Pi Pico 2, implementing point-to-point and broadcast messaging using RFM69 radio modules.

### Key Features

- **Addressed Communication**: Each node has a unique address for targeted messaging
- **Broadcast Support**: Nodes can send messages to all nodes in the network
- **Reliable Transmission**: Automatic acknowledgment and retry for unicast messages
- **CSMA Protocol**: Carrier Sense Multiple Access for collision avoidance
- **Flexible Addressing**: Configurable address filtering (none, node-only, or node+broadcast)
- **USB Serial Logging**: Real-time monitoring of transmission and reception

## 📡 Network Architecture

The system operates on **433 MHz** frequency with the following addressing scheme:

- **Node Addresses**: 0x01, 0x02, 0x03... (unique per device)
- **Broadcast Address**: 0xFF (all nodes receive)
- **Packet Format**: Variable length with CRC validation

### Communication Modes

1. **Unicast with ACK**: Reliable point-to-point messaging with automatic retries
2. **Broadcast**: One-to-many messaging without acknowledgment
3. **Listen Mode**: Continuous reception with address filtering

## 🏗️ Project Structure

```
piconet-node/
├── common/
│   ├── rfm69.h          # RFM69 register definitions and API
│   └── rfm69.c          # RFM69 driver implementation
├── sender/
│   └── main_sender.c    # Transmission node application
├── receiver/
│   └── main_receiver.c  # Reception node application
└── CMakeLists.txt       # Build configuration
```

### Hardware Configuration

**Pin Mapping (Raspberry Pi Pico 2):**
- SPI0 SCK: GPIO 18
- SPI0 MOSI: GPIO 19  
- SPI0 MISO: GPIO 16
- SPI0 CS: GPIO 17
- RESET: GPIO 20
- DIO0 (IRQ): GPIO 21
- Status LED: GPIO 25

## 🔧 Implementation Details

### RFM69 Driver (`common/rfm69.c`)

The driver provides a high-level API for wireless communication:

- **Initialization**: Configures radio parameters (frequency, modulation, addressing)
- **Transmission**: Handles packet formatting, transmission, and ACK waiting
- **Reception**: Manages interrupt-driven packet reception and filtering
- **Power Management**: Optimized sleep/standby mode transitions

### Sender Application (`sender/main_sender.c`)

**Node Address**: 0x01  
**Target Address**: 0x02

**Functionality**:
- Sends timestamped messages every 3 seconds
- Alternates between unicast (with ACK) and broadcast modes
- Implements simple CSMA before transmission
- Provides transmission status feedback via LED and serial

**Message Types**:
1. Unicast: `"Hello from Sender 0x01 - Count: X (UNICAST)"`
2. Broadcast: `"Hello from Sender 0x01 - Count: X (BROADCAST)"`

### Receiver Application (`receiver/main_receiver.c`)

**Node Address**: 0x02  
**Address Filter**: Node + Broadcast

**Functionality**:
- Continuously listens for incoming packets
- Automatically sends ACK for unicast messages
- Displays received data with RSSI and sender information
- Visual feedback via LED on packet reception

## 📊 Protocol Features

### Packet Structure
- **Header**: Target address, sender address, packet flags
- **Payload**: User data (up to 61 bytes)
- **CRC**: Automatic error detection

### Reliability Mechanisms
- **Acknowledgment**: Automatic ACK for unicast messages
- **Retry Logic**: Up to 3 transmission attempts
- **CSMA**: Listen-before-talk to avoid collisions
- **RSSI Monitoring**: Signal strength indication

### Address Filtering Options
- `ADDR_FILTER_NONE`: Accept all packets
- `ADDR_FILTER_NODE`: Accept only packets for this node
- `ADDR_FILTER_BOTH`: Accept node-addressed and broadcast packets

## 🚀 Getting Started

### Prerequisites
- Raspberry Pi Pico 2 boards
- RFM69 radio modules (433 MHz)
- Pico SDK environment set up

### Building
```bash
cd piconet-node
mkdir build && cd build
cmake .. -DPICO_BOARD=pico2_w
make -j4
```

### Flashing
1. Hold BOOTSEL and connect Pico via USB
2. Drag `piconet_sender.uf2` to one Pico
3. Drag `piconet_receiver.uf2` to another Pico

### Monitoring
Connect to USB serial at 115200 baud to view transmission logs and received messages.

## 🔍 Testing and Debugging

### Expected Behavior
- **Sender**: Alternates between unicast and broadcast every 3 seconds
- **Receiver**: Logs all received packets with RSSI values
- **LED Activity**: Blinks on successful transmission/reception

### Troubleshooting
- Check wiring connections match pin mapping
- Verify both modules use same frequency (433 MHz)
- Ensure adequate power supply for radio transmission
- Monitor serial output for initialization errors

## 📈 Performance Characteristics

- **Range**: ~100-500m (line of sight, depending on antenna and environment)
- **Data Rate**: 4.8 kbps (configurable)
- **Packet Rate**: ~1 packet per 3 seconds (configurable)
- **Power Consumption**: <100mA during transmission, <20mA during reception

## 🛠️ Customization

### Changing Node Addresses
Modify `NODE_ADDRESS` and `DEST_ADDRESS` constants in source files.

### Adjusting Transmission Intervals
Change `sleep_ms(3000)` values in sender loop.

### Network Expansion
Add more nodes by assigning unique addresses and configuring appropriate filtering.

## 📋 Technical Specifications

### Radio Configuration
- **Frequency**: 433 MHz
- **Modulation**: FSK (Frequency Shift Keying)
- **Data Rate**: 4.8 kbps
- **Sync Word**: 0x2D, 0xD4
- **Preamble**: 3 bytes
- **Output Power**: Configurable (default: moderate)

### Addressing System
- **Node ID Range**: 0x01 - 0xFE
- **Broadcast ID**: 0xFF
- **Address Length**: 1 byte
- **Maximum Nodes**: 254 (excluding broadcast)

### Packet Specifications
- **Maximum Payload**: 61 bytes
- **Header Overhead**: 3 bytes (addresses + flags)
- **CRC**: 16-bit polynomial
- **Variable Length**: Yes, with length byte

## 📚 API Reference

### Core Functions
```c
bool rfm69_init(uint8_t node_addr, addr_filter_t filter);
bool rfm69_send_with_ack(uint8_t dest_addr, const uint8_t *data, uint8_t len);
bool rfm69_send_packet_addressed(uint8_t dest_addr, const uint8_t *data, uint8_t len);
bool rfm69_receive_packet(uint8_t *buffer, uint8_t *len, uint8_t *sender_addr, int16_t *rssi);
```

### Utility Functions
```c
void rfm69_set_mode(uint8_t mode);
int16_t rfm69_get_rssi(void);
bool rfm69_packet_available(void);
```

## 🔧 Hardware Setup

### Required Components
- 2x Raspberry Pi Pico 2 boards
- 2x RFM69 radio modules (433 MHz)
- Breadboards and jumper wires
- USB cables for programming and serial monitoring

### Wiring Diagram
Connect RFM69 modules to both Pico boards according to the pin mapping. Ensure proper power (3.3V) and ground connections.

## 🌐 Network Topology

The system supports various network topologies:

1. **Point-to-Point**: Direct communication between two nodes
2. **Star Network**: Central hub communicating with multiple nodes
3. **Broadcast Network**: One node broadcasting to multiple receivers
4. **Mesh-Ready**: Foundation for implementing mesh routing protocols

## 🚀 Future Enhancements

Potential areas for expansion:
- Multi-hop routing for extended range
- Encryption for secure communication
- Power optimization with sleep modes
- Frequency hopping for interference resistance
- Integration with sensor networks
- Over-the-air firmware updates

---

**License**: MIT  