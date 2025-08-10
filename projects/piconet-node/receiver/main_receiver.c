#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "rfm69.h"

#define LED_PIN 25
#define NODE_ADDRESS 0x02  // This node's address

int main() {
    stdio_init_all();
    
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    
    // Startup blink
    for (int i = 0; i < 5; i++) {
        gpio_put(LED_PIN, 1);
        sleep_ms(50);
        gpio_put(LED_PIN, 0);
        sleep_ms(50);
    }
    
    sleep_ms(3000);
    
    printf("\n=== RFM69 RECEIVER (Node 0x%02X) ===\n", NODE_ADDRESS);
    printf("Initializing...\n");
    
    // Initialize with node+broadcast filtering
    if (!rfm69_init(NODE_ADDRESS, ADDR_FILTER_BOTH)) {
        printf("ERROR: Failed to initialize RFM69!\n");
        while (1) {
            gpio_put(LED_PIN, 1);
            sleep_ms(100);
            gpio_put(LED_PIN, 0);
            sleep_ms(100);
        }
    }
    
    printf("RFM69 initialized!\n");
    
    // Configure link-layer reliability for auto-ACK
    rfm69_reliability_config_t rel = {
        .max_retries = 3,
        .ack_timeout_ms = 150,
        .auto_ack_enabled = true,  // Enable automatic ACK sending
        .duplicate_suppression = true,
        .csma_enabled = false,  // Receiver doesn't need CSMA for ACKs
        .csma_rssi_threshold_dbm = -80,
        .csma_listen_time_ms = 5,
        .csma_max_backoff_ms = 30,
    };
    rfm69_set_reliability_config(&rel);
    
    printf("Listening for packets...\n");
    printf("Accepting: Node 0x%02X and Broadcast 0x%02X\n\n", 
           NODE_ADDRESS, BROADCAST_ADDR);
    
    rfm69_packet_t packet;
    int packet_count = 0;
    
    // Success pattern
    for (int i = 0; i < 3; i++) {
        gpio_put(LED_PIN, 1);
        sleep_ms(200);
        gpio_put(LED_PIN, 0);
        sleep_ms(200);
    }
    
    while (1) {
        // Try to receive a packet (100ms timeout)
        if (rfm69_receive_packet(&packet, 100)) {
            packet_count++;
            
            // Blink LED
            gpio_put(LED_PIN, 1);
            
            printf("\n=== PACKET #%d RECEIVED ===\n", packet_count);
            printf("From Node    : 0x%02X\n", packet.src_addr);
            printf("To Address   : 0x%02X %s\n", 
                   packet.dest_addr,
                   packet.dest_addr == BROADCAST_ADDR ? "(Broadcast)" : "(Unicast)");
         printf("RSSI         : %d dBm\n", packet.rssi);
            bool has_ll = (packet.dest_addr != BROADCAST_ADDR) &&
                          ((packet.length - 1) >= 3) &&
                          ((packet.flags & (RFM69_LL_FLAG_ACK | RFM69_LL_FLAG_ACK_REQ)) != 0);
            if (has_ll) {
                printf("Seq/Flags    : %u / 0x%02X%s\n",
                       packet.seq,
                       packet.flags,
                       (packet.flags & RFM69_LL_FLAG_ACK) ? " (ACK)" : ((packet.flags & RFM69_LL_FLAG_ACK_REQ) ? " (ACK_REQ)" : ""));
            } else {
                printf("Seq/Flags    : absent\n");
            }
         // User data length: if ll present -> total - 4, else legacy -> total - 2
         int user_len = (int)packet.length - (has_ll ? 4 : 2);
            if (user_len < 0) user_len = 0;
            printf("Payload Len  : %d bytes\n", user_len);
            
            // Print payload (skip source address byte)
            printf("Message      : \"");
            int start = has_ll ? 3 : 1; // skip src(+seq+flags if present)
            for (int i = start; i < packet.length - 1; i++) {
                putchar(packet.payload[i]);
            }
            printf("\"\n");
            
            // Hex dump
            printf("Raw Payload  : ");
            for (int i = 0; i < packet.length - 1; i++) {
                printf("%02X ", packet.payload[i]);
            }
            printf("\n");
            
            gpio_put(LED_PIN, 0);
            
            // Double blink for received packet
            sleep_ms(50);
            for (int i = 0; i < 2; i++) {
                gpio_put(LED_PIN, 1);
                sleep_ms(25);
                gpio_put(LED_PIN, 0);
                sleep_ms(25);
            }
        }
        
        // Quick LED pulse to show we're alive
        static int alive_counter = 0;
        if (++alive_counter > 10) {
            gpio_put(LED_PIN, 1);
            sleep_us(1000);
            gpio_put(LED_PIN, 0);
            alive_counter = 0;
        }
    }
    
    return 0;
}