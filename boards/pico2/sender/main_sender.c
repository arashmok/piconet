#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "rfm69.h"

#define LED_PIN 25
#define NODE_ADDRESS 0x01  // This node's address
#define DEST_ADDRESS 0x02  // Destination node address

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
    
    printf("\n=== RFM69 SENDER (Node 0x%02X) ===\n", NODE_ADDRESS);
    printf("Initializing...\n");
    
    // Initialize with no address filtering (sender doesn't need to receive)
    if (!rfm69_init(NODE_ADDRESS, ADDR_FILTER_NONE)) {
        printf("ERROR: Failed to initialize RFM69!\n");
        while (1) {
            gpio_put(LED_PIN, 1);
            sleep_ms(100);
            gpio_put(LED_PIN, 0);
            sleep_ms(100);
        }
    }
    
    printf("RFM69 initialized!\n");
    // Configure link-layer reliability
    rfm69_reliability_config_t rel = {
        .max_retries = 3,
        .ack_timeout_ms = 150,
        .auto_ack_enabled = true,
        .duplicate_suppression = true,
        .csma_enabled = true,
        .csma_rssi_threshold_dbm = -90,
        .csma_listen_time_ms = 5,
        .csma_max_backoff_ms = 30,
    };
    rfm69_set_reliability_config(&rel);
    printf("Sending to Node 0x%02X and Broadcast\n\n", DEST_ADDRESS);
    
    uint8_t hello_msg[] = "Hello from Node 1!";
    uint8_t broadcast_msg[] = "Broadcast message!";
    int packet_count = 0;
    
    // Success pattern
    for (int i = 0; i < 3; i++) {
        gpio_put(LED_PIN, 1);
        sleep_ms(200);
        gpio_put(LED_PIN, 0);
        sleep_ms(200);
    }
    
    while (1) {
        packet_count++;
        
        // Send unicast message to Node 2
        printf("[%d] Sending unicast to Node 0x%02X: '%s'\n", 
               packet_count, DEST_ADDRESS, hello_msg);
        
        gpio_put(LED_PIN, 1);
    if (rfm69_send_with_ack(DEST_ADDRESS, NODE_ADDRESS,
                hello_msg, sizeof(hello_msg) - 1)) {
            printf("    Sent OK!\n");
        } else {
            printf("    Send failed!\n");
        }
        gpio_put(LED_PIN, 0);
        
        sleep_ms(1000);
        
        // Send broadcast message
        printf("[%d] Sending broadcast: '%s'\n", 
               packet_count, broadcast_msg);
        
        gpio_put(LED_PIN, 1);
    if (rfm69_send_packet_addressed(BROADCAST_ADDR, NODE_ADDRESS,
                    broadcast_msg, sizeof(broadcast_msg) - 1)) {
            printf("    Sent OK!\n");
        } else {
            printf("    Send failed!\n");
        }
        gpio_put(LED_PIN, 0);
        
        // Double blink
        sleep_ms(100);
        for (int i = 0; i < 2; i++) {
            gpio_put(LED_PIN, 1);
            sleep_ms(50);
            gpio_put(LED_PIN, 0);
            sleep_ms(50);
        }
        
        printf("\n");
        sleep_ms(2000);
    }
    
    return 0;
}