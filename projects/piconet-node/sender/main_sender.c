#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "rfm69.h"

#define NODE_ADDRESS 0x01  // This node's address
#define DEST_ADDRESS 0x02  // Destination node address

int main() {
    stdio_init_all();
    
    // Initialize CYW43 for LED control on Pico W
    if (cyw43_arch_init()) {
        printf("Failed to initialize CYW43\n");
        return 1;
    }
    
    // Startup blink
    for (int i = 0; i < 5; i++) {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        sleep_ms(50);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        sleep_ms(50);
    }
    
    sleep_ms(3000);
    
    printf("\n=== RFM69 SENDER (Node 0x%02X) ===\n", NODE_ADDRESS);
    printf("Initializing...\n");
    
    // Initialize with node address filtering (sender needs to receive ACKs)
    if (!rfm69_init(NODE_ADDRESS, ADDR_FILTER_NODE)) {
        printf("ERROR: Failed to initialize RFM69!\n");
        while (1) {
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
            sleep_ms(100);
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
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
        .csma_enabled = true,  // CSMA/CA enabled
        .csma_rssi_threshold_dbm = -80,  // RSSI threshold for CSMA/CA enabled
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
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        sleep_ms(200);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        sleep_ms(200);
    }
    
    while (1) {
        packet_count++;
        
        // Send unicast message to Node 2
        printf("[%d] Sending unicast to Node 0x%02X: '%s'\n", 
               packet_count, DEST_ADDRESS, hello_msg);
        
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    if (rfm69_send_with_ack(DEST_ADDRESS, NODE_ADDRESS,
                hello_msg, sizeof(hello_msg) - 1)) {
            printf("    Sent OK!\n");
        } else {
            printf("    Send failed!\n");
        }
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        
        sleep_ms(1000);
        
        // Send broadcast message
        printf("[%d] Sending broadcast: '%s'\n", 
               packet_count, broadcast_msg);
        
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    if (rfm69_send_packet_addressed(BROADCAST_ADDR, NODE_ADDRESS,
                    broadcast_msg, sizeof(broadcast_msg) - 1)) {
            printf("    Sent OK!\n");
        } else {
            printf("    Send failed!\n");
        }
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        
        // Double blink
        sleep_ms(100);
        for (int i = 0; i < 2; i++) {
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
            sleep_ms(50);
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
            sleep_ms(50);
        }
        
        printf("\n");
        sleep_ms(2000);
    }
    
    return 0;
}