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
            printf("Payload Len  : %d bytes\n", packet.length - 2);  // -2 for addresses
            
            // Print payload (skip source address byte)
            printf("Message      : \"");
            for (int i = 1; i < packet.length - 1; i++) {  // Start from 1 to skip src addr
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