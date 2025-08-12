#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

#define LED_PIN 25

int main() {
    stdio_init_all();
    
    // Initialize LED pin
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    
    printf("=== SIMPLE LED TEST ===\n");
    printf("This device should blink LED on GPIO 25\n");
    
    while (1) {
        printf("LED ON\n");
        gpio_put(LED_PIN, 1);
        sleep_ms(500);
        
        printf("LED OFF\n");
        gpio_put(LED_PIN, 0);
        sleep_ms(500);
    }
    
    return 0;
}
