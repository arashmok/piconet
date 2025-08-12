#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

int main() {
    stdio_init_all();
    
    printf("=== LED TEST STARTING ===\n");
    
    // Initialize CYW43 for onboard LED on Pico2 W
    if (cyw43_arch_init()) {
        printf("Failed to initialize cyw43\n");
        return 1;
    }
    printf("CYW43 initialized successfully\n");
    
    printf("Starting LED blink test...\n");
    
    int count = 0;
    while (count < 20) {
        printf("Blink %d\n", count + 1);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        sleep_ms(500);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        sleep_ms(500);
        count++;
    }
    
    printf("LED test completed\n");
    cyw43_arch_deinit();
    return 0;
}
