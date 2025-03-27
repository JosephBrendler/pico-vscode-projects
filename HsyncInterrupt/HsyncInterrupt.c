#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

#define IRQ_PIN 15

volatile uint64_t lastInterruptTime = 0;
volatile uint64_t timeBetweenInterrupts = 0;
volatile uint64_t currentTime = 0;
volatile uint32_t interrupt_count = 0;
volatile bool interrupt_flag = false;

void gpio_callback(uint gpio, uint32_t events) {
    currentTime = time_us_64();
    timeBetweenInterrupts = currentTime - lastInterruptTime;
    lastInterruptTime = currentTime;
    interrupt_flag = true;
    interrupt_count++;
}

int main()
{
    stdio_init_all();

    gpio_init(IRQ_PIN);
    gpio_set_dir(IRQ_PIN, GPIO_IN);
    gpio_pull_down(IRQ_PIN);
    gpio_set_irq_enabled_with_callback(IRQ_PIN, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);

    int count = 0;
    while (count++ < 10) {
        printf("(%d) ", count);
        if (interrupt_flag) {
            printf("Hello, %dth interrupted world!\n", interrupt_count);
            printf("  time between Hsync interrupts: %llu\n", timeBetweenInterrupts);
            interrupt_flag = false;
        } else {
            printf("Hello, un-interrupted world!\n");
        }
        sleep_ms(1000);
    }
    return 0;
}
