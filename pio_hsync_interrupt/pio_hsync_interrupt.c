#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"

// Define the GPIO pin to monitor
#define GPIO_PIN 15

// Define the PIO instance and state machine to use
//#define PIO_INSTANCE pio0
static PIO PIO_INSTANCE = pio0;
#define SM 0

volatile bool hsync_irq_flag = false;
volatile uint64_t interrupt_count = 0;
volatile uint64_t currentTime = 0;
volatile uint64_t timeBetweenInterrupts = 0;
volatile uint64_t lastInterruptTime = 0;
volatile bool interrupt_flag = false;

// Interrupt handler function
void pio_irq_handler() {
    // Clear the interrupt flag
    pio_interrupt_clear(PIO_INSTANCE, SM);
    currentTime = time_us_64();
    timeBetweenInterrupts = currentTime - lastInterruptTime;
    lastInterruptTime = currentTime;
    interrupt_flag = true;
    interrupt_count++;
}

int main() {
    stdio_init_all();

    // Configure the GPIO pin as input with pull-down resistor
    gpio_init(GPIO_PIN);
    gpio_set_dir(GPIO_PIN, GPIO_IN);
    gpio_pull_down(GPIO_PIN);

    // Load PIO program to monitor rising edge
    uint offset = pio_add_program(PIO_INSTANCE, &hsync_program); 
    pio_sm_config c = program_get_default_config( );

   // Set the input pins
    sm_config_set_in_pins(&c, GPIO_PIN);

    // Configure the state machine
    sm_config_set_wrap(&c, offset, offset + program.length - 1);
    sm_config_set_clkdiv(&c, 1);
    sm_config_set_in_shift(&c, false, false, 32);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_NONE);
    pio_sm_init(PIO_INSTANCE, SM, offset, &c);

    // Enable interrupt on rising edge
    pio_sm_set_enabled(PIO_INSTANCE, SM, true);
    pio_set_irq0_source_enabled(PIO_INSTANCE, SM, true);

    // Set up and enable the interrupt handler
    irq_set_exclusive_handler(PIO0_IRQ_0, pio_irq_handler);
    irq_set_enabled(PIO0_IRQ_0, true);

    while (1) {
        // Your main program logic here
        if (interrupt_flag) {
            printf("Hello, %dth interrupted world!\n", interrupt_count);
            printf("  time between Hsync interrupts: %llu\n", timeBetweenInterrupts);
            interrupt_flag = false;
        }
        sleep_ms(1000);
    }
}