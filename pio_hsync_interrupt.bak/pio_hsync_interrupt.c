/****************************
    pio_hsync_interrupt.c
****************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"

#include "hsync.pio.h"

// Define the GPIO pin to monitor
#define GPIO_PIN 15

// Define the PIO instance and state machine to use
static PIO PIO_0;
static uint SM;

static uint64_t SM_CLK_FREQ = ( 25 * MHZ );

volatile bool hsync_irq_flag = false;
volatile uint64_t interrupt_count = 0;
volatile uint64_t currentTime = 0;
volatile uint64_t timeBetweenInterrupts = 0;
volatile uint64_t lastInterruptTime = 0;
volatile bool interrupt_flag = false;

// Interrupt handler function
void hsync_pio_isr() {
    // degug: print irq status bits, confirm what irq got us here
    printf("[IRQ] %d - ", PIO_0->irq);
    // Clear the interrupt flag
    if (pio_interrupt_get(PIO_0, 0)) // returns TRUE if IRQ 0 is set
    {
        printf("IRQ0 - Before clearing [INTR]: %x [IRQ]: %x; ", PIO_0->intr, PIO_0->irq);
        pio_interrupt_clear(PIO_0, 0);
        printf("After clearing [INTR]: %x [IRQ]: %x \n", PIO_0->intr, PIO_0->irq);
    } else {
        printf("got wrong interrupt signal [INTR]: %x [IRQ]: %x \n", PIO_0->intr, PIO_0->irq);
    }

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
    uint offset = pio_add_program(PIO_0, &hsync_program); 
    // claim pio0 sm0 as ours
    hsync_pio_sm_claim(PIO_O, 0);
    // initialize hsync program on sm
    hsync_program_init(PIO_O, 0, offset, GPIO_PIN, SM_CLK_FREQ);

    // enables IRQ for the statemachine - setting IRQ0_INTE - interrupt enable register
    // pio_set_irq0_source_enabled(PIO_O, pis_interrupt0, true); // sets NVIC IRQ0
    pio_set_irq0_source_enabled(PIO_0, SM, true);

    // Set up and enable the interrupt handler
    irq_set_exclusive_handler(PIO0_IRQ_0, &hsync_pio_isr);
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