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

// debug variable for testing
static uint verbosity = 3;

// Define the PIO instance and state machine to use
static PIO PIO_0;
static uint SM;
static uint PIO_IRQ;
static uint pioNum = 0;   // use to select PIO and IRQ

static float SM_CLK_FREQ = ( 25 * MHZ );

volatile bool hsync_irq_flag = false;
volatile uint64_t interrupt_count = 0;
volatile uint64_t currentTime = 0;
volatile uint64_t timeBetweenInterrupts = 0;
volatile uint64_t lastInterruptTime = 0;
volatile bool interrupt_flag = false;

// Interrupt handler function
void hsync_pio_isr() {
    // disable irqs while handling one
    irq_set_enabled(PIO_IRQ, false);
    sleep_ms(100);
    // degug: print irq status bits, confirm what irq got us here
    if (verbosity >= 3) { printf("[IRQ] %d - ", PIO_0->irq); }
    // Clear the interrupt flag
    if (pio_interrupt_get(PIO_0, 0)) // returns TRUE if IRQ 0 is set
    {
        if (verbosity >= 3) { printf("IRQ0 - Before clearing [INTR]: %x [IRQ]: %x; ", PIO_0->intr, PIO_0->irq); }
        pio_interrupt_clear(PIO_0, 0);
        if (verbosity >= 3) { printf("After clearing [INTR]: %x [IRQ]: %x \n", PIO_0->intr, PIO_0->irq); }
    } else {
        printf("Error: got wrong interrupt signal [INTR]: %x [IRQ]: %x \n", PIO_0->intr, PIO_0->irq);
    }

    currentTime = time_us_64();
    printf("currentTime: %llu\n", currentTime);
    
    timeBetweenInterrupts = currentTime - lastInterruptTime;
    lastInterruptTime = currentTime;
    interrupt_flag = true;
    interrupt_count++;
    // re-enable irqs after handling one
    irq_set_enabled(PIO_IRQ, true);
    sleep_ms(100);
}

int main() {
    stdio_init_all();

    // move this to pio helper function?
//    gpio_set_dir(GPIO_PIN, false); // set gpio as input for sync pin
//    gpio_pull_down(GPIO_PIN);       // maybe make this pull up (since it drives low)
//    gpio_pull_up(GPIO_PIN);       // maybe make this pull up (since it drives low)

    printf("start program\n");

    printf("assigning PIO and IRQ for pionum: %d\n", pioNum);
    PIO_0 = pioNum ? pio1 : pio0;               // Selects the pio instance (0 or 1 for pioNUM)
    PIO_IRQ = pioNum ? PIO1_IRQ_0 : PIO0_IRQ_0; // Selects the NVIC PIO_IRQ to use
    printf("assigned PIO %d, SM IRQ 0, mapped to NVIC PIO_IRQ (0/1): %d\n", pioNum, PIO_IRQ);

    // Load PIO program to monitor rising edge
    uint offset = pio_add_program(PIO_0, &hsync_program); 
    printf("loaded hsync_program and got offset: %d\n", offset);

    // select the desired state machine clock frequency (2000 is about the lower limit)
    printf("assigned SM_CLK_FREQ: %.4f\n", SM_CLK_FREQ);
    float div = clock_get_hz(clk_sys) / SM_CLK_FREQ ;  // calculates the clock divider
    printf("helper function will calculate divider div (clk_sys / SM_CLK_FREQ = %.4f)", div);
 
    // assign hsync pin
    printf("assigned Hsync GPIO_PIN: %d\n", GPIO_PIN);


    // disable irqs while dpomg seti[]
    irq_set_enabled(PIO_IRQ, false);
    sleep_ms(100);

    // examine existing SMs on this PIO
    for (uint i = 0; i < 4; i++) {
        if (pio_sm_is_claimed(PIO_0, i)){
            // don't know why it's busy, but set it free
            printf("SM: %d is claimed, trying to unclaim ...", i);
            pio_sm_unclaim(PIO_0, i);
            printf("now unclaimed.\n");
        } else {
            printf("checked SM: %d, (uncliamed)\n", i);
        }
    }
    printf("now trying to claim an unused SM ... ");
    SM = pio_claim_unused_sm(PIO_0, true);
    printf("claimed SM: %d\n", SM);
    // initialize hsync program on sm
    hsync_program_init(PIO_0, SM, offset, GPIO_PIN, SM_CLK_FREQ);
    printf("initialized hsync_program on SM: %d, at offset %d, GPIO_PIN: %d, and SM_CLK_FREQ: %d\n", SM, offset, GPIO_PIN, SM_CLK_FREQ);

    // enables IRQ for the statemachine - setting IRQ0_INTE - interrupt enable register
    // pio_set_irq0_source_enabled(PIO_O, pis_interrupt0, true); // sets NVIC IRQ0
    // enumeration is at sdk ref p. 215
    pio_set_irq0_source_enabled(PIO_0, pis_interrupt0, true);
    printf("enabled irq0_source on pis_interrupt0: %d\n", pis_interrupt0);

    // Set up and enable the interrupt handler
    irq_set_exclusive_handler(PIO_IRQ, &hsync_pio_isr);
    printf("irq_set_exclusive_handler done for PIO_IRQ: %d, &hsync_pio_isr\n", PIO_IRQ);

    // re-enable irqs after setup
    irq_set_enabled(PIO_IRQ, true);
    sleep_ms(100);
    printf("irq_set_enabled for PIO_IRQ %d\n", PIO_IRQ);
    printf("Done setup\n\n");

    int count = 0;
    while (count++ < 20) {
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