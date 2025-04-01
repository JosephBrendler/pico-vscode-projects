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
static uint HSYNC_PIN = 15;
// for input gpio; set true for pullup | false for pulldown (one true if needed; else set both false)
static bool PULL_UP = true;
static bool PULL_DOWN = false;

// debug variable for testing
static uint verbosity = 2;

// Define the PIO instance and state machine to use
static PIO PIO_0;
static uint SM;
static uint PIO_IRQ;
static uint pioNum = 0; // use to select PIO and IRQ

 static float SM_CLK_FREQ = (25 * MHZ);
//static float SM_CLK_FREQ = (2 * MHZ);

volatile bool hsync_irq_flag = false;
volatile uint64_t interrupt_count = 0;
volatile uint64_t currentTime = 0;
volatile uint64_t timeBetweenInterrupts = 0;
volatile uint64_t lastInterruptTime = 0;

void hsync_pio_isr() {
    // Clear the interrupt flag
    if (pio_interrupt_get(PIO_0, 0)) // returns TRUE if IRQ 0 is set (bit 8 in IRQ register)
    {
        //printf("Before Clear IRQ0 [INTR]: %x [IRQ]: %x || ", PIO_0->intr, PIO_0->irq);
        pio_interrupt_clear(PIO_0, 0);
        //printf("After Clear IRQ0 [INTR]: %x [IRQ]: %x \n", PIO_0->intr, PIO_0->irq);
        hsync_irq_flag = true;
    }
    else
    {
        printf("Error: got wrong interrupt signal [INTR]: %x [IRQ]: %x \n", PIO_0->intr, PIO_0->irq);
    }
}

void setup() {
    printf("starting setup ...\n");

    // disable irqs while dpomg setup
    irq_set_enabled(PIO_IRQ, false);
    printf("  disabled interrupts for PIO_IRQ: %d while running setup\n", PIO_IRQ);

    // select PIO instance and IRQ pabsed on glogal static pioNum defined at the top
    printf("  assigning PIO and IRQ for pionum: %d\n", pioNum);
    PIO_0 = pioNum ? pio1 : pio0;               // Selects the pio instance (0 or 1 for pioNUM)
    PIO_IRQ = pioNum ? PIO1_IRQ_0 : PIO0_IRQ_0; // Selects the NVIC PIO_IRQ to use
    printf("  assigned PIO: %d, SM IRQ 0, mapped to NVIC PIO_IRQ (0/1): %d\n", pioNum, PIO_IRQ);

    // Load PIO program to monitor rising edge of an acive-low Hsync pulse
    uint offset = pio_add_program(PIO_0, &hsync_program);
    printf("  loaded hsync_program and got offset: %d\n", offset);

    // examine existing SMs on this PIO; nothing else should be useing them, but unclaim them if needed
    for (uint i = 0; i < 4; i++)
    {
        if (pio_sm_is_claimed(PIO_0, i))
        {
            // don't know why it's busy, but set it free
            printf("  SM: %d is claimed, trying to unclaim ...", i);
            pio_sm_unclaim(PIO_0, i);
            printf("  now unclaimed.\n");
        }
        else
        {
            if ( verbosity >= 3 ) { printf("  checked SM: %d, (uncliamed)\n", i); }
        }
    }
    // now claim a state macinge (sm)
    printf("  now trying to claim an unused SM ... ");
    SM = pio_claim_unused_sm(PIO_0, true);
    printf("  claimed SM: %d\n", SM);

    // SM_CLK_FREQ was assigned in the top block; print here to ackowledge as part of setup
    printf("  assigned SM_CLK_FREQ: %.4f\n", SM_CLK_FREQ);
    // calculte the divider required to generate the desired state machine clock frequency
    float sys_clock = clock_get_hz(clk_sys);
    float div =  ( sys_clock / SM_CLK_FREQ ); 
    printf("  div = ( clock_get_hz(clk_sys): %.4f  / SM_CLK_FREQ: %.4f ) = div: %.4f \n", 
        sys_clock, SM_CLK_FREQ, div);

    //  print here to ackowledge as part of setup
    // set HSYNC_PIN (assigned in the top block) as gpio input; configure for pull up/down as applicable
    gpio_set_dir(HSYNC_PIN, false);
    char updown[] = "not set";
    if (PULL_DOWN) { gpio_pull_down(HSYNC_PIN); sprintf(updown, "down"); }
    if (PULL_UP) { gpio_pull_up(HSYNC_PIN); sprintf(updown, "up"); }
    printf("  configured HSYNC_PIN: %d as input, with pullup/down: %s\n", HSYNC_PIN, updown);

    // initialize hsync program on sm
    hsync_program_init(PIO_0, SM, offset, HSYNC_PIN, div);
    printf("  initialized hsync_program on SM: %d, at offset %d, HSYNC_PIN: %d, and SM_CLK_FREQ: %.5f\n", SM, offset, HSYNC_PIN, SM_CLK_FREQ);

    // use the pio_set_irq0_source_enabled API to enable PIO IRQ for the statemachine (set IRQ0_INTE)
    // interrupt enable register.  enumeration is at sdk ref p. 215  (pis_interrupt0 = 8)
    pio_set_irq0_source_enabled(PIO_0, pis_interrupt0, true);
    printf("  enabled irq0_source on pis_interrupt0: %d\n", pis_interrupt0);

    // Set up and enable the interrupt handler - for NVIC interrupts on the CPU
    irq_set_exclusive_handler(PIO_IRQ, &hsync_pio_isr);
    printf("  irq_set_exclusive_handler done for PIO_IRQ: %d, &hsync_pio_isr\n", PIO_IRQ);

    // re-enable irqs after setup -- this enagles the NVIC interrupt on the CPU
    irq_set_enabled(PIO_IRQ, true);
    printf("  irq_set_enabled for PIO_IRQ %d\n", PIO_IRQ);
    
    printf("Done setup\n\n");
}

void loop() {
    printf("running microcontroller loop ...\n");
    int count = 0;
    while (count < 10)
    {
        currentTime = time_us_64();
        if (hsync_irq_flag)
        {
            // disable irqs while handling one
//            irq_set_enabled(PIO_IRQ, false);      // don't use NVIC, just poll INTR in loop
            timeBetweenInterrupts = currentTime - lastInterruptTime;
            lastInterruptTime = currentTime;
            interrupt_count++;
            hsync_irq_flag = false;       // don't need this if not using NVIC IRQ
            // re-enable irqs after handling one    
//            irq_set_enabled(PIO_IRQ, true);           // donnt use NVIC - just poll INTR in loop
        }
        if ( time_us_64() % 1000000 == 0 ) {
            printf("Runtime: (%ds), IRQs: (%llu), Delta_t btw IRQs: (%llu us)\n", count ++, interrupt_count, timeBetweenInterrupts);
        }
    }
    printf("done microcontroller loop ...\n");
}

int main()
{
    // initialize standard io (to get status and debug info; e.g. printf won't work intil this is set up)
    stdio_init_all();
    printf("initialized stdio; starting main program\n");
    setup();
    printf("back in main()\n");
    loop();
    printf("back in main(); program complete\n");
    return 0;
}