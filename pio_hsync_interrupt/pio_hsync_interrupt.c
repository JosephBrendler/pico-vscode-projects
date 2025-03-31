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
#define HSYNC_PIN 15

// debug variable for testing
static uint verbosity = 2;

// Define the PIO instance and state machine to use
static PIO PIO_0;
static uint SM;
static uint PIO_IRQ;
static uint pioNum = 0; // use to select PIO and IRQ

// static float SM_CLK_FREQ = (25 * MHZ);
static float SM_CLK_FREQ = (2 * MHZ);

volatile bool hsync_irq_flag = false;
volatile uint64_t interrupt_count = 0;
volatile uint64_t currentTime = 0;
volatile uint64_t timeBetweenInterrupts = 0;
volatile uint64_t lastInterruptTime = 0;
volatile bool interrupt_flag = false;

// Interrupt handler function
void hsync_pio_isr()
{
    // degug: print irq status bits, confirm what irq got us here
//        printf("[IRQ] %d - ", PIO_0->irq);
    // now just set a flag and let main() do math/ops
    interrupt_flag = true;
}

void clear_irq() {
    // Clear the interrupt flag
    if (pio_interrupt_get(PIO_0, 0)) // returns TRUE if IRQ 0 is set
    {
        printf("Before Clear IRQ0 [INTR]: %x [IRQ]: %x || ", PIO_0->intr, PIO_0->irq);
        pio_interrupt_clear(PIO_0, 0);
        printf("After Clear IRQ0 [INTR]: %x [IRQ]: %x \n", PIO_0->intr, PIO_0->irq);
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
    printf("  disabled interrupts for PIO_IRQ: %d while running setup", PIO_IRQ);

    // move this to pio helper function?
    gpio_set_dir(HSYNC_PIN, false); // set gpio as input for sync pin
    //    gpio_pull_down(HSYNC_PIN);       // maybe make this pull up (since it drives low)
    gpio_pull_up(HSYNC_PIN);        // maybe make this pull up (since it drives low)

    // select PIO instance and IRQ pabsed on glogal static pioNum defined at the top
    printf("  assigning PIO and IRQ for pionum: %d\n", pioNum);
    PIO_0 = pioNum ? pio1 : pio0;               // Selects the pio instance (0 or 1 for pioNUM)
    PIO_IRQ = pioNum ? PIO1_IRQ_0 : PIO0_IRQ_0; // Selects the NVIC PIO_IRQ to use
    printf("  assigned PIO %d, SM IRQ 0, mapped to NVIC PIO_IRQ (0/1): %d\n", pioNum, PIO_IRQ);

    // Load PIO program to monitor rising edge
    uint offset = pio_add_program(PIO_0, &hsync_program);
    printf("  loaded hsync_program and got offset: %d\n", offset);

    // select the desired state machine clock frequency (2000 is about the lower limit)
    printf("  assigned SM_CLK_FREQ: %.4f\n", SM_CLK_FREQ);
    float div = clock_get_hz(clk_sys) / SM_CLK_FREQ; // calculates the clock divider
    printf("  helper function will calculate divider div (clk_sys / SM_CLK_FREQ = %.4f\n)", div);

    // assign hsync pin
    printf("  assigned Hsync HSYNC_PIN: %d\n", HSYNC_PIN);

    // examine existing SMs on this PIO
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
            printf("  checked SM: %d, (uncliamed)\n", i);
        }
    }
    printf("  now trying to claim an unused SM ... ");
    SM = pio_claim_unused_sm(PIO_0, true);
    printf("  claimed SM: %d\n", SM);
    // initialize hsync program on sm
    hsync_program_init(PIO_0, SM, offset, HSYNC_PIN, SM_CLK_FREQ);
    printf("  initialized hsync_program on SM: %d, at offset %d, HSYNC_PIN: %d, and SM_CLK_FREQ: %d\n", SM, offset, HSYNC_PIN, SM_CLK_FREQ);

    // enables IRQ for the statemachine - setting IRQ0_INTE - interrupt enable register
    // pio_set_irq0_source_enabled(PIO_O, pis_interrupt0, true); // sets PIO IRQ0 (bit 8 of INTR is src)
    // enumeration is at sdk ref p. 215  (pis_interrupt0 = 8)
    pio_set_irq0_source_enabled(PIO_0, pis_interrupt0, true);
    printf("  enabled irq0_source on pis_interrupt0: %d\n", pis_interrupt0);

    // Set up and enable the interrupt handler - for NVIC interrupts on the CPU
//    irq_set_exclusive_handler(PIO_IRQ, &hsync_pio_isr);
//    printf("  irq_set_exclusive_handler done for PIO_IRQ: %d, &hsync_pio_isr\n", PIO_IRQ);
// disabled isr() for now - poll in loop

    // re-enable irqs after setup -- this enagles the NVIC interrupt on the CPU
//    irq_set_enabled(PIO_IRQ, true);
//    printf("  irq_set_enabled for PIO_IRQ %d\n", PIO_IRQ);
    printf("Done setup\n\n");
}

void loop() {
    printf("running microcontroller loop ...");
    int count = 0;
    while (count < 60)
    {
//        if (interrupt_flag)   // disabled isr() for nwo - poll here
        if (pio_interrupt_get(PIO_0, 0))
        {
            // disable irqs while handling one
//            irq_set_enabled(PIO_IRQ, false);      // don't use NVIC, just poll INTR in loop

            // do math/ops/debug ...
            currentTime = time_us_64();
            printf("currentTime: %llu\n", currentTime);
            interrupt_count++;
            timeBetweenInterrupts = currentTime - lastInterruptTime;
            lastInterruptTime = currentTime;
//            interrupt_flag = false;       // don't need this if not using NVIC IRQ

            clear_irq();

            // re-enable irqs after handling one    
//            irq_set_enabled(PIO_IRQ, true);           // donnt use NVIC - just poll INTR in loop
        }
        else
        {
            printf("Hello, un-interrupted world!\n");
        }
        if ( currentTime % 1000000 ) {
            printf("Seconds: (%d), IRQs: (%llu), Delta t: (%llu us)\n\n", count ++, interrupt_count, timeBetweenInterrupts);
        }
    }
    printf("done microcontroller loop ...\n");
}

int main()
{
    printf("starting main program\n");
    // initialize standard io (tp get status and debug info)
    stdio_init_all();
    printf("initialized stdio\n");
    setup();
    printf("back in main()\n");
    loop();
    printf("done main program\n");
    return 0;
}