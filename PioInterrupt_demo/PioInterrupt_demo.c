// C code
/*
PIO Ep. 19 PIO IRQs
PIO Demo 19A - Demonstrating ID and clearing of IRQ 0 through 4 on one state machine
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "PioInterrupt_demo.pio.h"

// if using sm IRQs 4-7, they cannot generate NVIC IRQs to cpu, so we must poll instead of use isr/callback
// #define IRQ_CALLBACK true
#define IRQ_CALLBACK false

static PIO PIO_O;    // pio object
static uint SM;      // pio state machine index
static uint PIO_IRQ; // NVIC ARM CPU interrupt number
// int x;
// char* pointer = (char*)0x50200128; //pointer for PIO0 INTR register
// char* pointerNVIC = (char*)(PPB_BASE + M0PLUS_NVIC_ICPR_OFFSET); //pointer for NVIC ICPR register

static float SM_CLK_FREQ = 5000;

void print_binary(int number)
{
    char oldbinary[] = "";
    char binary[] = "";
    while (number > 0)
    {
        if (number % 2)
        {
            sprintf(binary, "1%s", oldbinary);
            sprintf(oldbinary, "%s", binary);
        }
        else
        {
            sprintf(binary, "0%s", oldbinary);
            sprintf(oldbinary, "%s", binary);
        }
        number /= 2;
    }
    printf("%s\n", binary);
}

void pioIRQ()
{ // This is our callback/polling function

    // could use this to set up a case statement instead - but only works for
    // printf("[IRQ] %d - ", PIO_O->irq & 0b1111);   // use this when only using callback irqs (0-3)
    uint8_t irq_status_bits = PIO_O->irq; // when polling, this wont be set at all, so just skip out
    if (irq_status_bits)
    {
        printf("[IRQ] %d - ", irq_status_bits);

        if (pio_interrupt_get(PIO_O, 0)) // returns TRUE if IRQ 0 is set
        {
            printf("IRQ0 - Before clearing [INTR]: %x [IRQ]: %x; ", PIO_O->intr, PIO_O->irq);
            pio_interrupt_clear(PIO_O, 0);
            printf("After clearing [INTR]: %x [IRQ]: %x \n", PIO_O->intr, PIO_O->irq);
        }
        else if (pio_interrupt_get(PIO_O, 1)) // returns TRUE if IRQ 1 is set
        {
            printf("IRQ1 - Before clearing [INTR]: %x [IRQ]: %x; ", PIO_O->intr, PIO_O->irq);
            pio_interrupt_clear(PIO_O, 1);
            printf("After clearing [INTR]: %x [IRQ]: %x \n", PIO_O->intr, PIO_O->irq);
        }
        else if (pio_interrupt_get(PIO_O, 2)) // returns TRUE if IRQ 2 is set
        {
            printf("IRQ2 - Before clearing [INTR]: %x [IRQ]: %x; ", PIO_O->intr, PIO_O->irq);
            pio_interrupt_clear(PIO_O, 2);
            printf("After clearing [INTR]: %x [IRQ]: %x \n", PIO_O->intr, PIO_O->irq);
        }
        else if (pio_interrupt_get(PIO_O, 3)) // returns TRUE if IRQ 3 is set
        {
            printf("IRQ3 - Before clearing [INTR]: %x [IRQ]: %x; ", PIO_O->intr, PIO_O->irq);
            pio_interrupt_clear(PIO_O, 3);
            printf("After clearing [INTR]: %x [IRQ]: %x \n", PIO_O->intr, PIO_O->irq);
        }
        else if (pio_interrupt_get(PIO_O, 4)) // returns TRUE if IRQ 3 is set
        {
            printf("IRQ4 - Before clearing [INTR]: %x [IRQ]: %x; ", PIO_O->intr, PIO_O->irq);
            pio_interrupt_clear(PIO_O, 4);
            printf("After clearing [INTR]: %x [IRQ]: %x \n", PIO_O->intr, PIO_O->irq);
        }
        else if (pio_interrupt_get(PIO_O, 5)) // returns TRUE if IRQ 3 is set
        {
            printf("IRQ5 - Before clearing [INTR]: %x [IRQ]: %x; ", PIO_O->intr, PIO_O->irq);
            pio_interrupt_clear(PIO_O, 5);
            printf("After clearing [INTR]: %x [IRQ]: %x \n", PIO_O->intr, PIO_O->irq);
        }
        else if (pio_interrupt_get(PIO_O, 6)) // returns TRUE if IRQ 3 is set
        {
            printf("IRQ6 - Before clearing [INTR]: %x [IRQ]: %x; ", PIO_O->intr, PIO_O->irq);
            pio_interrupt_clear(PIO_O, 6);
            printf("After clearing [INTR]: %x [IRQ]: %x \n", PIO_O->intr, PIO_O->irq);
        }
        else if (pio_interrupt_get(PIO_O, 7)) // returns TRUE if IRQ 3 is set
        {
            printf("IRQ7 - Before clearing [INTR]: %x [IRQ]: %x; ", PIO_O->intr, PIO_O->irq);
            pio_interrupt_clear(PIO_O, 7);
            printf("After clearing [INTR]: %x [IRQ]: %x \n", PIO_O->intr, PIO_O->irq);
        }
    }
}

void testIRQPIO(uint pioNum)
{
    printf("starting test for pionum: %d\n", pioNum);
    PIO_O = pioNum ? pio1 : pio0;               // Selects the pio instance (0 or 1 for pioNUM)
    PIO_IRQ = pioNum ? PIO1_IRQ_0 : PIO0_IRQ_0; // Selects the NVIC PIO_IRQ to use
    printf("assigned NVIC PIO_IRQ (0/1): %d\n", PIO_IRQ);

    // Our assembled program needs to be loaded into this PIO's instruction
    // memory. This SDK function will find a location (offset) in the
    // instruction memory where there is enough space for our program. We need
    // to remember this location!
    uint offset = pio_add_program(PIO_O, &pioIRQ_program);
    printf("assigned offset: %d\n", offset);

    // Assign GPIO0 as the blinking LED
    uint BLINK_LED_PIN = 25;
    printf("assigned BLINK_LED_PIN: %d\n", BLINK_LED_PIN);

    // select the desired state machine clock frequency (2000 is about the lower limit)
    printf("assigned SM_CLK_FREQ: %.4f\n", SM_CLK_FREQ);

    // Find a free state machine on our chosen PIO (erroring if there are
    // none). Configure it to run our program, and start it, using the
    // helper function we included in our .pio file.
    SM = pio_claim_unused_sm(PIO_O, true);
    pioIRQ_program_init(PIO_O, SM, offset, BLINK_LED_PIN, SM_CLK_FREQ);
    printf("completed pioIRQ_program_init\n");

    if (IRQ_CALLBACK)
    { // eliminate all callback assignments if POLLING instead
        // enables IRQ for the statemachine - setting IRQ0_INTE - interrupt enable register
        // pio_set_irq0_source_enabled(PIO_O, pis_interrupt0, true); // sets IRQ0
        // pio_set_irq0_source_enabled(PIO_O, pis_interrupt1, true); // sets IRQ1
        // pio_set_irq0_source_enabled(PIO_O, pis_interrupt2, true); // sets IRQ2
        // pio_set_irq0_source_enabled(PIO_O, pis_interrupt3, true); // sets IRQ3
        //*********or************
        int mask = 3840;                                     // setting all 4 at once
        pio_set_irq0_source_mask_enabled(PIO_O, mask, true); // setting all 4 at once
        printf("completed pio_set_irq0_source_mask_enabled with mask = dec: %d | hex: %x | binary: ", mask, mask);
        print_binary(mask);

        irq_set_exclusive_handler(PIO_IRQ, pioIRQ); // Set the handler in the NVIC
        irq_set_enabled(PIO_IRQ, true);             // enabling the PIO1_IRQ_0
    }

    printf("done test for pioNum: %d\n\n", pioNum);
}

int main(void)
{
    stdio_init_all();
    // sleep_ms(20000);  //gives me time to start PuTTY
    printf("start program\n");
    testIRQPIO(0);
    printf("PIO: %d, SM: %d, PIO_IRQ_num: %d \n", pio_get_index(PIO_O), SM, PIO_IRQ);

    while (1)
    {
        sleep_ms(200);
        printf("."); // watchdog printout
        if (!IRQ_CALLBACK)
        {             // if using hiher irqs, we must poll instead of callback
            pioIRQ(); // since loop sleeps 200ms, that determines the polling frequencdy
        }
    }

    return 0;
}