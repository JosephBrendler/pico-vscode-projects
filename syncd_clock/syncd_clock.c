#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/rosc.h"
#include "hardware/pll.h"
#include "hardware/timer.h"
#include "hardware/regs/rosc.h"
#include "hardware/structs/rosc.h"
#include "hardware/structs/pll.h"
#include "hardware/structs/clocks.h"

#include "blink.pio.h"

// definitions ---------------------------------------------------
#define PLL_SYS_KHZ (133 * 1000) // system clock defined as 133 MHz
#define IRQ_PIN 15
#define BLINK_PIN 25
#define Rosc_COUNTER 5   // 133 MHz / 5 = 26.6 Mhz

//#define Rosc_IRQ_IRQ_NR 17   // 17 is clock IRQ

// variables ----------------------------------------------------
volatile float div = 1.0; // clock divider use to set clk_sys after clk_peri is tied to PLL

volatile bool hsync_flag = false;

volatile uint64_t lastHsyncInterruptTime = 0;
volatile uint64_t timeBetweenHsyncInterrupts = 0;
volatile uint64_t HsyncTime = 0;
volatile uint64_t HsyncCount = 0;

volatile bool Rosc_flag = false;

volatile uint64_t lastRoscInterruptTime = 0;
volatile uint64_t timeBetweenRoscInterrupts = 0;
volatile uint64_t RoscTime = 0;
volatile uint64_t RoscCount = 0;// volatile float div                              = 6.5;     // clock divider use to set clk_sys after clk_peri is tied to PLL

// uint blink_freq = 3;
uint blink_freq = 4;

// functions ----------------------------------------------------

void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, uint freq)
{
    blink_program_init(pio, sm, offset, pin);
    pio_sm_set_enabled(pio, sm, true);

    printf("Blinking pin %d at %d Hz\n", pin, freq);

    // PIO counter program takes 3 more cycles in total than we pass as
    // input (wait for n + 1; mov; jmp)

    pio->txf[sm] = ((133 * MHZ) / (2 * freq)) - 3;
}

void gpio_callback(uint gpio, uint32_t events)
{
    HsyncTime = time_us_64();
    timeBetweenHsyncInterrupts = HsyncTime - lastHsyncInterruptTime;
    lastHsyncInterruptTime = HsyncTime;
    hsync_flag = true;
    HsyncCount++;
}

void set_Rosc_counter(uint32_t count) {
    // Disable the Rosc during configuration
    Rosc_hw->ctrl &= ~Rosc_CTRL_ENABLE_BITS;

    // Set the desired counter value
    Rosc_hw->startup = count;

    //Re-enable the Rosc
    Rosc_hw->ctrl |= Rosc_CTRL_ENABLE_BITS;

    // Wait for Rosc to stabilize (optional, but recommended)
    while (!(Rosc_hw->status & Rosc_STATUS_STABLE_BITS));
}

// Interrupt handler for Rosc counter
void Rosc_interrupt_handler() {
    // Clear the interrupt flag
//    Rosc_clear_interrupt();
    // set software flag instead
    RoscTime = time_us_64();
    timeBetweenRoscInterrupts = RoscTime - lastRoscInterruptTime;
    lastRoscInterruptTime = RoscTime;
    Rosc_flag = true;
    RoscCount++;
}

void measure_freqs(void) {
    uint f_pll_sys = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_PLL_SYS_CLKSRC_PRIMARY);
    uint f_pll_usb = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_PLL_USB_CLKSRC_PRIMARY);
    uint f_rosc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_ROSC_CLKSRC);
    uint f_clk_sys = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS);
    uint f_clk_peri = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_PERI);
    uint f_clk_usb = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_USB);
    uint f_clk_adc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_ADC);
#ifdef CLOCKS_FC0_SRC_VALUE_CLK_RTC
    uint f_clk_rtc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_RTC);
#endif

    printf("pll_sys  = %dkHz\n", f_pll_sys);
    printf("pll_usb  = %dkHz\n", f_pll_usb);
    printf("rosc     = %dkHz\n", f_rosc);
    printf("clk_sys  = %dkHz\n", f_clk_sys);
    printf("clk_peri = %dkHz\n", f_clk_peri);
    printf("clk_usb  = %dkHz\n", f_clk_usb);
    printf("clk_adc  = %dkHz\n", f_clk_adc);
#ifdef CLOCKS_FC0_SRC_VALUE_CLK_RTC
    printf("clk_rtc  = %dkHz\n", f_clk_rtc);
#endif

    // Can't measure clk_ref / xosc as it is the ref
}

int main()
{
    // Set the system frequency to 133 MHz. vco_calc.py from the SDK tells us
    // this is exactly attainable at the PLL from a 12 MHz crystal: FBDIV =
    // 133 (so VCO of 1596 MHz), PD1 = 6, PD2 = 2. This function will set the
    // system PLL to 133 MHz and set the clk_sys divisor to 1.
    set_sys_clock_khz(PLL_SYS_KHZ, true);

    // The previous line automatically detached clk_peri from clk_sys, and
    // attached it to pll_usb, so that clk_peri won't be disturbed by future
    // changes to system clock or system PLL. If we need higher clk_peri
    // frequencies, we can attach clk_peri directly back to system PLL (no
    // divider available) and then use the clk_sys divider to scale clk_sys
    // independently of clk_peri.
    clock_configure(
        clk_peri,
        0,                                                // No glitchless mux
        CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS, // System PLL on AUX mux
        PLL_SYS_KHZ * 1000,                               // Input frequency
        PLL_SYS_KHZ * 1000                                // Output (must be same as no divider)
    );

    // The serial clock won't vary from this point onward, so we can configure
    // the UART etc.

    stdio_init_all();

    puts("stdio initialized.\nPeripheral clock is attached directly to system PLL (peri_clk).");
    puts("We can vary the system clock divisor (and thus sys_clk) w/o affecting peri_clk");
    puts("Thus we can print reliably with the UART.\n");

    // configure clocks
    /*/ ---------------- sample freqs per div choice ----------------------
    //for ( div = 1.0; div <= 10.0; div+=0.5) {
    //for ( div = 6.0; div <= 6.5; div+=0.01) {
    // 0.00390625 = 1/256 - the smallest possible increment
    for ( div = 6.4; div <= 6.5; div+=0.00390625) {
        printf("Setting system clock divisor to %.5f\n", div);
            clock_configure(
                clk_sys,
                CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
                CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
                PLL_SYS_KHZ,
                PLL_SYS_KHZ / div
            );
            printf("Measuring system clock with frequency counter:");
            // Note that the numbering of frequency counter sources is not the
            // same as the numbering of clock slice register blocks. (If we passed
            // the clk_sys enum here we would actually end up measuring XOSC.)
            printf("%u kHz\n", frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS));
            // For more examples of clocks use see https://github.com/raspberrypi/pico-examples/tree/master/clocks
        }
    //------------------------------------------------------------------------*/

    // reconfigure clk_sys with divider (float div) set in variable declarations above
    printf("Setting system clock with divisor: %.5f\n", div);
    clock_configure(
        clk_sys,
        CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
        CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
        PLL_SYS_KHZ,
        PLL_SYS_KHZ / div);

    printf("Measuring system clock with frequency counter: ");
    // Note that the numbering of frequency counter sources is not the
    // same as the numbering of clock slice register blocks. (If we passed
    // the clk_sys enum here we would actually end up measuring XOSC.)
    printf("%u kHz\n", frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS));
    // For more examples of clocks use see https://github.com/raspberrypi/pico-examples/tree/master/clocks

    // use function above to measure all freqs
    measure_freqs();

    // set up pio for blinking
    PIO pio = pio0;
    uint offset = pio_add_program(pio, &blink_program);
    printf("Loaded program at %d\n", offset);

    blink_pin_forever(pio, 0, offset, BLINK_PIN, blink_freq);
    // For more pio examples see https://github.com/raspberrypi/pico-examples/tree/master/pio

    // set up gpio IRQ
    gpio_init(IRQ_PIN);
    gpio_set_dir(IRQ_PIN, GPIO_IN);
    gpio_pull_down(IRQ_PIN);
    gpio_set_irq_enabled_with_callback(IRQ_PIN, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);

    // Enable the Rosc counter interrupt
    irq_set_exclusive_handler(ROSC_IRQ_IRQ_NR, rosc_interrupt_handler); // Set the interrupt handler
    irq_set_enabled(ROSC_IRQ_IRQ_NR, true); // Enable the interrupt in the NVIC
//    irq_set_exclusive_handler(ROSC_IRQ_IRQn, rosc_interrupt_handler); // Set the interrupt handler
//    irq_set_enabled(rosc_IRQ_IRQn, true); // Enable the interrupt in the NVIC

    // Configure the rosc counter (example: interrupt every second)
 //   rosc_setup(); // Initialize rosc
 //   rosc_enable_counter_interrupt(ROSC_MHZ); // Set interrupt at 1 MHz intervals, adjust as needed

    // Example: Set the Rosc counter to 1000
    set_Rosc_counter(ROSC_COUNTER);
    printf("set rosc_counter to %d\n", ROSC_COUNTER);

    // setup complete - run program
    // for now, run for 20 seconds
    uint count = 0;
    while (count < 10)
    {
        if (time_us_64() % 1000000 == 0)
        { // once a second, do this
            if (hsync_flag)
            {
                printf("(%d) Hello, %dth hsync-interrupted world!\n", count++, HsyncCount);
                printf("Time difference between Hsync interrupts: %llu microseconds\n", timeBetweenHsyncInterrupts);
                hsync_flag = false;
            } else {
                printf("(%d) Hello, un-hsync-interrupted world!\n", count);
            }
            if (rosc_flag)
            {
                printf("(%d) Hello, %dth rosc-interrupted world!\n", count++, RoscCount);
                printf("Time difference between Hsync interrupts: %llu microseconds\n", timeBetweenRoscInterrupts);
                rosc_flag = false;
            } else {
                printf("(%d) Hello, un-rosc-interrupted world!\n", count);
            }
        } // end if time
    } // end while count++ < 10
} // end main
