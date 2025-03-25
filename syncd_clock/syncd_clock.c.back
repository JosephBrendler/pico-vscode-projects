#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"
#include "hardware/uart.h"

#include "blink.pio.h"

// definitions ---------------------------------------------------
#define PLL_SYS_KHZ (133 * 1000)  // system clock defined as 133 MHz
#define IRQ_PIN 15

// variables ----------------------------------------------------
volatile uint64_t alarm_time = 0;

volatile bool timer_flag                        = false;
volatile bool hsync_flag                        = false;

volatile uint64_t lastHsyncInterruptTime        = 0;
volatile uint64_t timeBetweenHsyncInterrupts    = 0;
volatile uint64_t HsyncTime                     = 0;

volatile uint64_t lastTimerInterruptTime        = 0;
volatile uint64_t timeBetweenTimerInterrupts    = 0;
volatile uint64_t TimerTime                     = 0;

volatile float div                              = 6.5;     // clock divider use to set clk_sys after clk_peri is tied to PLL

long delta = 0;  // difference between hsync pulse and alarm, in us

// functions ----------------------------------------------------

void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, uint freq) {
    blink_program_init(pio, sm, offset, pin);
    pio_sm_set_enabled(pio, sm, true);

    printf("Blinking pin %d at %d Hz\n", pin, freq);

    // PIO counter program takes 3 more cycles in total than we pass as
    // input (wait for n + 1; mov; jmp)

    pio->txf[sm] = ((133 * MHZ) / (2 * freq)) - 3;
}

void gpio_callback(uint gpio, uint32_t events) {
    hsync_flag = true;
    HsyncTime = time_us_64();
    timeBetweenHsyncInterrupts = HsyncTime - lastHsyncInterruptTime;
    lastHsyncInterruptTime = HsyncTime;
}

int64_t alarm_callback(alarm_id_t id, void *user_data) {
    timer_flag = true;
    TimerTime = time_us_64();
    timeBetweenTimerInterrupts = TimerTime - lastTimerInterruptTime;
    lastTimerInterruptTime = TimerTime;
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
// ---------------- sample freqs per div choice ----------------------
//for ( div = 1.0; div <= 10.0; div+=0.5) {
//for ( div = 6.0; div <= 6.5; div+=0.01) {
for ( div = 6.484; div <= 6.485; div+=0.0001) {
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

        printf("System Clock Frequency is %d Hz\n", clock_get_hz(clk_sys));
        printf("USB Clock Frequency is %d Hz\n", clock_get_hz(clk_usb));
        printf("Peripheral Clock Frequency is %d Hz\n\n", clock_get_hz(clk_peri));
        // For more examples of clocks use see https://github.com/raspberrypi/pico-examples/tree/master/clocks
    }
//------------------------------------------------------------------------*/
/*
    // reconfigure clk_sys with divider (float div) set in variable declarations above
    printf("Setting system clock with divisor: %.5f\n", div);
    clock_configure(
        clk_sys,
        CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
        CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
        PLL_SYS_KHZ,
        PLL_SYS_KHZ / div
    );
*/

    printf("Measuring system clock with frequency counter: ");
    // Note that the numbering of frequency counter sources is not the
    // same as the numbering of clock slice register blocks. (If we passed
    // the clk_sys enum here we would actually end up measuring XOSC.)
    printf("%u kHz\n", frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS));

    printf("System Clock Frequency is %d Hz\n", clock_get_hz(clk_sys));
    printf("USB Clock Frequency is %d Hz\n", clock_get_hz(clk_usb));
    printf("Peripheral Clock Frequency is %d Hz\n", clock_get_hz(clk_usb));
    // For more examples of clocks use see https://github.com/raspberrypi/pico-examples/tree/master/clocks

    // set up pio for blinking
    PIO pio = pio0;
    uint offset = pio_add_program(pio, &blink_program);
    printf("Loaded program at %d\n", offset);

    #ifdef PICO_DEFAULT_LED_PIN    // GPIO 25 on pico/rp2040
    blink_pin_forever(pio, 0, offset, PICO_DEFAULT_LED_PIN, 3);
    #else
    blink_pin_forever(pio, 0, offset, 6, 3);
    #endif
    // For more pio examples see https://github.com/raspberrypi/pico-examples/tree/master/pio

    // set up Timer fires off the callback after 2,000,000us
    add_alarm_in_us((39), alarm_callback, NULL, false);
    // For more examples of timer use see https://github.com/raspberrypi/pico-examples/tree/master/timer

    // set up gpio IRQ
    gpio_init(IRQ_PIN);
    gpio_set_dir(IRQ_PIN, GPIO_IN);
    gpio_pull_down(IRQ_PIN);
    gpio_set_irq_enabled_with_callback(IRQ_PIN, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);

    // for now, run for 20 seconds
//    while (true) {
    uint count = 0;
    while ( count++ < 20) {
        printf("Hello, world!\n");
        while ( ! hsync_flag ) {
            if ( time_us_64() % 1000000 == 0 ) {   // once a second, do this
                printf("Time difference between Hsync and Timer interrupts: %llu microseconds\n", delta);
            }  // end if time
        }  // end while not hsync

        if ( hsync_flag ) {
            printf("Time between Hsync interrupts: %llu microseconds\n", timeBetweenHsyncInterrupts);
            // figure out if clock is leading or lagging, and adjust in real time
            delta = HsyncTime - TimerTime;
            printf("HsyncrTime: %llu\n", HsyncTime);
            // also need to reset alarm here, so timer can sync to it
            hsync_flag = false;
        }
        if ( timer_flag ) {
            printf("Time between Timer interrupts: %llu microseconds\n", timeBetweenTimerInterrupts);
            printf("TimerTime: %llu\n", TimerTime);
            timer_flag = false;
        }

        sleep_ms(1000);
    }
}
