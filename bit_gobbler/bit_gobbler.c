#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"

// Data will be copied from src to dst
const char src[] = "Hello, world! (from DMA)";
char dst[count_of(src)];

#include "./bit_gobbler.pio.h"
#include "./hsync.pio.h"

#define PIO_IRQ PIO0_IRQ_0

static uint HSYNC_PIN = 15;        // GPIO 15; pin 35 (from inverter pin 2)
static uint VSYNC_PIN = 5;         // GPIO 5;  pin 11 (from inverter pin 4)
static uint VIDEO_PIN = 2;         // GPIO 2;  pin 38 (from trimpot1 wiper)
static uint INTENSITY_PIN = 3;     // GPIO 3;  pin 15 (from trimpot2 wiper)
// GND(black) on pin 20; 3.3v (red) on pin 1 (for trim-pot v-divs and inverters)
// for input gpio; set true for pullup | false for pulldown (one true if needed; else set both false)
static bool PULL_UP = true;
static bool PULL_DOWN = false;

static uint sm0 = 0;
static uint sm1 = 1;
static uint sm2 = 2;
static uint pio_irq_num = 0;
static float PIXEL_FREQ = (25 * MHZ);

volatile bool hsync_irq_flag = false;
volatile uint64_t interrupt_count = 0;
volatile uint64_t currentTime = 0;
volatile uint64_t timeBetweenInterrupts = 0;
volatile uint64_t lastInterruptTime = 0;


// run five state machines - 4 on on PIO0 (hsync, vsync, rx_video_data, rx_intensity_data)
static PIO pio_0 = pio0;

void bit_gobble_forever(PIO pio, uint sm, uint offset, uint pin, uint freq) {
    bit_gobbler_program_init(pio, sm, offset, pin);
    pio_sm_set_enabled(pio, sm, true);

    printf("Gobbling data from pin %d at %d Hz\n", pin, freq);

    // PIO counter program takes 3 more cycles in total than we pass as
    // input (wait for n + 1; mov; jmp)
//    pio->txf[sm] = (125000000 / (2 * freq)) - 3;
    pio->txf[sm] = ((133 * MHZ) / (2 * freq)) - 3;
}

// currently just getting one (640 pixel) line of video scan, between HSYNC pulses
// need to add 400 scan lines, between VSYNC pulses

void pio_irq_handler() {
    // Clear the interrupt flag
    if (pio_interrupt_get(pio_0, 0)) // returns TRUE if IRQ 0 is set (bit 8 in IRQ register)
    {
        printf("Before Clear IRQ0 [INTR]: %x [IRQ]: %x || ", pio_0->intr, pio_0->irq);
        pio_interrupt_clear(pio_0, 0);
        printf("After Clear IRQ0 [INTR]: %x [IRQ]: %x \n", pio_0->intr, pio_0->irq);
        hsync_irq_flag = true;
    }
    else
    {
        printf("Error: got wrong interrupt signal [INTR]: %x [IRQ]: %x \n", pio_0->intr, pio_0->irq);
    }
}

void print_binary(uint32_t num) {
    for (int i = 31; i >= 0; i--) {
        printf("%d", (num >> i) & 1);
    }
    printf("\n");
}

void setup() {
    // disable irqs while dpomg setup
    irq_set_enabled(PIO_IRQ, false);

    printf("Starting setup()\n");

    // Get a free channel, panic() if there are none
    int chan = dma_claim_unused_channel(true);
    printf("  claimed dma channel: %d\n", chan);

    // 8 bit transfers. Both read and write address increment after each
    // transfer (each pointing to a location in src or dst respectively).
    // No DREQ is selected, so the DMA transfers as fast as it can.

    dma_channel_config c = dma_channel_get_default_config(chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, true);
    printf("  configured dma channel with DMA_SIZE_8: %d, and read and write increment: true\n", DMA_SIZE_8);

    dma_channel_configure(
        chan,          // Channel to be configured
        &c,            // The configuration we just created
        dst,           // The initial write address
        src,           // The initial read address
        count_of(src), // Number of transfers; in this case each is 1 byte.
        true           // Start immediately.
    );
    printf("  configured dma channel: %d initial addresses r: %x, w: %x, and set to start\n", chan, dst, src);

    // We could choose to go and do something else whilst the DMA is doing its
    // thing. In this case the processor has nothing else to do, so we just
    // wait for the DMA to finish.
    dma_channel_wait_for_finish_blocking(chan);
    printf("  dma_channel_wait_for_finish_blocking complete, dma channel finished\n");

    printf("  dma has copied text from transmit buffer (src), to receive buffer (dst). Printing it ...\n");
    // The DMA has now copied our text from the transmit buffer (src) to the
    // receive buffer (dst), so we can print it out from there.
    puts(dst);

    //---- currently planning for 3 sms (load their programs here ----------------------
    // sm0 - generate Hsync interrupt IRQ0
    uint offset0 = pio_add_program(pio_0, &hsync_program);
    printf("  added &hsync_program and got offset0: %d\n", offset0);

    // sm1 - generate Vsync interrupt IRQ1
    // maybe something like hsync above

    // sm2 - read video bits into fifo
 //   uint offset2 = pio_add_program(pio_0, &bit_gobbler_program);
 //   printf("  Loaded gobbler program at %d\n", offset2);
    //----- now claim the sms ----------------
     // examine existing SMs on this PIO; nothing else should be useing them, but unclaim them if needed
     for (uint i = 0; i < 4; i++)
     {
         if (pio_sm_is_claimed(pio_0, i))
         {
             // don't know why it's busy, but set it free
             printf("  SM: %d is claimed, trying to unclaim ...", i);
             pio_sm_unclaim(pio_0, i);
             printf("  now unclaimed.\n");
         }
         else
         {
             printf("  checked SM: %d, (uncliamed)\n", i);
         }
     }
     // now claim state macinges (sm0-2)
     pio_sm_claim (pio_0, 0);
     printf("  claimed sm0\n");
     pio_sm_claim (pio_0, 1);
     printf("  claimed sm1\n");
     pio_sm_claim (pio_0, 2);
     printf("  claimed sm2\n");

//    printf("  Gobbling bits from pin %d at %llu Hz\n", VIDEO_PIN, PIXEL_FREQ);
printf("  System Clock Frequency is %d Hz\n", clock_get_hz(clk_sys));
printf("  USB Clock Frequency is %d Hz\n", clock_get_hz(clk_usb));
printf("  Pixel Clock Frequency is %d Hz\n", PIXEL_FREQ);
// For more examples of clocks use see https://github.com/raspberrypi/pico-examples/tree/master/clocks

// calculate divider needed to get PIXEL_FREQ
    float sys_clock = (float)clock_get_hz(clk_sys);
    float div = sys_clock / PIXEL_FREQ;
    printf("  calculated divider to generate PIXEL_FREQ: %.4f\n", div));

    // set HSYNC_PIN (assigned in the top block) as gpio input; configure for pull up/down as applicable
    gpio_set_dir(HSYNC_PIN, false);
    char updown[] = "not set";
    if (PULL_DOWN) { gpio_pull_down(HSYNC_PIN); sprintf(updown, "down"); }
    if (PULL_UP) { gpio_pull_up(HSYNC_PIN); sprintf(updown, "up"); }
    printf("  configured HSYNC_PIN: %d as input, with pullup/down: %s\n", HSYNC_PIN, updown);

    // initailize pio hsync program with offset, pin, and div
    // void hsync_program_init(PIO pio, uint sm, uint offset, uint pin, float div)
    hsync_program_init(pio_0, sm0, offset0, HSYNC_PIN, div);
    printf("  Initialized (pio) hsync_program for sm0, offset0: %x hex, HSYNC_PIN: %d, and divider: %.4f\n", offset0, HSYNC_PIN, div);

     // Write the number to the TX FIFO (blocking if full)
    uint32_t scan_line_pixel_count = 640;
//    printf("  About to write scan_line_pixel_count: %d to sm tx fifo\n", scan_line_pixel_count);
 //   pio_sm_put_blocking(pio_0, sm2, scan_line_pixel_count);

    // initialize and start the statemachine
//    bit_gobble_forever(pio_0, sm2, offset2, VIDEO_PIN, PIXEL_FREQ);
//    printf("  Gobbler initialized on sm %d, running at %.4f\n", sm2, PIXEL_FREQ);

    // sm3 - read intensity bits into fifo
    // maybe something like bit_gobbler above

    // Configure interrupt (source, handler, then enable)
    pio_set_irq0_source_enabled(pio_0, pis_interrupt0, true);
    printf("  pio_set_irq0_source_enabled (IRQ flag) done for pio pis_interrupt0: %d\n", pis_interrupt0);
    irq_set_exclusive_handler(PIO_IRQ, pio_irq_handler);
    printf("  irq_set_exclusive_handler done for PIO-NVIC IRQ %d\n", PIO_IRQ);
    irq_set_enabled(PIO_IRQ, true);
    printf("  irq_set_enabled true for PIO-NVIC IRQ %d\n", PIO_IRQ);
}

void loop() {
    printf("Running microcontroller loop() function");
    int count = 0;
    while (count++ < 10) {
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
        if (!pio_sm_is_rx_fifo_empty(pio_0, sm2)) {
            uint32_t data = pio_sm_get_blocking(pio_0, sm2);
            printf(" Received hex: %x   binary: ", data);
            print_binary(data);
        }
//        sleep_ms(1000);
    }
}

int main()
{
    stdio_init_all();
    printf("Main() about to run setup()\n");
    setup();
    printf("Back in main(), about to run loop()\n");
    loop();
    printf("Returned from loop() to main(); program complete\n");
    return 0;
}
