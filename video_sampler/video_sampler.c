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

#include "./sample.pio.h"
#include "./sync.pio.h"

//-----[] define and declare constants and static variables ]--------------------------
#define PIO_IRQ0 PIO0_IRQ_0
#define PIO_IRQ1 PIO0_IRQ_1

#define START_DELAY 5 // seconds (give time to connect monitor)

// set up system clock
static uint32_t MY_SYS_CLK_HZ = 250 * MHZ;
// this system freq (overclocked) is set by assignment at the top of
//   main(){ //prior to stdio_init_all() } [botom of this file]

// set pixel clock freq for 800 x Hsync freq x 3 for over-sampling
// Hsync freq (1/38us = 21,315.789 Hz) x 3 x 800 = 63,157,893.6  Hz (63.1578936 MHZ)
const static uint OVERSAMPLE = 3;            // oversampling multiplier
const static uint PIXELS_PER_SCANLINE = 800; // includes front and back porch
static uint32_t PIXEL_FREQ = (uint32_t)(21.315789 * MHZ * PIXELS_PER_SCANLINE * OVERSAMPLE);

const static uint activezone_scan_line_pixel_count = 640;
const static uint activezone_scan_line_count = 400;

const static uint HSYNC_PIN = 15;    // GPIO 15; pin 35 (from inverter pin 2)
const static uint VSYNC_PIN = 5;     // GPIO 5;  pin 11 (from inverter pin 4)
const static uint VIDEO_PIN = 2;     // GPIO 2;  pin 38 (from trimpot1 wiper)
const static uint INTENSITY_PIN = 3; // GPIO 3;  pin 15 (from trimpot2 wiper)
// GND(black) on pin 20; 3.3v (red) on pin 1 (for trim-pot v-divs and inverters)
// for input gpio; set true for pullup | false for pulldown (one true if needed; else set both false)
static bool PULL_UP = true;
static bool PULL_DOWN = false;
static char updown[] = "not set";

static uint hsync_sm = 0;
static uint vsync_sm = 1;
static uint video_sm = 2;
static uint intensity_sm = 3;
static uint pio_irq_num = 0;

volatile uint64_t currentTime = 0;

volatile bool hsync_irq_flag = false;
volatile uint64_t hsync_interrupt_count = 0;
volatile uint64_t timeBetweenHsyncInterrupts = 0;
volatile uint64_t lastHsyncInterruptTime = 0;

volatile bool vsync_irq_flag = false;
volatile uint64_t vsync_interrupt_count = 0;
volatile uint64_t timeBetweenVsyncInterrupts = 0;
volatile uint64_t lastVsyncInterruptTime = 0;

// run 4 state machines - 4 on on PIO0 (hsync, vsync, rx_video_data, rx_intensity_data)
static PIO pio_0 = pio0;

volatile bool dma_irq_flag = false;
// set BUFFER_SIZE as nneded to sample ( 640 pixels / 32 px-per-fifo-sample ) * OVERSAMPLE
// const uint BUFFER_SIZE = (uint)( activezone_scan_line_pixel_count / 32) * OVERSAMPLE;
#define BUFFER_SIZE 60
static int32_t sample_buffer[BUFFER_SIZE] = {0};
static int dma_chan; // dma channel number (TBA)

//-----[ functions ]----------------------------------------------------------------
void pio_irq0_handler()
{
    // Clear the interrupt flag
    if (pio_interrupt_get(pio_0, 0)) // returns TRUE if IRQ 0 is set (bit 8 in IRQ register)
    {
        // printf("Before Clear IRQ0 [INTR]: %x [IRQ]: %x || ", pio_0->intr, pio_0->irq);
        pio_interrupt_clear(pio_0, 0);
        // printf("After Clear IRQ0 [INTR]: %x [IRQ]: %x \n", pio_0->intr, pio_0->irq);
        hsync_irq_flag = true;
    }
    else
    {
        printf("Error: got wrong interrupt signal [INTR]: %x [IRQ]: %x \n", pio_0->intr, pio_0->irq);
    }
}

void pio_irq1_handler()
{
    // Clear the interrupt flag
    if (pio_interrupt_get(pio_0, 1)) // returns TRUE if IRQ 0 is set (bit 8 in IRQ register)
    {
        // printf("Before Clear IRQ0 [INTR]: %x [IRQ]: %x || ", pio_0->intr, pio_0->irq);
        pio_interrupt_clear(pio_0, 1);
        // printf("After Clear IRQ0 [INTR]: %x [IRQ]: %x \n", pio_0->intr, pio_0->irq);
        vsync_irq_flag = true;
    }
    else
    {
        printf("Error: got wrong interrupt signal [INTR]: %x [IRQ]: %x \n", pio_0->intr, pio_0->irq);
    }
}

void dma_irq_handler()
{
    // clear the hw irq flag and set the boolean flag
    dma_hw->ints0 = 1u << dma_chan;
    dma_irq_flag = true;
}

void print_binary(uint32_t num)
{
    for (int i = 31; i >= 0; i--)
    {
        printf("%d", (num >> i) & 1);
    }
    printf("\n");
}

void setup_my_gpio_pin(uint myPIN)
{
    // set myPIN (assigned in the top block) as gpio input
    // configure for pull up/down as applicable
    gpio_set_dir(myPIN, false);
    sprintf(updown, "not set");
    if (PULL_DOWN)
    {
        gpio_pull_down(myPIN);
        sprintf(updown, "down");
    }
    if (PULL_UP)
    {
        gpio_pull_up(myPIN);
        sprintf(updown, "up");
    }
    printf("  gpio%d: configured as input, with pullup/down: %s\n", myPIN, updown);
}

void setup()
{
    // disable irqs while dpomg setup
    irq_set_enabled(PIO_IRQ0, false); // hsync
    irq_set_enabled(PIO_IRQ1, false); // vsync

    printf("Starting program setup()\n");

    // load program for both hsync_sm and vsync_sm (H/V_sync interrupts)
    uint offset0 = pio_add_program(pio_0, &sync_program);
    printf("  pio_0: loaded &sync_program for hsync_sm and vsync_sm, and got offset0: %d\n", offset0);

    // load program for both video_sm and intensity_sm (video/intensity bit samplers)
    uint offset1 = pio_add_program(pio_0, &sample_program);
    printf("  pio_0: loaded &sample_program for video_sm and intensity_sm, and got offset1: %d\n", offset1);

    //-----[ set up 4 x state machines on pio_0 ]--------------------------------------------
    // hsync_sm - generate Hsync interrupt IRQ0
    // vsync_sm - generate Vsync interrupt IRQ1
    // video_sm - video bit sampler (bit 1 of 2-bit color0)
    // intensity_sm - intensity bit sampler (bit 2 of 2-bit color0)

    // examine existing SMs on this PIO; nothing else should be useing them, but unclaim them if needed
    for (uint i = 0; i < 4; i++)
    {
        if (pio_sm_is_claimed(pio_0, i))
        {
            // don't know why it's busy, but set it free
            printf("  sm%d: (already claimed), fixing...", i);
            pio_sm_unclaim(pio_0, i);
            printf(" (unclaimed)");
            pio_sm_claim(pio_0, i);
            printf(" (properly claimed)\n");
        }
        else
        {
            printf("  sm%d: (unclaimed)", i);
            pio_sm_claim(pio_0, i);
            printf(" (properly claimed)\n");
        }
    }

    // initailize hsync_sm and vsync_sm sync program with offset, pin, and div
    setup_my_gpio_pin(HSYNC_PIN);
    setup_my_gpio_pin(VSYNC_PIN);
    sync_program_init(pio_0, hsync_sm, offset0, HSYNC_PIN, div);
    printf("  pio_0: initialized sync_program for hsync_sm, offset0: %x hex, HSYNC_PIN: %d, and divider: %.8f\n", offset0, HSYNC_PIN, div);
    sync_program_init(pio_0, vsync_sm, offset0, VSYNC_PIN, div);
    printf("  pio_0: initialized sync_program for vsync_sm, offset0: %x hex, VSYNC_PIN: %d, and divider: %.8f\n", offset0, VSYNC_PIN, div);

    // initailize video_sm and intensity_sm sample program with offset, pin, and div
    setup_my_gpio_pin(VIDEO_PIN);
    setup_my_gpio_pin(INTENSITY_PIN);
    sample_program_init(pio_0, video_sm, offset1, VIDEO_PIN, div);
    printf("  pio_0: initialized sample_program for video_sm, offset1: %x hex, VIDEO_PIN: %d, and divider: %.8f\n", offset1, VIDEO_PIN, div);
    sample_program_init(pio_0, intensity_sm, offset1, INTENSITY_PIN, div);
    printf("  pio_0: initialized sample_program for intensity_sm, offset1: %x hex, INTENSITY_PIN: %d, and divider: %.8f\n", offset1, INTENSITY_PIN, div);

    // Write to the TX FIFO the frame sizes for both video_sm & intensity_sm
    // (this is only done once; sm's read once prior to wrap_target)
    pio_sm_put_blocking(pio_0, video_sm, activezone_scan_line_pixel_count);
    pio_sm_put_blocking(pio_0, video_sm, activezone_scan_line_count);
    printf("  video_sm: filled tx_fifo with frame sizes (%d x %d)\n",
           activezone_scan_line_pixel_count, activezone_scan_line_count);
    pio_sm_put_blocking(pio_0, intensity_sm, activezone_scan_line_pixel_count);
    pio_sm_put_blocking(pio_0, intensity_sm, activezone_scan_line_count);
    printf("  intensity_sm: filled tx_fifo with frame sizes (%d x %d)\n",
           activezone_scan_line_pixel_count, activezone_scan_line_count);

    // set up and display system frequencies
    printf("  System Clock Frequency is %d Hz\n", clock_get_hz(clk_sys));
    printf("  USB Clock Frequency is %d Hz\n", clock_get_hz(clk_usb));
    printf("  Pixel Clock Frequency is %d Hz\n", PIXEL_FREQ);

    // calculate divider needed to get PIXEL_FREQ
    uint32_t sys_clock = clock_get_hz(clk_sys);
    float div = {sys_clock / (float)PIXEL_FREQ};
    //float div = sys_clock / PIXEL_FREQ;
    printf("  calculated divider to generate PIXEL_FREQ: %.8f\n", div);

    //-----[ set up DMA ]----------------------------------
    // 32 bit transfers. Both read and write address increment after each
    // transfer (each pointing to a location in src or dst respectively).
    // No DREQ is selected, so the DMA transfers as fast as it can.

    // Get a free channel, panic() if there are none
    dma_chan = dma_claim_unused_channel(true);
    printf("  dma channel %d:claimed\n", dma_chan);

    dma_channel_config dma_config = dma_channel_get_default_config(dma_chan);
    channel_config_set_transfer_data_size(&dma_config, DMA_SIZE_32);
    channel_config_set_read_increment(&dma_config, true);
    channel_config_set_write_increment(&dma_config, true);
    uint dma_dreq = pio_get_dreq(pio_0, video_sm, false);
    channel_config_set_dreq(&dma_config, dma_dreq);
    printf("  dma channel %d: configured with DMA_SIZE_32: %d, and r, w incr: true, and dreq: %d\n",
           dma_chan, DMA_SIZE_32, dma_dreq);

    // set up dma to trigger upon completion
    dma_channel_configure(
        dma_chan,              // Channel to be configured
        &dma_config,           // The configuration we just created
        sample_buffer,         // The initial write address
        &pio_0->rxf[video_sm], // The initial read address
        BUFFER_SIZE,           // Number of transfers; in this case each is 1 word
        true                   // Start immediately.
    );
    printf("  dma channel %d: configured initial addresses r: %x, w: %x, and set to start\n",
           dma_chan, dst, src);

    // set up dma interrupt
    irq_set_exclusive_handler(DMA_IRQ_0, &dma_irq_handler);
    printf("  ran (dma) irq_set_exclusive_handler: &dma_irq_handler\n");
    irq_set_enabled(DMA_IRQ_0, true);
    printf("  ran (dma) irq_set_enabled()\n");

    // Configure HSYNC interrupt (source, handler, then enable)
    pio_set_irq0_source_enabled(pio_0, pis_interrupt0, true);
    printf("  pio_set_irq0_source_enabled (IRQ flag) done for pio pis_interrupt0: %d\n", pis_interrupt0);
    irq_set_exclusive_handler(PIO_IRQ0, pio_irq0_handler);
    printf("  irq_set_exclusive_handler done for PIO-NVIC IRQ %d\n", PIO_IRQ0);
    irq_set_enabled(PIO_IRQ0, true);
    printf("  irq_set_enabled true for PIO-NVIC IRQ %d\n", PIO_IRQ0);

    // Configure VSYNC interrupt (source, handler, then enable)
    pio_set_irq1_source_enabled(pio_0, pis_interrupt1, true);
    printf("  pio_set_irq1_source_enabled (IRQ flag) done for pio pis_interrupt1: %d\n", pis_interrupt1);
    irq_set_exclusive_handler(PIO_IRQ1, pio_irq1_handler);
    printf("  irq_set_exclusive_handler done for PIO-NVIC IRQ %d\n", PIO_IRQ1);
    irq_set_enabled(PIO_IRQ1, true);
    printf("  irq_set_enabled true for PIO-NVIC IRQ %d\n", PIO_IRQ1);
}

void loop()
{
    printf("Running microcontroller loop() function\n");
    uint count = 0;
    while (count < 10)
    {
        if (hsync_irq_flag)
        {
            // disable irqs while handling one
            //            irq_set_enabled(PIO_IRQ0, false);      // don't use NVIC, just poll INTR in loop
            currentTime = time_us_64();
            timeBetweenHsyncInterrupts = currentTime - lastHsyncInterruptTime;
            lastHsyncInterruptTime = currentTime;
            hsync_interrupt_count++;
            hsync_irq_flag = false; // don't need this if not using NVIC IRQ
                                    // re-enable irqs after handling one
            //            irq_set_enabled(PIO_IRQ0, true);           // donnt use NVIC - just poll INTR in loop
        }
        if (vsync_irq_flag)
        {
            // disable irqs while handling one
            //            irq_set_enabled(PIO_IRQ1, false);      // don't use NVIC, just poll INTR in loop
            currentTime = time_us_64();
            timeBetweenVsyncInterrupts = currentTime - lastVsyncInterruptTime;
            lastVsyncInterruptTime = currentTime;
            vsync_interrupt_count++;
            vsync_irq_flag = false; // don't need this if not using NVIC IRQ
            // re-enable irqs after handling one
            //            irq_set_enabled(PIO_IRQ1, true);           // donnt use NVIC - just poll INTR in loop
        }
        if (dma_irq_flag)
        {
            // display data
            printf("sampled data:\n");
            for (int i = 0; i < BUFFER_SIZE; i++)
            {
                printf("%x ", sample_buffer[i]);
                if ((i + 1) % 16 == 0)
                {
                    printf("\n");
                }
            }
            dma_irq_flag = false;
        }
        if (time_us_64() % 1000000 == 0)
        {
            uint seconds = (uint)(time_us_64() / MHZ);
            printf("Runtime: (%ds)\n", seconds - START_DELAY);
            printf("    H_IRQs: (%llu), Delta_t btw H_IRQs: (%llu us)\n",
                   hsync_interrupt_count, timeBetweenHsyncInterrupts);
            printf("    V_IRQs: (%llu), Delta_t btw V_IRQs: (%llu us)\n",
                   vsync_interrupt_count, timeBetweenVsyncInterrupts);
            count++;
        }

        //        sleep_ms(1000);
        /*-------------------------------------
            uint32_t rx_data = 0;
            while ( ! pio_sm_is_rx_fifo_empty(pio_0, video_sm))
            {
                rx_data = pio_sm_get_blocking(pio_0, video_sm);
                printf(" In hex: %x\n", rx_data);
                print_binary(rx_data);
            }
        //--------------------------------------*/
    }
}

int main()
{

    // setup overclocking
    set_sys_clock_hz(MY_SYS_CLK_HZ, true);

    stdio_init_all();
    sleep_ms(START_DELAY * 1000); // give me time to turn on minicom monitor

    printf("Main() about to run setup()\n");
    setup();
    printf("Back in main(), about to run loop()\n");
    loop();
    printf("Returned from loop() to main(); program complete\n");
    return 0;
}
