/**
 * Copyright (c) 2021 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/spi.h"

#include "unclocked_input.pio.h"

// Set up a PIO state machine to shift in serial data, sampling with
// system clock (to-do: sync clk_sys to input Hsync)
// and push the data to the RX FIFO, 8 bits at a time.
//

#define SPI_SCK_PIN 2
#define SPI_TX_PIN 3

// GPIO 5 for PIO data input, todo: GPIO 15 for Hsynd input:
#define PIO_INPUT_PIN_BASE 5
#define IRQ_PIN 15

#define BUF_SIZE 8

volatile uint64_t lastHsyncInterruptTime = 0;
volatile uint64_t currentTime = 0;
volatile uint64_t timeBetweenHsyncInterrupts = 0;
volatile bool hsync_flag = false;

void gpio_callback(uint gpio, uint32_t events) {
    currentTime = time_us_64();
    timeBetweenHsyncInterrupts = currentTime - lastHsyncInterruptTime;
    lastHsyncInterruptTime = currentTime;
    hsync_flag=true;
}

int main() {
    stdio_init_all();

    // configure interrupt for Hsync pulse
    gpio_init(IRQ_PIN);
    gpio_set_dir(IRQ_PIN, GPIO_IN);
    gpio_pull_down(IRQ_PIN);
    gpio_set_irq_enabled_with_callback(IRQ_PIN, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);
    printf("IRQ set up on pin %d\n", IRQ_PIN);

    // Configure the SPI before PIO to avoid driving any glitches into the
    // state machine.
    spi_init(spi0, 1000 * 1000);
    uint actual_freq_hz = spi_set_baudrate(spi0, clock_get_hz(clk_sys) / 1);
    printf("SPI running at %u Hz\n", actual_freq_hz);
    gpio_set_function(SPI_TX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI_SCK_PIN, GPIO_FUNC_SPI);

    // Load the clocked_input program, and configure a free state machine
    // to run the program.
    PIO pio = pio0;
    uint offset = pio_add_program(pio, &unclocked_input_program);
    uint sm = pio_claim_unused_sm(pio, true);
    unclocked_input_program_init(pio, sm, offset, PIO_INPUT_PIN_BASE);

    // Make up some random data to send.
    static uint8_t txbuf[BUF_SIZE];
    puts("Data to transmit/store:");
    for (int i = 0; i < BUF_SIZE; ++i) {
        txbuf[i] = rand() >> 16;
        printf("%02x\n", txbuf[i]);
    }

    // The "blocking" write function will send all the data in one go, and not
    // return until the full transmission is finished.
    spi_write_blocking(spi0, (const uint8_t*)txbuf, BUF_SIZE);

    // The data we just sent should now be present in the state machine's
    // FIFO. We only sent 8 bytes, so all the data received by the state
    // machine will fit into the FIFO. Generally you want to be continuously
    // reading data out as it appears in the FIFO -- either with polling, FIFO
    // interrupts, or DMA.
    while (1) {
        if (hsync_flag) {
            hsync_flag = false;
            // Hsync interrupt fires on rising edge, so wait out sync pulse and front porch time
            // then read 800 pixels (oversampling) and wait for next Hsync
            sleep_us(7);
            for (int pixel=0; pixel<(800*4); pixel++) {
                puts("Reading back from RX FIFO:");
                for (int i = 0; i < BUF_SIZE; ++i) {
                    uint8_t rxdata = pio_sm_get_blocking(pio, sm);
                    printf("%02x %s\n", rxdata, rxdata == txbuf[i] ? "OK" : "FAIL");
                }  // end for buffered data
            }  // end for pixel scan line
            while ( ! hsync_flag ) {
                if ( time_us_64() % 1000000 == 0 ) {   // once a second, do this
                    printf("Time between Hsync interrupts: %llu microseconds\n", timeBetweenHsyncInterrupts);
                }  // end if second
            }  // end if not hsync_flag (waiting for one)
        }  // end if hsync_flag
    } // end while (1)
}  // end main
