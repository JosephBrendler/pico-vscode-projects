// -------------------------------------------------- //
// This file is autogenerated by pioasm; do not edit! //
// -------------------------------------------------- //

#pragma once

#if !PICO_NO_HARDWARE
#include "hardware/pio.h"
#endif

// ------ //
// pioIRQ //
// ------ //

#define pioIRQ_wrap_target 0
#define pioIRQ_wrap 10
#define pioIRQ_pio_version 0

static const uint16_t pioIRQ_program_instructions[] = {
            //     .wrap_target
    0xe001, //  0: set    pins, 1
    0xe63f, //  1: set    x, 31                  [6]
    0xbd42, //  2: nop                           [29]
    0x0042, //  3: jmp    x--, 2
    0xe000, //  4: set    pins, 0
    0xe63f, //  5: set    x, 31                  [6]
    0xbd42, //  6: nop                           [29]
    0x0046, //  7: jmp    x--, 6
    0xbd42, //  8: nop                           [29]
    0xc023, //  9: irq    wait 3
    0x0000, // 10: jmp    0
            //     .wrap
};

#if !PICO_NO_HARDWARE
static const struct pio_program pioIRQ_program = {
    .instructions = pioIRQ_program_instructions,
    .length = 11,
    .origin = -1,
    .pio_version = pioIRQ_pio_version,
#if PICO_PIO_VERSION > 0
    .used_gpio_ranges = 0x0
#endif
};

static inline pio_sm_config pioIRQ_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + pioIRQ_wrap_target, offset + pioIRQ_wrap);
    return c;
}

#include "hardware/clocks.h"
static inline void pioIRQ_program_init(PIO pio, uint sm, uint offset, uint pin, float freq) {
    pio_sm_config c = pioIRQ_program_get_default_config(offset);
    // Map the state machine's SET pin group to one pin, namely the `pin`
    // parameter to this function.
    sm_config_set_set_pins(&c, pin, 1);
    // Set this pin's GPIO function (connect PIO to the pad)
    pio_gpio_init(pio, pin);
    // Set the pin direction to output at the PIO
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);
	// set the state machine clock rate
	float div = clock_get_hz(clk_sys) / freq ;  // calculates the clock divider
	sm_config_set_clkdiv(&c, div);
    // Load our configuration, and jump to the start of the program
    pio_sm_init(pio, sm, offset, &c);
    // Set the state machine running
    pio_sm_set_enabled(pio, sm, true);
}

#endif

