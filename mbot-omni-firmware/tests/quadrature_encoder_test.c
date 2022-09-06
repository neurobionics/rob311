/**
 * Copyright (c) 2021 pmarques-dev @ github
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/timer.h"

#include "quadrature_encoder.pio.h"

//
// ---- quadrature encoder interface example
//
// the PIO program reads phase A/B of a quadrature encoder and increments or
// decrements an internal counter to keep the current absolute step count
// updated. At any point, the main code can query the current count by using
// the quadrature_encoder_*_count functions. The counter is kept in a full
// 32 bit register that just wraps around. Two's complement arithmetic means
// that it can be interpreted as a 32-bit signed or unsigned value, and it will
// work anyway.
//
// As an example, a two wheel robot being controlled at 100Hz, can use two
// state machines to read the two encoders and in the main control loop it can
// simply ask for the current encoder counts to get the absolute step count. It
// can also subtract the values from the last sample to check how many steps
// each wheel as done since the last sample period.
//
// One advantage of this approach is that it requires zero CPU time to keep the
// encoder count updated and because of that it supports very high step rates.
//

int main() {
    int new_value0, total0 = 0;
    int new_value1, total1 = 0;
    int new_value2, total2 = 0;

    // Base pin to connect the A phase of the encoder.
    // The B phase must be connected to the next pin
    // These pins are per the schematic for the pico RCBoard
    const uint PIN0_AB = 6;
    const uint PIN1_AB = 8;
    const uint PIN2_AB = 10;
    
    stdio_init_all();
    PIO pio = pio0;
    const uint sm0 = pio_claim_unused_sm(pio, true);
    const uint sm1 = pio_claim_unused_sm(pio, true);
    const uint sm2 = pio_claim_unused_sm(pio, true);

    uint offset = pio_add_program(pio, &quadrature_encoder_program);

    quadrature_encoder_program_init(pio, sm0, offset, PIN0_AB, 0);
    quadrature_encoder_program_init(pio, sm1, offset, PIN1_AB, 0);
    quadrature_encoder_program_init(pio, sm2, offset, PIN2_AB, 0);

    quadrature_encoder_set_count(sm0, 0);
    quadrature_encoder_set_count(sm1, 0);
    quadrature_encoder_set_count(sm2, 0);

    //in the beginning the deltas are trash, try reading once to clear them
    for(uint i = 0; i<3; i++){
        quadrature_encoder_request_delta(pio, sm0);
        quadrature_encoder_request_delta(pio, sm1);
        quadrature_encoder_request_delta(pio, sm2);
        new_value0 = quadrature_encoder_fetch_delta(pio, sm0);
        new_value1 = quadrature_encoder_fetch_delta(pio, sm1);
        new_value2 = quadrature_encoder_fetch_delta(pio, sm2);
    }

    sleep_ms(2000);
    printf("delta0 | delta1 | delta2 | total0 | total1 | total2 | count0 | count1 | count2                                      \n");
    while (1) {
        // note: thanks to two's complement arithmetic delta will always
        // be correct even when new_value wraps around MAXINT / MININT

        quadrature_encoder_request_delta(pio, sm0);
        quadrature_encoder_request_delta(pio, sm1);
        quadrature_encoder_request_delta(pio, sm2);
        new_value0 = quadrature_encoder_fetch_delta(pio, sm0);
        new_value1 = quadrature_encoder_fetch_delta(pio, sm1);
        new_value2 = quadrature_encoder_fetch_delta(pio, sm2);
        // new_value0 = quadrature_encoder_get_delta(pio, sm0);
        // new_value1 = quadrature_encoder_get_delta(pio, sm1);
        // new_value2 = quadrature_encoder_get_delta(pio, sm2);
        printf("%7d| %7d| %7d| ", new_value0, new_value1, new_value2);
        

        total0 += new_value0;
        total1 += new_value1;
        total2 += new_value2;

        printf("%7d| %7d| %7d| ", total0, total1, total2);

        new_value0 = quadrature_encoder_get_count(sm0);
        new_value1 = quadrature_encoder_get_count(sm1);
        new_value2 = quadrature_encoder_get_count(sm2);

        printf("%7d| %7d| %7d\r", new_value0, new_value1, new_value2);
        sleep_ms(20);
    }
}

