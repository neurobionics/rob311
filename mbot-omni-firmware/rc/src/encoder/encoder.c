#include <stdio.h>
#include <pico/stdlib.h>
#include <hardware/pwm.h>

#include <rc/encoder/encoder.h>
#include "quadrature_encoder.pio.h"

PIO pio;
uint sm0, sm1, sm2;

//  Initializes the encoder counters for channels 1-3.
//  This also resets the encoder position to 0 so the first position read is consistent.
//  Returns 0 on success or -1 on failure
int rc_encoder_init() {
    stdio_init_all();
    pio = pio0;
    sm0 = pio_claim_unused_sm(pio, true);
    sm1 = pio_claim_unused_sm(pio, true);
    sm2 = pio_claim_unused_sm(pio, true);

    uint offset = pio_add_program(pio, &quadrature_encoder_program);

    quadrature_encoder_program_init(pio, sm0, offset, PIN0_AB, 0);
    quadrature_encoder_program_init(pio, sm1, offset, PIN1_AB, 0);
    quadrature_encoder_program_init(pio, sm2, offset, PIN2_AB, 0);

    quadrature_encoder_set_count(sm0, 0);
    quadrature_encoder_set_count(sm1, 0);
    quadrature_encoder_set_count(sm2, 0);
    
    for(uint i = 0; i<3; i++){
        quadrature_encoder_request_delta(pio, sm0);
        quadrature_encoder_request_delta(pio, sm1);
        quadrature_encoder_request_delta(pio, sm2);
        quadrature_encoder_fetch_delta(pio, sm0);
        quadrature_encoder_fetch_delta(pio, sm1);
        quadrature_encoder_fetch_delta(pio, sm2);
    }
    return (pio_sm_is_claimed(pio, sm0) & pio_sm_is_claimed(pio, sm1) & pio_sm_is_claimed(pio, sm2)) ? 0 : -1;
}

// Stops the encoder counters.
// This is not strictly necessary but is recommended that the user calls this function at the end of their program.
// Returns 0 on success or -1 on failure.
int rc_encoder_cleanup() {
    pio_sm_set_enabled(pio, sm0, false);
    pio_sm_set_enabled(pio, sm1, false);
    pio_sm_set_enabled(pio, sm2, false);
    pio_sm_unclaim(pio, sm0);
    pio_sm_unclaim(pio, sm1);
    pio_sm_unclaim(pio, sm2);
    return (!pio_sm_is_claimed(pio, sm0) & !pio_sm_is_claimed(pio, sm1) & !pio_sm_is_claimed(pio, sm2)) ? 0 : -1;
}

// Reads the current delta of an encoder channel.
// Returns the current delta (signed 32-bit integer) or -1 and prints an error message is there is a problem.
int rc_encoder_read_delta(uint ch) {
    switch (ch) {
        case 1:
            quadrature_encoder_request_delta(pio, sm0);
            return quadrature_encoder_fetch_delta(pio, sm0);
        case 2:
            quadrature_encoder_request_delta(pio, sm1);
            return quadrature_encoder_fetch_delta(pio, sm1);
        case 3:
            quadrature_encoder_request_delta(pio, sm2);
            return quadrature_encoder_fetch_delta(pio, sm2);
        default:
            fprintf(stderr, "Invalid channel!\n");
            return -1;
    }
}

// Reads the current count of an encoder channel.
// This is a signed 32-bit integer that wraps around if the count is allowed to read +- 2^31
// Returns the current count (signed 32-bit integer) or -1 and prints an error message is there is a problem.
int rc_encoder_read_count(uint ch) {
    switch (ch) {
        case 1:
            return quadrature_encoder_get_count(sm0);
        case 2:
            return quadrature_encoder_get_count(sm1);
        case 3:
            return quadrature_encoder_get_count(sm2);
        default:
            fprintf(stderr, "Invalid channel!\n");
            return -1;
    }
    return 0;
}

// Sets the current position of an encoder channel.
// Usually for resetting a counter to 0 but can set an arbitrary position if desired.
// Returns 0 on success, -1 on failure
int rc_encoder_write(uint ch, int pos) {
    switch (ch) {
        case 1:
            quadrature_encoder_set_count(sm0, pos);
            break;
        case 2:
            quadrature_encoder_set_count(sm1, pos);
            break;
        case 3:
            quadrature_encoder_set_count(sm2, pos);
            break;
        default:
            fprintf(stderr, "Invalid channel!\n");
            return -1;
    }
    return 0;
}
