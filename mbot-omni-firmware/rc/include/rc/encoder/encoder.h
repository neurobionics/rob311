#include <stdio.h>
#include <pico/stdlib.h>

#ifndef RC_ENCODER_H
#define RC_ENCODER_H

#define PIN0_AB     6
#define PIN1_AB     8
#define PIN2_AB     10

int rc_encoder_init();
int rc_encoder_cleanup();
int rc_encoder_read_delta(uint ch);
int rc_encoder_read_count(uint ch);
int rc_encoder_write(uint ch, int pos);

#endif /* RC_ENCODER_H */
