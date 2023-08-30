#ifndef MBOT_UTILS_H
#define MBOT_UTILS_H

int mbot_init_i2c();
int _mbot_init_i2c(unsigned int pico_sda_pin, unsigned int pico_scl_pin);
int _check_i2c0_enabled();

#endif