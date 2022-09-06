#ifndef CURRENT_H
#define CURRENT_H

#define ADC0_GPIO 26
#define ADC1_GPIO 27
#define ADC2_GPIO 28

void rc_current_sense_init(void);
int rc_current_sense_get_raw(int motor);

#endif