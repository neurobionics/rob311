#include <stdio.h>
#include <pico/stdlib.h>
#include <hardware/adc.h>

#include <rc/current/current.h>

void rc_current_sense_init(void)
{
    // init the adc controller
    adc_init();

    // make sure the GPIO pins are in high-impedance mode
    adc_gpio_init(ADC0_GPIO);
    adc_gpio_init(ADC1_GPIO);
    adc_gpio_init(ADC2_GPIO);
}

int rc_current_sense_get_raw(int motor)
{
    adc_select_input(motor - 1);
    return adc_read();
}