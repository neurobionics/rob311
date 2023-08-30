#include <mbot/utils/utils.h>
#include <pico/stdlib.h>
#include <hardware/i2c.h>

//Always operates on the I2C0 line.

//Checks I2C enable status and initialize iff I2C hasn't been enabled already
//Returns 1 if i2c was initialized as a result, 0 if I2C was already enabled
int mbot_init_i2c(){
    return _mbot_init_i2c(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN);
}

int _mbot_init_i2c(unsigned int pico_sda_pin, unsigned int pico_scl_pin)
{
    if(!_check_i2c0_enabled()){
        i2c_init(i2c0, 400 * 1000);
        gpio_set_function(pico_sda_pin, GPIO_FUNC_I2C);
        gpio_set_function(pico_scl_pin, GPIO_FUNC_I2C);
        gpio_pull_up(pico_sda_pin);
        gpio_pull_up(pico_scl_pin);
        return 1;
    }
    return 0;
}

int _check_i2c0_enabled(){
    // 0x9C: I2C_ENABLE_STATUS has bit 0 = 1 when initialized and 0 when not (default 0)
    return *(volatile uint32_t*)(I2C0_BASE + 0x9C) & 1;
}