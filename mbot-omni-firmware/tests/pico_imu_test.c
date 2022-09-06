#include <stdio.h>
#include <stdint.h>
#include <pico/stdlib.h>
#include <pico/binary_info.h>
#include <rc/mpu/mpu.h>

#define LED_PIN 25

int main() {
    bi_decl(bi_program_description("This is a test binary."));
    bi_decl(bi_1pin_with_name(LED_PIN, "On-board LED"));
    // Pins
    const uint sda_pin = 4;
    const uint scl_pin = 5;
    // Ports
    printf("starting...\n");
    printf("stdio init...\n");
    stdio_init_all();


    i2c_inst_t *i2c = i2c0;
    //Initialize I2C port at 400 kHz
    i2c_init(i2c, 400 * 1000);
    // Initialize I2C pins
    printf("setting functions...\n");
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    printf("setting pullups...\n");
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(sda_pin, scl_pin, GPIO_FUNC_I2C));

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    printf("resetting mpu...\n");
    mpu9250_reset(i2c);
    printf("initing dmp...\n");
    static rc_mpu_data_t mpu_data;
    mpu9250_init_dmp(i2c, &mpu_data);  

    while (1) {
        //spin...
        gpio_put(LED_PIN, 0);
        sleep_ms(250);
        //mpu9250_read_raw(i2c, acceleration, gyro, mag, &temp);
        gpio_put(LED_PIN, 1);
        //printf("Acc. X = %d, Y = %d, Z = %d\n", acceleration[0], acceleration[1], acceleration[2]);
        //printf("Gyro. X = %d, Y = %d, Z = %d\n", gyro[0], gyro[1], gyro[2]);
        //printf("Mag. X = %d, Y = %d, Z = %d\n", mag[0], mag[1], mag[2]);
        //printf("Temp. = %f\n", (temp / 340.0) + 36.53);
        sleep_ms(250);
    }
}

void blink() {
    gpio_put(PICO_DEFAULT_LED_PIN, true);
    sleep_ms(500);
    gpio_put(PICO_DEFAULT_LED_PIN, false);
}
