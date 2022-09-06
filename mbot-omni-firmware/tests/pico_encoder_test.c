#include <stdio.h>
#include <stdint.h>
#include <pico/stdlib.h>

#include <rc/motor/motor.h>
#include <rc/encoder/encoder.h>

#define INT_16_MAX 32768

void drive_motor_up_down(int);
void reset_encoders();
void blink();

int main() {
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    
    rc_motor_init();
    rc_encoder_init();
    
    while (true) {
        blink();
        drive_motor_up_down(1);
        
        blink();
        drive_motor_up_down(2);

        blink();
        drive_motor_up_down(3);
        
        blink();
        drive_motor_up_down(0);
        
        reset_encoders();
    }
    
    return 0;
}

void drive_motor_up_down(int motor) {
    int32_t d = 0;
    int new_value0, total0;
    int new_value1, total1;
    int new_value2, total2;
    for (; d < INT_16_MAX; d += 64) {
        rc_motor_set(motor, d);
        sleep_ms(4);
        new_value0 = rc_encoder_read_delta(1);
        new_value1 = rc_encoder_read_delta(2);
        new_value2 = rc_encoder_read_delta(3);
        total0 = rc_encoder_read_count(1);
        total1 = rc_encoder_read_count(2);
        total2 = rc_encoder_read_count(3);
        printf("\033[2A\r| Delta A | Delta B | Delta C | total A | total B | total C |\n\r| %7d | %7d | %7d | %7d | %7d | %7d |", new_value0, new_value1, new_value2, total0, total1, total2);
    }
    for (; d > 0; d -= 64) {
        rc_motor_set(motor, d);
        sleep_ms(4);
        new_value0 = rc_encoder_read_delta(1);
        new_value1 = rc_encoder_read_delta(2);
        new_value2 = rc_encoder_read_delta(3);
        total0 = rc_encoder_read_count(1);
        total1 = rc_encoder_read_count(2);
        total2 = rc_encoder_read_count(3);
        printf("\033[2A\r| Delta A | Delta B | Delta C | total A | total B | total C |\n\r| %7d | %7d | %7d | %7d | %7d | %7d |", new_value0, new_value1, new_value2, total0, total1, total2);
    }
    for (; d > -INT_16_MAX; d -= 64) {
        rc_motor_set(motor, d);
        sleep_ms(4);
        new_value0 = rc_encoder_read_delta(1);
        new_value1 = rc_encoder_read_delta(2);
        new_value2 = rc_encoder_read_delta(3);
        total0 = rc_encoder_read_count(1);
        total1 = rc_encoder_read_count(2);
        total2 = rc_encoder_read_count(3);
        printf("\033[2A\r| Delta A | Delta B | Delta C | total A | total B | total C |\n\r| %7d | %7d | %7d | %7d | %7d | %7d |", new_value0, new_value1, new_value2, total0, total1, total2);
    }
    for (; d < 0; d += 64) {
        rc_motor_set(motor, d);
        sleep_ms(4);
        new_value0 = rc_encoder_read_delta(1);
        new_value1 = rc_encoder_read_delta(2);
        new_value2 = rc_encoder_read_delta(3);
        total0 = rc_encoder_read_count(1);
        total1 = rc_encoder_read_count(2);
        total2 = rc_encoder_read_count(3);
        printf("\033[2A\r| Delta A | Delta B | Delta C | total A | total B | total C |\n\r| %7d | %7d | %7d | %7d | %7d | %7d |", new_value0, new_value1, new_value2, total0, total1, total2);
    }
}

void blink() {
    gpio_put(PICO_DEFAULT_LED_PIN, true);
    sleep_ms(500);
    gpio_put(PICO_DEFAULT_LED_PIN, false);
}

void reset_encoders() {
    rc_encoder_write(1, 0);
    rc_encoder_write(2, 0);
    rc_encoder_write(3, 0);
}
