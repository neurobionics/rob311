#ifndef RC_MOTOR_H
#define RC_MOTOR_H

#define M0_DIR_PIN      14
#define M1_DIR_PIN      15
#define M2_DIR_PIN      13

#define M0_PWM_PIN      2
#define M1_PWM_PIN      3
#define M2_PWM_PIN      12

#define M0_SLICE        1
#define M1_SLICE        1
#define M2_SLICE        6

#define M0_CHAN         0   //  A
#define M1_CHAN         1   //  B
#define M2_CHAN         0   //  A

#define CLOCK           125000000

#define DEFAULT_FREQ    25000

typedef enum rc_motor_state {ON, OFF} rc_motor_state;

int rc_motor_init();
int rc_motor_init_freq(uint f);
int rc_motor_cleanup();
int rc_motor_set(uint ch, int16_t duty);
int rc_motor_free_spin(uint ch);
int rc_motor_brake(uint ch);

#endif /* RC_MOTOR_H */
