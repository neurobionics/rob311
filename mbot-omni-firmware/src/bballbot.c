/**
 * This file is the main executable for the MBot firmware.
 */

#include <pico/stdlib.h>
#include <pico/mutex.h>
#include <pico/multicore.h>
#include <rc/motor/motor.h>
#include <rc/encoder/encoder.h>
#include <rc/current/current.h>
#include <rc/motor/motor.h>
#include <rc/mpu/mpu.h>
#include <comms/common.h>
#include <comms/protocol.h>
#include <comms/listener.h>
#include <comms/messages.h>
#include <comms/topic_data.h>
#include <controls/controller.h>
#include <math.h>

#define INT_16_MAX 32768

#define MAX_PWM (INT_16_MAX - 128)
#define MIN_PWM 1240

#define DMP_SATURATION_TIME 10.0f //25.0f

#define Rw 0.048f // Radius of the Wheel
#define Rk 0.1210f 
#define ALPHA 0.7854f

#define ENCODER_RESOLUTION 64
#define GEAR_RATIO 70
#define FREQ 240.0f
#define DT_S (double)1/FREQ
#define DT_MS (int)1000/FREQ //in ms [FOR COMPUTATION USE THE ACTUAL TIMESTEP VARIABLE]
#define DT_US (int)1000000/FREQ //in ms [FOR COMPUTATION USE THE ACTUAL TIMESTEP VARIABLE]
#define TF (double)(2.0 * 0.005) //5, 10, 50 Hz

// 3 deg = 0.0523599 rad
// 2 deg = 0.0349066 rad

#define DUTY_SATURATION 1.4f // Duty cycle saturation limit for each plane (-1.4% to 1.4% maps to -0.9% to 0.9% fr each)

// data to hold current mpu state
static mb_mpu_data_t mpu_data;

static double theta_roll_offset = 0.0f;
static double theta_pitch_offset = 0.0f;
static double theta_yaw_offset = 0.0f;

static absolute_time_t tic = {0};
static absolute_time_t toc = {0};

rc_filter_t theta_roll_command = RC_FILTER_INITIALIZER;
rc_filter_t theta_pitch_command = RC_FILTER_INITIALIZER;

static double roll_duty;
static double pitch_duty;
static double yaw_duty = 0.0f;

static double theta_roll_sp = 0.0f;
static double theta_pitch_sp = 0.0f;

// static double motor_1_duty;
// static double motor_2_duty;
// static double motor_3_duty;

static int motor_1_pwm;
static int motor_2_pwm;
static int motor_3_pwm;

// Initializing structs
static mo_cmds_t mo_cmd = {0};
static mo_pid_params_t mo_pid = {0};
static mo_states_t mo_state = {0};

static double timestamp = 0.0f;
static double timestep;

double compute_psi(int enc) {
    // Position is negative when duty cycle is positive, hence the "-1 *"
    return (-1 * 2 * PI * enc)/(GEAR_RATIO * ENCODER_RESOLUTION);
}

double compute_dpsi(int delta, double timestep) {
    // Delta is negative when duty cycle is positive, hence the "-1 *"
    return (-1 * 2 * PI * delta)/(timestep * GEAR_RATIO * ENCODER_RESOLUTION);
}

int compute_pwm(double duty_cycle) {
    if (duty_cycle >= 0.0f) {
        return ((MAX_PWM - MIN_PWM) * duty_cycle + MIN_PWM);
    }
    else {
        return ((MAX_PWM - MIN_PWM) * duty_cycle - MIN_PWM);
    }
}

void register_topics()
{
    // Commands, States
    comms_register_topic((uint16_t)101, sizeof(mo_cmds_t), (Deserialize)&mo_cmds_t_deserialize, (Serialize)&mo_cmds_t_serialize);
    comms_register_topic((uint16_t)121, sizeof(mo_states_t), (Deserialize)&mo_states_t_deserialize, (Serialize)&mo_states_t_serialize);
}

int main() {
    bi_decl(bi_program_description("Binary for ROB 311 Ball Balance Bot."));
    stdio_init_all();
    sleep_ms(1500); // quick sleep so we can catch the bootup process in terminal

    printf("Booting up...\r\n");
    rc_encoder_init();
    rc_current_sense_init();

    static int32_t start = 0;
    int cmd_success = true;

    // LED
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    // Pins
    // for the i2c to the IMU
    i2c_inst_t *i2c = i2c0;
    // Initialize I2C pins
    i2c_init(i2c, 400 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

    // Reset the mpu using the intialized I2C, and init the motion processor    
    // give the dmp the location of where to put data
    // printf("Initializing DMP!\r\n");
    mb_mpu_config_t mpu_config = mb_mpu_default_config();
    mpu_config.i2c_bus = i2c;
    mpu_config.dmp_fetch_accel_gyro=1;
    mpu_config.enable_magnetometer = 0;
    mpu_config.read_mag_after_callback = 0;
    mpu_config.orient = ORIENTATION_Z_UP;
    mpu_config.dmp_sample_rate = 200;

    sleep_ms(1000);
    mb_mpu_initialize_dmp(&mpu_data, mpu_config); 
    gpio_set_irq_enabled_with_callback(MB_MPU_INTERRUPT_GPIO, GPIO_IRQ_EDGE_FALL, true, &mb_dmp_callback);
    sleep_ms(100);

    //create topics and register the serialize/deserialize functions
    printf("Initializing Communication with RPi.\r\n");
    comms_init_protocol();
    comms_init_topic_data();
    register_topics();

    // launch the other core's comm loop
    printf("Launching communication thread [CORE 1].\r\n");
    multicore_launch_core1(comms_listener_loop);

    // wait for other core to get rolling
    sleep_ms(500);

    while (true) {

        rc_motor_init();
        cmd_success &= comms_get_topic_data(101, &mo_cmd);
        start = mo_cmd.start;

        while (start>0.5f) {
            toc = get_absolute_time();
            timestep = absolute_time_diff_us(tic, toc) * 0.000001f; // Microseconds to Seconds

            if (timestep > 2*DT_S) {
                timestep = DT_S;
            }
            else if (timestep < 0.5*DT_S) {
                timestep = DT_S;
            }

            timestamp += timestep;

            if(timestamp < DMP_SATURATION_TIME) {
                printf("Calibrating calibrating calibrating!\r\n");
                // Get the initial offsets for IMU-TB values
                theta_roll_offset = mpu_data.dmp_TaitBryan[0];
                theta_pitch_offset = mpu_data.dmp_TaitBryan[1];
                theta_yaw_offset = mpu_data.dmp_TaitBryan[2];

                gpio_put(PICO_DEFAULT_LED_PIN, true);
            }
            else
            {
                gpio_put(PICO_DEFAULT_LED_PIN, false);

                mo_state.timestep = timestep;

                // IMU DMP DATA: CHECK
                mo_state.theta_roll = 1.0 * (mpu_data.dmp_TaitBryan[0] - theta_roll_offset);
                mo_state.theta_pitch = 1.0 * (mpu_data.dmp_TaitBryan[1] - theta_pitch_offset);
                mo_state.theta_yaw = 0.0; // LOCAL
                
                // MOTOR STATES: CHECK
                mo_state.psi_1 = compute_psi(rc_encoder_read_count(1));
                mo_state.dpsi_1 = compute_dpsi(rc_encoder_read_delta(1), DT_S);

                mo_state.psi_2 = compute_psi(rc_encoder_read_count(2));
                mo_state.dpsi_2 = compute_dpsi(rc_encoder_read_delta(2), DT_S);

                mo_state.psi_3 = compute_psi(rc_encoder_read_count(3));
                mo_state.dpsi_3 = compute_dpsi(rc_encoder_read_delta(3), DT_S);

                // SEND STATES
                comms_write_topic(121, &mo_state);

                // GET COMMANDS
                cmd_success &= comms_get_topic_data(101, &mo_cmd);
                start = mo_cmd.start;

                printf("\r");
                printf("Roll: %7.3f, Pitch: %7.3f | ", mo_state.theta_roll, mo_state.theta_pitch);

                motor_1_pwm = compute_pwm(mo_cmd.motor_1_duty);
                motor_2_pwm = compute_pwm(mo_cmd.motor_2_duty);
                motor_3_pwm = compute_pwm(mo_cmd.motor_3_duty);

                printf("MOTOR 1: %7.3f, MOTOR 2: %7.3f, MOTOR 3: %7.3f", mo_cmd.motor_1_duty, mo_cmd.motor_2_duty, mo_cmd.motor_3_duty);

                rc_motor_set(1, motor_1_pwm);
                rc_motor_set(2, motor_2_pwm);
                rc_motor_set(3, motor_3_pwm);

            }

            tic = toc;
            sleep_us(DT_US);
        }

        printf("Exiting!\n");
        timestamp = 0.0f;
        rc_motor_cleanup();

    }
    
    return 0;      
}