/**
 * This file is the main executable for the MBot firmware.
 */

#include <pico/stdlib.h>
#include <pico/mutex.h>
#include <pico/multicore.h>
#include <rc/motor/motor.h>
#include <rc/encoder/encoder.h>
#include <rc/motor/motor.h>
#include <rc/mpu/mpu.h>
#include <comms/common.h>

#define LED_PIN 25

// controls wether we are printing locally 
// (e.g. we are using *just* the pico, and want human readable output on the USB out)
// comment this line out if we instead want to talk to the RPi via the USB connection
//#define PRINT_LOCAL

// struct & mutex for sensor data back to the RPi
static data_rpi feedback_data;
auto_init_mutex(feedback_mutex);
// struct & mutex for command data to the Pico
static data_pico command_data;
auto_init_mutex(command_mutex);

// data to hold current mpu state
static rc_mpu_data_t mpu_data;

data_rpi read_sensors(rc_mpu_data_t mpu_data)
{
    data_rpi sensor_data;
    sensor_data.encod1_pos = rc_encoder_read_count(1);
    sensor_data.encod2_pos = rc_encoder_read_count(2);
    sensor_data.encod3_pos = rc_encoder_read_count(3);
    sensor_data.encod1_delta = rc_encoder_read_delta(1);
    sensor_data.encod2_delta = rc_encoder_read_delta(2);
    sensor_data.encod3_delta = rc_encoder_read_delta(3);
    sensor_data.imu_tb_x = mpu_data.dmp_TaitBryan[0];
    sensor_data.imu_tb_y = mpu_data.dmp_TaitBryan[1];
    sensor_data.imu_tb_z = mpu_data.dmp_TaitBryan[2];

    return sensor_data;
}

/*
TODO: Refactor these mutex based getter/setter methods
to instead be in a library
*/
void update_sensor_data(data_rpi sensor_data_in)
{
    mutex_enter_blocking(&feedback_mutex);
    feedback_data = sensor_data_in;
    mutex_exit(&feedback_mutex);
}

void update_command_data(data_pico command_data_in)
{
    mutex_enter_blocking(&command_mutex);
    command_data = command_data_in;
    mutex_exit(&command_mutex);
}

data_rpi get_sensor_struct()
{
    data_rpi toReturn;
    mutex_enter_blocking(&feedback_mutex);
    toReturn = feedback_data;
    mutex_exit(&feedback_mutex);
    return toReturn;
}

data_pico get_command_struct()
{
    data_pico toReturn;
    mutex_enter_blocking(&command_mutex);
    toReturn = command_data;
    mutex_exit(&command_mutex);
    return toReturn;
}

/*
This function is launched on core 1 and handles all of the comms between the pico
and the RPi. It reads in data from core
*/
void core1_comms()
{
    // loop infinitely until we die
    while(1)
    {
        // get the current sensor data in a safe manner
        data_rpi loop_data = get_sensor_struct(mpu_data);
        // send the sensor data over the wire
        //rpi_serial_write(loop_data);

        // read in command data from the wire
        data_pico loop_cmd_data = {0};//pico_serial_read_blocking();
        // and set our current commands in a safe manner
        update_command_data(loop_cmd_data);
    }
}


int main() {
    bi_decl(bi_program_description("The main binary for the MBot Pico Board."));
    bi_decl(bi_1pin_with_name(LED_PIN, "On-board LED"));

    stdio_init_all();
    sleep_ms(1500); // quick sleep so we can catch the bootup process in terminal
    //printf("\nMBot Booting Up!\n");

    //printf("initing motors...\n");
    rc_motor_init();
    //printf("initing encoders...\n");
    rc_encoder_init();

    // Pins
    // for the i2c to the IMU
    const uint sda_pin = 4;
    const uint scl_pin = 5;

    // Ports
    i2c_inst_t *i2c = i2c0;
    //Initialize I2C port at 400 kHz
    i2c_init(i2c, 400 * 1000);
    // Initialize I2C pins
    //printf("setting i2c functions...\n");
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    //printf("setting i2c pullups...\n");
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(sda_pin, scl_pin, GPIO_FUNC_I2C));

    //printf("setting heartbeat LED GPIOs...\n");
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    //printf("resetting mpu...\n");
    mpu9250_reset(i2c);
    //printf("initing dmp...\n");
    mpu9250_init_dmp(i2c, &mpu_data); 

    // if we aren't trying to print to USB in human readable manner
    // then we need to setup the other core to run the communications loop
    #ifndef PRINT_LOCAL
    // launch the other core's comm loop
    multicore_launch_core1(core1_comms);
    #endif

    int running = 1;

    //printf("Done Booting Up!\n\n");
    while (running) {
        //spin...
        gpio_put(LED_PIN, 0);

        data_rpi loop_data = read_sensors(mpu_data);
        data_pico loop_cmd_data;

        #ifdef PRINT_LOCAL
            printf("\033[2A\r|   TB X  |   TB Y  |   TB Z  | Delta A | Delta B | Delta C | total A | total B | total C |\n\r| %7f | %7f | %7f | %7d | %7d | %7d | %7d | %7d | %7d |",
                loop_data.imu_tb_x,
                loop_data.imu_tb_y,
                loop_data.imu_tb_z,
                loop_data.encod1_delta,
                loop_data.encod2_delta,
                loop_data.encod3_delta,
                loop_data.encod1_pos,
                loop_data.encod2_pos,
                loop_data.encod3_pos);
        #else
            // set the sensor data struct
            update_sensor_data(loop_data);
            // get the command data struct
            loop_cmd_data = get_command_struct();
        #endif

        // assign the motors according to the command struct entries
        
        
        sleep_ms(250);
        gpio_put(LED_PIN, 1);
        sleep_ms(250);
    }
}
