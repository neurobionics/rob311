#define PICO_STDIO_ENABLE_CRLF_SUPPORT 0

#include <stdio.h>
#include <stdint.h>
#include <pico/stdlib.h>
#include <pico/multicore.h>
#include <comms/common.h>
#include <comms/protocol.h>
#include <comms/listener.h>
#include <comms/topic_data.h>

#define LED_PIN 25


int deserialize_test(uint8_t* src, data_rpi* dest)
{
    memcpy(dest, src, sizeof(data_rpi));
    return 1;
}

int serialize_test(data_rpi* src, uint8_t* dest)
{
    memcpy(dest, src, sizeof(data_rpi));
    return 1;
}

int main() {
    bi_decl(bi_program_description("This is a test binary."));
    bi_decl(bi_1pin_with_name(LED_PIN, "On-board LED"));

    stdio_init_all();
    sleep_ms(1500); // quick sleep so we can catch the bootup process in terminal

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    data_rpi test_data = {0};

    comms_init_protocol();
    comms_register_topic((uint16_t)101, sizeof(data_rpi), (Deserialize)&deserialize_test, (Serialize)&serialize_test);
    comms_register_topic((uint16_t)102, sizeof(data_rpi), (Deserialize)&deserialize_test, (Serialize)&serialize_test);

    comms_init_topic_data();

    // launch the listener on the other core
    multicore_launch_core1(comms_listener_loop);
    
    while (1) {
        //spin...
        gpio_put(LED_PIN, 1);
        //get the current state sent to us
        while(comms_get_topic_data((uint16_t)101, &test_data) == 0)
        {
            //spin
            sleep_ms(10);
        }
        
        sleep_ms(250);

        gpio_put(LED_PIN, 0);
        // and resend it out
        comms_write_topic((uint16_t)102, &test_data);
        sleep_ms(250);
    }
}
