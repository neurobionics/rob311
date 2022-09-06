/*
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
    This file contains global IO structs, common definitions, and common functions that much of ROS Serial depends on.
    Notes:
        *RPI_IN_MSG --> RPI_IN_LENG?
        *PICO_IN_MSG --> PICO_IN_LENG?
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
*/

#include <stdio.h>
#include <stdint.h>
#include <memory.h>
#include <pico/binary_info.h>

#ifndef COMMONS_H
#define COMMONS_H

// struct containing all data read in by pico, sent by rpi (robot frame velocity commands)
typedef struct data_pico {
    int32_t vx;
    int32_t vz;
    int32_t vtheta;
} data_pico;

// struct containing all data read in by rpi, sent by pico (imu data, encoder data, odometry)
typedef struct data_rpi {
    int32_t imu_a_x;
    int32_t imu_a_y;
    int32_t imu_a_z;
    int32_t imu_a_pitch;
    int32_t imu_a_roll;
    int32_t imu_a_yaw;
    int32_t imu_tb_x;
    int32_t imu_tb_y;
    int32_t imu_tb_z;
    int32_t encod1_pos;
    int32_t encod2_pos;
    int32_t encod3_pos;
    int32_t encod1_delta;
    int32_t encod2_delta;
    int32_t encod3_delta;
    int32_t odometry_x;
    int32_t odometry_z;
    int32_t odometry_theta;
    int32_t odometry_vx;
    int32_t odometry_vz;
    int32_t odometry_vtheta;
} data_rpi;

// initialize global structs for storing most recently read in/calculated data
extern data_pico in_pico; //stores info being read into the pico from the RPI
extern data_rpi out_pico; //stores info being calculated on the pico to be sent to the RPI
extern data_rpi in_rpi; //stores info being read into the RPI from the pico
extern data_pico out_rpi; //stores info being calculated on the RPI to be sent to the pico
#endif


// definitions
#define SYNC_FLAG       0xff //beginning of packet sync flag
#define VERSION_FLAG    0xfe //version flag compatible with ROS2
#define SENSORS_TOPIC   101 //default message topic when a message is sent from the pico to the rpi
#define COMMANDS_TOPIC  102 //default message topic when a message is sent from the rpi to the pico

#define PICO_IN_MSG     sizeof(data_pico) //equal to the size of the data_pico struct
#define RPI_IN_MSG      sizeof(data_rpi) //equal to the size of the data_rpi struct
#define ROS_HEADER_LENGTH 7
#define ROS_FOOTER_LENGTH 1
#define ROS_PKG_LENGTH  (ROS_HEADER_LENGTH + ROS_FOOTER_LENGTH) //length (in bytes) of ros packaging (header and footer)
#define PICO_IN_BYTES   (PICO_IN_MSG + ROS_PKG_LENGTH) //equal to the size of the data_pico struct data plus bytes for ros packaging
#define RPI_IN_BYTES    (RPI_IN_MSG + ROS_PKG_LENGTH) //equal to the size of the data_rpi struct data plus bytes for ros packaging

// specific checksum method as defined by http://wiki.ros.org/rosserial/Overview/Protocol
uint8_t checksum(uint8_t* addends, int len);

// converts an array of four uint8_t members to an int32_t
int32_t bytes_to_int32(uint8_t bytes[4]);

// converts an int32_t to an array of four uint8_t members
uint8_t* int32_to_bytes(int32_t i32t);

// decodes a complete ROS packet into bytes array 'MSG' and uint16 topic as defined by http://wiki.ros.org/rosserial/Overview/Protocol
int decode_rospkt(uint8_t* ROSPKT, int rospkt_len, uint16_t* TOPIC, uint8_t* MSG, int msg_len);

// encodes a message and topic into a bytes array 'ROSPKT' as defined by http://wiki.ros.org/rosserial/Overview/Protocol
int encode_msg(uint8_t* MSG, int msg_len, uint16_t TOPIC, uint8_t* ROSPKT, int rospkt_len);