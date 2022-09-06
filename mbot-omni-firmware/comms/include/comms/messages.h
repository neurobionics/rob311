#include <stdio.h>
#include <stdint.h>
#include <memory.h>
#include <pico/binary_info.h> 

#ifndef MESSAGES_H
#define MESSAGES_H

typedef struct mo_cmds {
    double kill;
    double phi_roll_duty; // Roll Set Point
    double phi_pitch_duty; // Pitch Set Point
    double phi_yaw_duty; // Yaw Set Point
} mo_cmds_t;

int mo_cmds_t_deserialize(uint8_t* src, mo_cmds_t* dest);
int mo_cmds_t_serialize(mo_cmds_t* src, uint8_t* dest);

typedef struct mo_pid_params {
    double theta_kp;
    double theta_ki;
    double theta_kd;
} mo_pid_params_t;

int mo_pid_params_t_deserialize(uint8_t* src, mo_pid_params_t* dest);
int mo_pid_params_t_serialize(mo_pid_params_t* src, uint8_t* dest);

typedef struct mo_states {
    double timestep;
    double theta_roll;
    double theta_pitch;
    double theta_yaw;

    double dpsi_1;
    double dpsi_2;
    double dpsi_3;

    double psi_1;
    double psi_2;
    double psi_3;
} mo_states_t;

int mo_states_t_deserialize(uint8_t* src, mo_states_t* dest);
int mo_states_t_serialize(mo_states_t* src, uint8_t* dest);

#endif