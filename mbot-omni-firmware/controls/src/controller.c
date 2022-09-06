#include <controls/controller.h>

int init_controllers(rc_filter_t** base_filters, mo_pid_params_t pid_params)
{
    int to_return = 0;
    // to_return += rc_filter_pid(base_filters[0], pid_params.kp_x, pid_params.ki_x, pid_params.kd_x, CONTROLLER_TIMESTEP, (CONTROLLER_TIMESTEP / 0.5) - EPS);
    // to_return += rc_filter_pid(base_filters[1], pid_params.kp_v, pid_params.ki_v, pid_params.kd_v, CONTROLLER_TIMESTEP, (CONTROLLER_TIMESTEP / 0.5) - EPS);
    // to_return += rc_filter_pid(base_filters[2], pid_params.kp_i, pid_params.ki_i, pid_params.kd_i, CONTROLLER_TIMESTEP, (CONTROLLER_TIMESTEP / 0.5) - EPS);
    return to_return;
}

double run_controller(rc_filter_t** filters, mo_pid_params_t pid_params, mo_cmds_t setpoints, mo_states_t states, MotorFeedforward position_ff, MotorFeedforward velocity_ff, MotorFeedforward current_ff)
{
    // double vel_cmd = rc_filter_march(filters[0], states.x - setpoints.x_sp);
    // if(position_ff != NULL)
    // {
    //     vel_cmd += position_ff(setpoints, states);
    // }

    // double cur_cmd = rc_filter_march(filters[1], states.v - (setpoints.v_sp + vel_cmd));
    // if(velocity_ff != NULL)
    // {
    //     cur_cmd += position_ff(setpoints, states);
    // }
    // for(int i = 0; i < filters[2]->num.len; i++)
    // {
    //     printf("i filter num %d: %d\r\n", i, filters[2]->num.d[i]);
    // }
    // for(int i = 0; i < filters[2]->den.len; i++)
    // {
    //     printf("i filter den %d: %d\r\n", i, filters[2]->den.d[i]);
    // }
    double pwm_cmd = 0.0f; // rc_filter_march(filters[2], states.i - (setpoints.i_sp + cur_cmd));
    if(current_ff != NULL)
    {
        pwm_cmd += current_ff(setpoints, states);
    }
    return pwm_cmd;
}
