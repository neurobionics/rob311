#include <comms/messages.h>

/*
mo_cmds_t commands serialize/deserialize
*/
int mo_cmds_t_deserialize(uint8_t* src, mo_cmds_t* dest)
{
    memcpy(dest, src, sizeof(mo_cmds_t));
    return 1;
}
int mo_cmds_t_serialize(mo_cmds_t* src, uint8_t* dest)
{
    memcpy(dest, src, sizeof(mo_cmds_t));
    return 1;
}


/*
mo_pid_params_t commands serialize/deserialize
*/
int mo_pid_params_t_deserialize(uint8_t* src, mo_pid_params_t* dest)
{
    memcpy(dest, src, sizeof(mo_pid_params_t));
    return 1;
}
int mo_pid_params_t_serialize(mo_pid_params_t* src, uint8_t* dest)
{
    memcpy(dest, src, sizeof(mo_pid_params_t));
    return 1;
}

/*
mo_pid_params_t commands serialize/deserialize
*/
int mo_states_t_deserialize(uint8_t* src, mo_states_t* dest)
{
    memcpy(dest, src, sizeof(mo_states_t));
    return 1;
}
int mo_states_t_serialize(mo_states_t* src, uint8_t* dest)
{
    memcpy(dest, src, sizeof(mo_states_t));
    return 1;
}