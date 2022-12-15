import numpy as np

mo_cmds_dtype =  np.dtype([
    ("start", np.double),
    ("motor_1_duty", np.double),
    ("motor_2_duty", np.double),
    ("motor_3_duty", np.double)
])

mo_states_dtype =  np.dtype([  
    ("timestep", np.double),
    ("theta_roll", np.double),
    ("theta_pitch", np.double),
    ("theta_yaw", np.double),
    ("dpsi_1", np.double),
    ("dpsi_2", np.double),
    ("dpsi_3", np.double),
    ("psi_1", np.double),
    ("psi_2", np.double),
    ("psi_3", np.double)
])

mo_pid_params_dtype =  np.dtype([
    ("theta_kp", np.double),
    ("theta_ki", np.double),
    ("theta_kd", np.double)
])