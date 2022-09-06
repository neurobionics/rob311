import sys
import threading
import time
import numpy as np
from threading import Thread
from MBot.Messages.message_defs import mo_states_dtype, mo_cmds_dtype, mo_pid_params_dtype
from MBot.SerialProtocol.protocol import SerialProtocol
from rtplot import client
from scipy.signal import butter, lfilter
from simple_pid import PID
from pyPS4Controller.controller import Controller
import board
import adafruit_dotstar as dotstar

# CONTROL BANDWITH: 0.85943669 Degrees

JOYSTICK_SCALE = 32767

FREQ = 200
DT = 1/FREQ

RW = 0.096
RK = 0.1210
ALPHA = np.deg2rad(45)

N_DOTS = 72
MAX_BRIGHTNESS = 0.055
MIN_BRIGHTNESS = 0.01

MAX_TILT = np.deg2rad(5) # Maximum inclination: 5 degrees
MAX_LINEAR_VELOCITY = 0.7 # m/s --> Corresponds to lean angle as of now.

MAX_DUTY = 0.8

ARC_START = np.deg2rad(15)
ARC_STOP = 2*np.pi - np.deg2rad(15)

ARC = ARC_STOP - ARC_START
ARC_PER_DOT = ARC/N_DOTS

THETA_KP = 20.0
THETA_KI = 0.0
THETA_KD = 0.1

DPHI_KP_PITCH = 0.0
DPHI_KI_PITCH = 0.0
DPHI_KD_PITCH = 0.0

DPHI_KP_ROLL = 0.0
DPHI_KI_ROLL = 0.0
DPHI_KD_ROLL = 0.0

BUTTERWORTH_ORDER = 2
BUTTERWORTH_CUTOFF_HZ = 30
BUTTERWORTH_PADDING = 12

BUTTERWORTH_NYQ = 0.5 * FREQ
BUTTERWORTH_CUTOFF = BUTTERWORTH_CUTOFF_HZ/BUTTERWORTH_NYQ
BUTTERWORTH_BF, BUTTERWORTH_AF = butter(BUTTERWORTH_ORDER, BUTTERWORTH_CUTOFF, btype='low', analog=False)

BUTTERWORTH_SP_ORDER = 2
BUTTERWORTH_SP_CUTOFF_HZ = 2
BUTTERWORTH_SP_PADDING = 12

BUTTERWORTH_SP_NYQ = 0.5 * FREQ
BUTTERWORTH_SP_CUTOFF = BUTTERWORTH_SP_CUTOFF_HZ/BUTTERWORTH_SP_NYQ
BUTTERWORTH_SP_BF, BUTTERWORTH_SP_AF = butter(BUTTERWORTH_SP_ORDER, BUTTERWORTH_SP_CUTOFF, btype='high', analog=False)

J11 = -2 * RW/(3 * RK * np.cos(ALPHA))
J12 = RW / (3 * RK * np.cos(ALPHA))
J13 = J12
J21 = 0
J22 = -np.sqrt(3) * RW/ (3 * RK * np.cos(ALPHA))
J23 = -1 * J22
J31 = RW / (3 * RK * np.sin(ALPHA))
J32 = J31
J33 = J31

J = np.array([[J11, J12, J13], [J21, J22, J23], [J31, J32, J33]])

class MoController(Controller):
    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)
        self.roll_velocity = 0.0
        self.pitch_velocity = 0.0

    def on_L3_right(self, value):
        # VOID #
        pass

    def on_L3_left(self, value):
        # VOID #
        pass

    def on_L3_up(self, value):
        # VOID #
        pass

    def on_L3_down(self, value):
        # VOID #
        pass

    def on_L3_x_at_rest(self):
        # VOID #
        pass

    def on_L3_y_at_rest(self):
        # VOID #
        pass

    def on_R3_up(self, value):
        self.pitch_velocity = -1.0 * MAX_LINEAR_VELOCITY * (value/JOYSTICK_SCALE)

    def on_R3_down(self, value):
        self.pitch_velocity = -1.0 * MAX_LINEAR_VELOCITY * (value/JOYSTICK_SCALE)

    def on_R3_right(self, value):
        self.roll_velocity = MAX_LINEAR_VELOCITY * (value/JOYSTICK_SCALE)
        pass

    def on_R3_left(self, value):
        self.roll_velocity = MAX_LINEAR_VELOCITY * (value/JOYSTICK_SCALE)
        pass

    def on_R3_x_at_rest(self):
        self.roll_velocity = 0.0

    def on_R3_y_at_rest(self):
        self.pitch_velocity = 0.0

    def on_options_press(self):
        print("Exiting controller thread.")
        sys.exit()

def register_topics(ser_dev:SerialProtocol):
    # Mo :: Commands, Gains, States
    ser_dev.serializer_dict[101] = [lambda bytes: np.frombuffer(bytes, dtype=mo_cmds_dtype), lambda data: data.tobytes()]
    ser_dev.serializer_dict[111] = [lambda bytes: np.frombuffer(bytes, dtype=mo_pid_params_dtype), lambda data: data.tobytes()]
    ser_dev.serializer_dict[121] = [lambda bytes: np.frombuffer(bytes, dtype=mo_states_dtype), lambda data: data.tobytes()]

def init_lights(brightness):
        dots = dotstar.DotStar(board.SCK, board.MOSI, N_DOTS, brightness=brightness)
        dots.fill(color=(0, 0, 0))
        dots.show()

        return dots

def compute_dots(roll, pitch):
        x = np.sin(roll)
        y = np.sin(pitch)

        slope = np.arctan(y/x)

        if y >= 0 and x >= 0:
                dot_position = np.pi/2 - slope
        elif y >= 0 and x <= 0:
                dot_position = 3/2 * np.pi - slope
        elif y <= 0 and x >= 0:
                dot_position = np.pi/2 - slope
        elif y <= 0 and x <= 0:
                dot_position = 3/2 * np.pi - slope

        dot_intensity = (abs(np.sin(roll)) + abs(np.sin(pitch)))/(2 * abs(np.sin(MAX_TILT)))
        center_dot = int((dot_position - ARC_START)/ARC_PER_DOT)
        half_dots = int(dot_intensity * N_DOTS/2)

        center_start = center_dot - half_dots
        center_stop = center_dot + half_dots + 1

        if center_start < 0:
                center_start = 0

        if center_stop > N_DOTS:
                center_stop = N_DOTS

        dots = np.arange(center_start, center_stop)
        return dots

if __name__ == "__main__":

    imu_states = {'names': ['Roll', 'Pitch'],
                    'title': "Orientation",
                    'ylabel': "rad",
                    'xlabel': "time",
                    'colors' : ["r", "g"],
                    'line_width': [2]*2,
                    'yrange': [-2.0 * np.pi, 2.0 * np.pi]
                    }

    motor_states = {'names': ['Motor 1', 'Motor 2', 'Motor 3'],
                    'title': "Angular Velocity",
                    'ylabel': "rad/sec",
                    'xlabel': "time",
                    'colors' : ["r", "g", "b"],
                    'line_width': [2]*3,
                    'yrange': [-2.0 * np.pi, 2.0 * np.pi]
                    }

    filter_states = {'names': ['Motor 1 Raw', 'Motor 1 Filtered', 'Motor 2 Raw', 'Motor 2 Filtered', 'Motor 3 Raw', 'Motor 3 Filtered'],
                    'title': "Butterworth Filter",
                    'ylabel': "rad/sec",
                    'xlabel': "time",
                    'colors' : ["r", "g", "b", "y", "m", "c"],
                    'line_width': [2]*6,
                    'yrange': [-2.0 * np.pi, 2.0 * np.pi]
                    }

    dphi_filtering = {'names': ['Ball Roll Raw', 'Ball Roll Filtered', 'Ball Pitch Raw', 'Ball Pitch Filtered'],
                    'title': "dPhi Filtering",
                    'ylabel': "m/s",
                    'xlabel': "time",
                    'colors' : ["r", "g", "b", "y"],
                    'line_width': [2]*4,
                    # 'yrange': [-MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY]
                    }

    dphi_roll_controller = {'names': ['Proportional', 'Integral', 'Derivative', 'Controller Output'],
                    'title': "Dphi Roll",
                    'ylabel': "rad",
                    'xlabel': "time",
                    'colors' : ["r", "g", "b", "y"],
                    'line_width': [2]*4,
                    'yrange': [-MAX_TILT, MAX_TILT]
                    }

    velocity_controller = {'names': ['SP Ball Roll', 'Ball Roll', 'SP Ball Pitch', 'Ball Pitch'],
                    'title': "Velocity Controller",
                    'ylabel': "m",
                    'xlabel': "time",
                    'colors' : ["r", "g", "b", "y"],
                    'line_width': [2]*4,
                    # 'yrange': [-MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY]
                    }

    stability_controller = {'names': ['SP Body Roll', 'Body Roll', 'SP Body Pitch', 'Body Pitch'],
                    'title': "Stability Controller",
                    'ylabel': "rad",
                    'xlabel': "time",
                    'colors' : ["r", "g", "b", "y"],
                    'line_width': [2]*4,
                    'yrange': [-MAX_TILT, MAX_TILT]
                    }

    roll_control = {'names': ['Roll Setpoint', 'Roll'],
                    'title': "Roll PID Controller",
                    'ylabel': "rad",
                    'xlabel': "time",
                    'colors' : ["r", "b"],
                    'line_width': [2]*2,
                    'yrange': [-0.5 * np.pi, 0.5 * np.pi]
                    }

    pitch_control = {'names': ['Pitch Setpoint', 'Pitch'],
                    'title': "Pitch PID Controller",
                    'ylabel': "rad",
                    'xlabel': "time",
                    'colors' : ["r", "b"],
                    'line_width': [2]*2,
                    'yrange': [-0.5 * np.pi, 0.5 * np.pi]
                    }

    plot_config = [velocity_controller]
    client.initialize_plots(plot_config)

    ser_dev = SerialProtocol()
    register_topics(ser_dev)

    # Init serial
    serial_read_thread = Thread(target = SerialProtocol.read_loop, args=(ser_dev,), daemon=True)
    serial_read_thread.start()

    # Local structs
    setpoints = np.zeros(1, dtype=mo_cmds_dtype)[0]
    gains = np.zeros(1, dtype=mo_pid_params_dtype)[0]
    states = np.zeros(1, dtype=mo_states_dtype)[0]

    gains['theta_kp'] = THETA_KP
    gains['theta_ki'] = THETA_KI
    gains['theta_kd'] = THETA_KD

    setpoints['kill'] = 0.0

    # Time for comms to sync
    time.sleep(1.0)

    # Send the gains 
    ser_dev.send_topic_data(111, gains)
    ser_dev.send_topic_data(101, setpoints)

    local_time = 0
    pico_dt = DT

    dpsi = np.zeros((3, 1))
    dphi = np.zeros((3, 1))

    dphi_roll = 0.0
    dphi_pitch = 0.0

    danger = []

    zeroed = False
    home = False

    dphi_zero = np.zeros((3, 1))
    dpsi_offset = np.zeros((3, 1))

    dphi_roll_array = np.zeros(BUTTERWORTH_PADDING)
    dphi_pitch_array = np.zeros(BUTTERWORTH_PADDING)

    dphi_roll_sp_array = np.zeros(BUTTERWORTH_PADDING)
    dphi_pitch_sp_array = np.zeros(BUTTERWORTH_PADDING)

    filtered_dphi_roll = 0.0
    filtered_dphi_pitch = 0.0

    filtered_dphi_roll_sp = 0.0
    filtered_dphi_pitch_sp = 0.0

    dphi_roll_sp = 0.0
    dphi_pitch_sp = 0.0

    dphi_roll_pid = PID(DPHI_KP_ROLL, DPHI_KI_ROLL, DPHI_KD_ROLL, dphi_roll_sp)
    dphi_pitch_pid = PID(DPHI_KP_PITCH, DPHI_KI_PITCH, DPHI_KD_PITCH, dphi_pitch_sp)

    dphi_roll_pid.output_limits = (-MAX_DUTY, MAX_DUTY)
    dphi_pitch_pid.output_limits = (-MAX_DUTY, MAX_DUTY)
    
    mo_controller = MoController(interface="/dev/input/js0", connecting_using_ds4drv=False)
    mo_controller_thread = threading.Thread(target=mo_controller.listen, args=(10,))
    mo_controller_thread.start()

    dots = init_lights(MAX_BRIGHTNESS)

    while(True):
        try:
            try:
                states = ser_dev.get_cur_topic_data(121)[0]
            except KeyError as e:
                print("<< CALIBRATING >>")
                dots.fill(color=(255, 191, 0))
                dots.show()
                continue

            # print("<< ROLLING >>")

            pico_dt = states['timestep']
            local_time += pico_dt

            dpsi[0] = states['psi_1']
            dpsi[1] = states['psi_2']
            dpsi[2] = states['psi_3']

            dphi = np.matmul(J, dpsi)
            dphi[1] = -1.0 * dphi[1]
            dphi = RK * dphi

            if not zeroed:
                print("SLEEPING!!!!")
                dots.fill(color=(255, 80, 10))
                dots.show()                
                time.sleep(1)

                dpsi[0] = states['psi_1']
                dpsi[1] = states['psi_2']
                dpsi[2] = states['psi_3']

                dphi_zero = np.matmul(J, dpsi)
                dphi_zero[1] = -1.0 * dphi_zero[1]
                dphi_zero = RK * dphi_zero

                dphi_roll_pid.setpoint = dphi_zero[0][0]
                dphi_pitch_pid.setpoint = dphi_zero[1][0]

                print(dphi_roll_pid.setpoint, dphi_pitch_pid.setpoint)
                zeroed = True
                print("Rock and roll!")


            dphi_roll_array = np.delete(dphi_roll_array, 0, axis=0)
            dphi_roll_array = np.append(dphi_roll_array, dphi[0], axis=0)

            dphi_pitch_array = np.delete(dphi_pitch_array, 0, axis=0)
            dphi_pitch_array = np.append(dphi_pitch_array, dphi[1], axis=0)

            dphi_roll_sp_array = np.delete(dphi_roll_sp_array, 0, axis=0)
            dphi_roll_sp_array = np.append(dphi_roll_sp_array, [mo_controller.roll_velocity], axis=0)

            dphi_pitch_sp_array = np.delete(dphi_pitch_sp_array, 0, axis=0)
            dphi_pitch_sp_array = np.append(dphi_pitch_sp_array, [mo_controller.pitch_velocity], axis=0)

            filtered_dphi_roll = lfilter(BUTTERWORTH_BF, BUTTERWORTH_AF, dphi_roll_array)[-1]
            filtered_dphi_pitch = lfilter(BUTTERWORTH_BF, BUTTERWORTH_AF, dphi_pitch_array)[-1]

            filtered_dphi_roll_sp = lfilter(BUTTERWORTH_SP_BF, BUTTERWORTH_SP_AF, dphi_roll_sp_array)[-1]
            filtered_dphi_pitch_sp = lfilter(BUTTERWORTH_SP_BF, BUTTERWORTH_SP_AF, dphi_pitch_sp_array)[-1]

            dphi_roll_pid.setpoint += mo_controller.roll_velocity * DT
            dphi_pitch_pid.setpoint += mo_controller.pitch_velocity * DT

            # if np.abs(states['theta_roll']) > MAX_TILT or np.abs(states['theta_pitch']) > MAX_TILT:
            #     dphi_roll_pid.setpoint += 0.0
            #     dphi_pitch_pid.setpoint += 0.0
            # else:
            #     dphi_roll_pid.setpoint += mo_controller.roll_velocity * DT
            #     dphi_pitch_pid.setpoint += mo_controller.pitch_velocity * DT

            if np.abs(states['theta_roll']) > MAX_TILT or np.abs(states['theta_pitch']) > MAX_TILT:
                setpoints['phi_roll_sp'] = 0.0 # dphi_roll_pid(filtered_dphi_roll)
                setpoints['phi_pitch_sp'] = 0.0 # dphi_pitch_pid(filtered_dphi_pitch)
            else:
                setpoints['phi_roll_sp'] = dphi_roll_pid(filtered_dphi_roll)
                setpoints['phi_pitch_sp'] = dphi_pitch_pid(filtered_dphi_pitch)


            # setpoints['phi_roll_sp'] = filtered_dphi_roll_sp # dphi_roll_pid(filtered_dphi_roll)
            # setpoints['phi_pitch_sp'] = filtered_dphi_pitch_sp # dphi_pitch_pid(filtered_dphi_pitch)

            print(mo_controller.roll_velocity, mo_controller.pitch_velocity)

            # data = [states['theta_roll'], states['theta_pitch'], states['theta_yaw'], states['dpsi_1'], states['dpsi_2'], states['dpsi_3']]
            # data = [setpoints['theta_roll_sp'], states['theta_roll'], setpoints['theta_pitch_sp'], states['theta_pitch']]

            ser_dev.send_topic_data(101, setpoints)

            # data = [dphi_roll_array[-1], filtered_dphi_roll, dphi_pitch_array[-1], filtered_dphi_pitch]
            data = [dphi_roll_sp_array[-1], setpoints['phi_roll_sp'], dphi_pitch_sp_array[-1], setpoints['phi_pitch_sp']]
            # data = [0.0, states['theta_roll'], 0.0, states['theta_pitch']]
            # data = [states['theta_roll'], states['theta_pitch']]

            client.send_array(data)

            if np.abs(states['theta_roll']) != 0.0:
                danger = compute_dots(states['theta_roll'], -1.0 * states['theta_pitch'])

            for dot in range(N_DOTS):
                if dot in danger:
                        dots[dot] = (255, 20, 20)
                else:
                        dots[dot] = (53, 118, 174)

            time.sleep(DT)
            dots.show()

        except KeyboardInterrupt as key:
            print("Resetting Mo commands.")
            setpoints['kill'] = 1.0
            setpoints['phi_roll_sp'] = 0.0
            setpoints['phi_pitch_sp'] = 0.0
            ser_dev.send_topic_data(101, setpoints)

            dots.fill(color=(0, 0, 0))
            dots.show()            

            print("Exiting Outer Loop")
            exit(0)