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

RW = 0.0048
RK = 0.1210
ALPHA = np.deg2rad(45)

N_DOTS = 72
MAX_BRIGHTNESS = 0.055
MIN_BRIGHTNESS = 0.01

MAX_TILT = np.deg2rad(5) # Maximum inclination: 5 degrees
MAX_LINEAR_VELOCITY = 0.7 # m/s --> Corresponds to duty cycle as for now.

MAX_DUTY = 0.7

ARC_START = np.deg2rad(15)
ARC_STOP = 2*np.pi - np.deg2rad(15)

ARC = ARC_STOP - ARC_START
ARC_PER_DOT = ARC/N_DOTS

THETA_KP = 8.0
THETA_KI = 0.0
THETA_KD = 0.0

# ---------------------------------------------------------------------------
# Gray C. Thomas, Ph.D's Soft Real Time Loop
# This library will soon be hosted as a PIP module and added as a python dependency.
# https://github.com/UM-LoCoLab/NeuroLocoMiddleware/blob/main/SoftRealtimeLoop.py

"""
Soft Realtime Loop---a class designed to allow clean exits from infinite loops
with the potential for post-loop cleanup operations executing.

The Loop Killer object watches for the key shutdown signals on the UNIX operating system (which runs on the PI)
when it detects a shutdown signal, it sets a flag, which is used by the Soft Realtime Loop to stop iterating.
Typically, it detects the CTRL-C from your keyboard, which sends a SIGTERM signal.

the function_in_loop argument to the Soft Realtime Loop's blocking_loop method is the function to be run every loop.
A typical usage would set function_in_loop to be a method of an object, so that the object could store program state.
See the 'ifmain' for two examples.

Author: Gray C. Thomas, Ph.D
https://github.com/GrayThomas, https://graythomas.github.io
"""

import signal
import time
from math import sqrt

PRECISION_OF_SLEEP = 0.0001

# Version of the SoftRealtimeLoop library
__version__ = "1.0.0"

class LoopKiller:
    def __init__(self, fade_time=0.0):
        signal.signal(signal.SIGTERM, self.handle_signal)
        signal.signal(signal.SIGINT, self.handle_signal)
        signal.signal(signal.SIGHUP, self.handle_signal)
        self._fade_time = fade_time
        self._soft_kill_time = None

    def handle_signal(self, signum, frame):
        self.kill_now = True

    def get_fade(self):
        # interpolates from 1 to zero with soft fade out
        if self._kill_soon:
            t = time.time() - self._soft_kill_time
            if t >= self._fade_time:
                return 0.0
            return 1.0 - (t / self._fade_time)
        return 1.0

    _kill_now = False
    _kill_soon = False

    @property
    def kill_now(self):
        if self._kill_now:
            return True
        if self._kill_soon:
            t = time.time() - self._soft_kill_time
            if t > self._fade_time:
                self._kill_now = True
        return self._kill_now

    @kill_now.setter
    def kill_now(self, val):
        if val:
            if self._kill_soon:  # if you kill twice, then it becomes immediate
                self._kill_now = True
            else:
                if self._fade_time > 0.0:
                    self._kill_soon = True
                    self._soft_kill_time = time.time()
                else:
                    self._kill_now = True
        else:
            self._kill_now = False
            self._kill_soon = False
            self._soft_kill_time = None

class SoftRealtimeLoop:
    def __init__(self, dt=0.001, report=False, fade=0.0):
        self.t0 = self.t1 = time.time()
        self.killer = LoopKiller(fade_time=fade)
        self.dt = dt
        self.ttarg = None
        self.sum_err = 0.0
        self.sum_var = 0.0
        self.sleep_t_agg = 0.0
        self.n = 0
        self.report = report

    def __del__(self):
        if self.report:
            print("In %d cycles at %.2f Hz:" % (self.n, 1.0 / self.dt))
            print("\tavg error: %.3f milliseconds" % (1e3 * self.sum_err / self.n))
            print(
                "\tstddev error: %.3f milliseconds"
                % (
                    1e3
                    * sqrt((self.sum_var - self.sum_err**2 / self.n) / (self.n - 1))
                )
            )
            print(
                "\tpercent of time sleeping: %.1f %%"
                % (self.sleep_t_agg / self.time() * 100.0)
            )

    @property
    def fade(self):
        return self.killer.get_fade()

    def run(self, function_in_loop, dt=None):
        if dt is None:
            dt = self.dt
        self.t0 = self.t1 = time.time() + dt
        while not self.killer.kill_now:
            ret = function_in_loop()
            if ret == 0:
                self.stop()
            while time.time() < self.t1 and not self.killer.kill_now:
                if signal.sigtimedwait(
                    [signal.SIGTERM, signal.SIGINT, signal.SIGHUP], 0
                ):
                    self.stop()
            self.t1 += dt
        print("Soft realtime loop has ended successfully.")

    def stop(self):
        self.killer.kill_now = True

    def time(self):
        return time.time() - self.t0

    def time_since(self):
        return time.time() - self.t1

    def __iter__(self):
        self.t0 = self.t1 = time.time() + self.dt
        return self

    def __next__(self):
        if self.killer.kill_now:
            raise StopIteration

        while (
            time.time() < self.t1 - 2 * PRECISION_OF_SLEEP and not self.killer.kill_now
        ):
            t_pre_sleep = time.time()
            time.sleep(
                max(PRECISION_OF_SLEEP, self.t1 - time.time() - PRECISION_OF_SLEEP)
            )
            self.sleep_t_agg += time.time() - t_pre_sleep

        while time.time() < self.t1 and not self.killer.kill_now:
            if signal.sigtimedwait([signal.SIGTERM, signal.SIGINT, signal.SIGHUP], 0):
                self.stop()
        if self.killer.kill_now:
            raise StopIteration
        self.t1 += self.dt
        if self.ttarg is None:
            # inits ttarg on first call
            self.ttarg = time.time() + self.dt
            # then skips the first loop
            return self.t1 - self.t0
        error = time.time() - self.ttarg  # seconds
        self.sum_err += error
        self.sum_var += error**2
        self.n += 1
        self.ttarg += self.dt
        return self.t1 - self.t0

# ---------------------------------------------------------------------------

def register_topics(ser_dev:SerialProtocol):
    # Mo :: Commands, States
    ser_dev.serializer_dict[101] = [lambda bytes: np.frombuffer(bytes, dtype=mo_cmds_dtype), lambda data: data.tobytes()]
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

    stability_controller = {'names': ['SP Body Roll', 'Body Roll', 'SP Body Pitch', 'Body Pitch'],
                    'title': "Stability Controller",
                    'ylabel': "rad",
                    'xlabel': "time",
                    'colors' : ["r", "g", "b", "y"],
                    'line_width': [2]*4,
                    'yrange': [-MAX_TILT, MAX_TILT]
                    }

    plot_config = [imu_states]
    client.initialize_plots(plot_config)

    ser_dev = SerialProtocol()
    register_topics(ser_dev)

    # Init serial
    serial_read_thread = Thread(target = SerialProtocol.read_loop, args=(ser_dev,), daemon=True)
    serial_read_thread.start()

    # Local structs
    commands = np.zeros(1, dtype=mo_cmds_dtype)[0]
    states = np.zeros(1, dtype=mo_states_dtype)[0]

    commands['kill'] = 0.0

    # Time for comms to sync
    time.sleep(1.0)

    # Send the gains 
    # ser_dev.send_topic_data(111, gains)
    ser_dev.send_topic_data(101, commands)

    theta_roll_sp = 0.0
    theta_pitch_sp = 0.0

    theta_roll_pid = PID(THETA_KP, THETA_KI, THETA_KD, theta_roll_sp)
    theta_pitch_pid = PID(THETA_KP, THETA_KI, THETA_KD, theta_pitch_sp)

    theta_roll_pid.output_limits = (-MAX_DUTY, MAX_DUTY)
    theta_pitch_pid.output_limits = (-MAX_DUTY, MAX_DUTY)
    
    dots = init_lights(MAX_BRIGHTNESS)

    for t in SoftRealtimeLoop(dt=DT, report=True):
        try:
            states = ser_dev.get_cur_topic_data(121)[0]
        except KeyError as e:
            print("<< CALIBRATING >>")
            dots.fill(color=(255, 191, 0))
            dots.show()
            continue

        Tx = theta_roll_pid(states['theta_roll'])
        Ty = theta_pitch_pid(states['theta_pitch'])
        Tz = 0.0

        # Motor 1-3's positive direction is flipped hence the negative sign

        commands['motor_1_duty'] = (-0.3333) * (Tz - (2.8284 * Ty))
        commands['motor_2_duty'] = (-0.3333) * (Tz + (1.4142 * (Ty + 1.7320 * Tx))) 
        commands['motor_3_duty'] = (-0.3333) * (Tz + (1.4142 * (Ty - 1.7320 * Tx)))

        ser_dev.send_topic_data(101, commands)

        data = [states['theta_roll'], states['theta_pitch']]
        client.send_array(data)

        if np.abs(states['theta_roll']) != 0.0:
            danger = compute_dots(states['theta_roll'], states['theta_pitch'])

        for dot in range(N_DOTS):
            if dot in danger:
                    dots[dot] = (255, 20, 20)
            else:
                    dots[dot] = (53, 118, 174)
        dots.show()

    print("Resetting Mo commands.")
    commands['kill'] = 1.0
    commands['motor_1_duty'] = 0.0
    commands['motor_2_duty'] = 0.0
    commands['motor_3_duty'] = 0.0
    ser_dev.send_topic_data(101, commands)

    dots.fill(color=(0, 0, 0))
    dots.show()
