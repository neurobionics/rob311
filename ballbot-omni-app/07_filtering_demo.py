import time
import numpy as np
from threading import Thread
from MBot.Messages.message_defs import mo_states_dtype, mo_cmds_dtype
from MBot.SerialProtocol.protocol import SerialProtocol
from collections import deque
from DataLogger import dataLogger

import FIR as fir

"""
ROB 311 - Ball-bot filtering demo

This program uses a soft realtime loop to enforce loop timing. Soft real time loop is a  class
designed to allow clean exits from infinite loops with the potential for post-loop cleanup operations executing.

The Loop Killer object watches for the key shutdown signals on the UNIX operating system (which runs on the PI)
when it detects a shutdown signal, it sets a flag, which is used by the Soft Realtime Loop to stop iterating.
Typically, it detects the CTRL-C from your keyboard, which sends a SIGTERM signal.

the function_in_loop argument to the Soft Realtime Loop's blocking_loop method is the function to be run every loop.
A typical usage would set function_in_loop to be a method of an object, so that the object could store program state.
See the 'ifmain' for two examples.

Authors: Senthur Raj, Gray Thomas, Yves Nazon and Elliott Rouse 
Neurobionics Lab / Locomotor Control Lab
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
# BALL-BOT PARAMETERS

FREQ = 200
DT = 1/FREQ

RW = 0.048
RK = 0.1210
ALPHA = np.deg2rad(45)

# ---------------------------------------------------------------------------
# LOWPASS FILTER PARAMETERS

Fs = FREQ # Sampling rate in Hz
Fc = 1.0 # Cut-off frequency of the filter in Hz

Fn = Fc/Fs # Normalized equivalent of Fc
N = 60 # Taps of the filter

# ---------------------------------------------------------------------------
# CREATING TWO OBJECTS OF THE FIR() CLASS FOR X AND Y AXES

lowpass_filter_dphi_x = fir.FIR()
lowpass_filter_dphi_x.lowpass(N, Fn)

lowpass_filter_dphi_y = fir.FIR()
lowpass_filter_dphi_y.lowpass(N, Fn)

# ---------------------------------------------------------------------------
# WEIGHTED AVERAGE FILTER PARAMETERS

WMA_WEIGHTS = np.array([0.1, 0.3, 0.4, 0.8])

WMA_WINDOW_SIZE = len(WMA_WEIGHTS)
WMA_NORM = WMA_WEIGHTS/np.sum(WMA_WEIGHTS)

# ---------------------------------------------------------------------------

def wma_filter(wma_window):
    return np.sum(WMA_NORM * wma_window)

# ---------------------------------------------------------------------------

def register_topics(ser_dev:SerialProtocol):
    # Mo :: Commands, States
    ser_dev.serializer_dict[101] = [lambda bytes: np.frombuffer(bytes, dtype=mo_cmds_dtype), lambda data: data.tobytes()]
    ser_dev.serializer_dict[121] = [lambda bytes: np.frombuffer(bytes, dtype=mo_states_dtype), lambda data: data.tobytes()]

def transform_w2b(m1, m2, m3):
    """
    Returns Phi attributes
    """

    x = 0.323899 * m2 - 0.323899 * m3
    y = -0.374007 * m1 + 0.187003 * m2 + 0.187003 * m3
    z = 0.187003 * m1 + 0.187003 * m2 + 0.187003 * m3

    return x, y, z

if __name__ == "__main__":

    trial_num = int(input('Trial Number? '))
    filename = 'ROB311_Filtering_Demo_%i' % trial_num
    dl = dataLogger(filename + '.txt')

    t_start = 0.0

    ser_dev = SerialProtocol()
    register_topics(ser_dev)

    # Init serial
    serial_read_thread = Thread(target = SerialProtocol.read_loop, args=(ser_dev,), daemon=True)
    serial_read_thread.start()

    # Local structs
    commands = np.zeros(1, dtype=mo_cmds_dtype)[0]
    states = np.zeros(1, dtype=mo_states_dtype)[0]

    commands['kill'] = 0.0
    zeroed = False

    psi = np.zeros((3, 1))
    psi_offset = np.zeros((3, 1))

    phi = np.zeros((3, 1))
    dpsi = np.zeros((3, 1))
    dphi = np.zeros((3, 1))

    # ---------------------------------------------------------------------------
    # WMA FILTER VARIABLES

    theta_x_window = deque(maxlen=WMA_WINDOW_SIZE) # A sliding window of values
    theta_y_window = deque(maxlen=WMA_WINDOW_SIZE) # A sliding window of values

    for _ in range(WMA_WINDOW_SIZE):
        theta_x_window.append(0.0)
        theta_y_window.append(0.0)

    theta_x = 0.0 # Variable for the filtered value
    theta_y = 0.0 # Variable for the filtered value

    dphi_x = 0.0 # Variable for the filtered value
    dphi_y = 0.0 # Variable for the filtered value

    # ---------------------------------------------------------------------------

    # Time for comms to sync
    time.sleep(1.0)

    # Send the gains
    ser_dev.send_topic_data(101, commands)
    i = 0

    for t in SoftRealtimeLoop(dt=DT, report=True):

        try:
            states = ser_dev.get_cur_topic_data(121)[0]

        except KeyError as e:
            # Calibration: 10 seconds
            print("<< CALIBRATING :: {:.2f} >>".format(t))
            continue

        # Phi and dPhi calculation
        psi[0] = states['psi_1']
        psi[1] = states['psi_2']
        psi[2] = states['psi_3']

        dpsi[0] = states['dpsi_1']
        dpsi[1] = states['dpsi_2']
        dpsi[2] = states['dpsi_3']

        phi[0], phi[1], phi[2] = transform_w2b(psi[0], psi[1], psi[2])
        dphi[0], dphi[1], dphi[2] = transform_w2b(dpsi[0], dpsi[1], dpsi[2])

        # ---------------------------------------------------------------------------
        # Lowpass filtering velocities

        dphi_x = lowpass_filter_dphi_x.filter(dphi[0][0])
        dphi_y = lowpass_filter_dphi_y.filter(dphi[1][0])

        # ---------------------------------------------------------------------------
        # WMA filtering IMU values

        theta_x_window.append(states['theta_roll'])
        theta_y_window.append(states['theta_pitch'])

        theta_x = wma_filter(theta_x_window)
        theta_y = wma_filter(theta_y_window)

        # ---------------------------------------------------------------------------
        # 10 seconds of wait to set the bot on top of the ball

        if t > 11.0 and t < 21.0:
            print("<< PLACE THE BOT ON TOP OF THE BALL :: {:.2f} >>".format(t))
        elif t > 21.0:
            if not zeroed:
                psi_offset = psi
                zeroed = True

        # ---------------------------------------------------------------------------
        # Subtracting the initial values from the motor encoders to start at zero position

        psi = psi - psi_offset

        if zeroed:
            if i == 0:
                t_start = time.time()

            i = i + 1
            t_now = time.time() - t_start

        ser_dev.send_topic_data(101, commands)

        if zeroed:
            print(" << Iteration no: {}, THETA X: {:.2f}, THETA Y: {:.2f} >>".format(i, theta_x, theta_y))

            # Construct the data matrix for saving - you can add more variables by replicating the format below
            data = [i] + [t_now] + \
                [states['theta_roll']] + [states['theta_pitch']] + \
                            [phi[0][0]] + [phi[1][0]] + [phi[2][0]] + \
                                                    [dphi[0][0]] + [dphi[1][0]] + [dphi[2][0]] + \
                                                        [theta_x] + [theta_y] + [dphi_x] + [dphi_y]

            dl.appendData(data)

    print("Resetting Motor commands.")
    time.sleep(0.25)
    commands['motor_1_duty'] = 0.0
    commands['motor_2_duty'] = 0.0
    commands['motor_3_duty'] = 0.0
    time.sleep(0.25)
    commands['kill'] = 1.0
    time.sleep(0.25)
    ser_dev.send_topic_data(101, commands)
    time.sleep(0.25)

    dl.writeOut()
