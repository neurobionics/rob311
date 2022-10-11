#!/usr/bin/python3
from itertools import count
import board
import time
import numpy as np
import adafruit_dotstar as dotstar

# TEST

FREQ = 200
DT = 1/FREQ

N_DOTS = 72
MAX_BRIGHTNESS = 0.055
MIN_BRIGHTNESS = 0.01

MAX_TILT = np.deg2rad(10)
ARC_START = np.deg2rad(15)
ARC_STOP = 2*np.pi - np.deg2rad(15)

ARC = ARC_STOP - ARC_START
ARC_PER_DOT = ARC/N_DOTS

INC = 0.01

t = time.time()

points = [20, 21, 22, 23, 24, 25, 26, 27]

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

        print(x, y, slope, dot_position, dot_intensity)

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

if __name__ == '__main__':

        dots = init_lights(MAX_BRIGHTNESS)
        ct = 0.01

        roll = -0.01
        pitch = -0.01

        danger = compute_dots(roll, pitch)

        while True:
                for dot in range(N_DOTS):
                        if dot in danger:
                                dots[dot] = (255, 20, 20)
                        else:
                                dots[dot] = (53, 118, 174)

                        time.sleep(DT)

                dots.fill(color=(0, 0, 0))
                dots.show()                
