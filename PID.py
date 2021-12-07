"""
PID controller

"""

# Parameters definition

import numpy as np
import math
import matplotlib.pyplot as plt
from robot import Robot

# Parameters
k = 0.1  # look forward gain
Lfc = 2.0  # [m] look-ahead distance
Kp = 1.0  # speed proportional gain
dt = 0.1  # [s] time tick
WB = 2.9  # [m] wheel base of vehicle

show_animation = True

def proportional_control(target, current):
    a = Kp * (target - current)

    return a

