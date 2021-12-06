"""
Move to specified position
"""

import matplotlib.pyplot as plt
import numpy as np
from random import random

# simulation parameters 
Kp_dist = 9
Kp_alpha = 17
dt = 0.01

show_animation = True
def move_to_position(x_robot, y_robot, angle_robot, x_goal, y_goal):
    """
    dist is the distance between the robot and the goal position
    alpha is the angle to the goal respectively to the angle of the robot
    beta is the angle between the robot's position and the goal position + goal angle
    
    Kp_dist * dist and Kp_alpha * alpha drive the robot along a line towards the goal
   

    """
    x = x_robot
    y = y_robot
    theta = angle_robot

    x_diff = x_goal - x
    y_diff = y_goal - y

    x_traj, y_traj = [], []

    dist = np.hypot(x_diff, y_diff)

    while dist > 0.001 :
        x_traj.append(x)
        y_traj.append(y)

        # update the distance 
        x_diff = x_goal - x
        y_diff = y_goal - y
        dist = np.hypot(x_diff, y_diff)

        # definition of angle alpha
        alpha = ( np.arctan2(y_diff, x_diff) - theta +np.pi) % (2*np.pi) - np.pi

        v = Kp_dist * dist
        w = Kp_alpha * alpha

        theta = theta + w * dt
        x = x + v * np.cos(theta) * dt
        y = y + v * np.sin(theta) * dt

        if show_animation:  # pragma: no cover
            plt.cla()
            plt.arrow(x_robot, y_robot, np.cos(angle_robot),
                      np.sin(angle_robot), color='r', width=0.1)
            plt.plot (x_goal, y_goal,'ro')
            plot_vehicle(x, y, theta, x_traj, y_traj)


def plot_vehicle(x, y, theta, x_traj, y_traj):  # pragma: no cover
    # Corners of triangular vehicle when pointing to the right (0 radians)
    p1_i = np.array([0.5, 0, 1]).T
    p2_i = np.array([-0.5, 0.25, 1]).T
    p3_i = np.array([-0.5, -0.25, 1]).T

    T = transformation_matrix(x, y, theta)
    p1 = np.matmul(T, p1_i)
    p2 = np.matmul(T, p2_i)
    p3 = np.matmul(T, p3_i)

    plt.plot([p1[0], p2[0]], [p1[1], p2[1]], 'k-')
    plt.plot([p2[0], p3[0]], [p2[1], p3[1]], 'k-')
    plt.plot([p3[0], p1[0]], [p3[1], p1[1]], 'k-')

    plt.plot(x_traj, y_traj, 'b--')

    # for stopping simulation with the esc key.
    plt.gcf().canvas.mpl_connect('key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])

    plt.xlim(0, 20)
    plt.ylim(0, 20)

    plt.pause(dt)


def transformation_matrix(x, y, theta):
    return np.array([
        [np.cos(theta), -np.sin(theta), x],
        [np.sin(theta), np.cos(theta), y],
        [0, 0, 1]
    ])


def main():

    for i in range(5):
        x_robot = 20 * random()
        y_robot = 20 * random()
        angle_robot = 2 * np.pi * random() - np.pi
        x_goal = 20 * random()
        y_goal = 20 * random()

        print("Initial x: %.2f m\nInitial y: %.2f m\nInitial theta: %.2f rad\n" %
              (x_robot, y_robot, angle_robot))
        print("Goal x: %.2f m\nGoal y: %.2f m\n" %
              (x_goal, y_goal))
        move_to_position(x_robot, y_robot, angle_robot, x_goal, y_goal)


if __name__ == '__main__':
    main()











    