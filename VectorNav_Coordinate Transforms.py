import numpy as np
import matplotlib.pyplot as plt
from Functions import Rx, Ry, Rz, quat2rot
import tkinter as tk
from vnpy import *

# TODO figure out how to update plot without resetting the view
#plot settings
origin = [0,0,0]
f2_label = ['x_2: mass', 'y_2', 'z_2']
f1_label = ['x_1', 'y_1', 'z_1: Pipe']
f0_label = ['x_0: down', 'y_0', 'z_0']
fig = plt.figure()

# how an I going to find th_1?

# set up a slider window
sliderWindow = tk.Tk()
sliderWindow.geometry('300x140')
sliderWindow.title('Encoder Value Slider')
encoder_value = tk.DoubleVar()
tip_value = tk.DoubleVar()

class Angles:
    theta_1 = 0.0
    theta_2 = 0.0
    def update_th1(self, x):
        self.theta_1 = float(x)* np.pi / 180
        # print(f'Theta 1 has changed to {self.theta_1}')
    def update_th2(self, x):
        self.theta_2 = float(x)* np.pi / 180

angles = Angles()
vn = VnSensor()
vn.connect('COM5', 921600) # comm port may change on each machine

val = tk.Scale(sliderWindow, from_=-60, to=60, variable=encoder_value, orient='horizontal', label='Set Steer Angle (deg)', length=300, command=angles.update_th2)
val.pack()
val = tk.Scale(sliderWindow, from_=-90, to=90, variable=tip_value, orient='horizontal', label='Set Drive Angle (deg)', length=300, command=angles.update_th1)
val.pack()

# loop the animation
prev1 = None
prev2 = None
while True:
    # read in the pendulum orientation from the VN
    data = vn.read_quaternion_magnetic_acceleration_and_angular_rates()
    s0 = data.quat.w
    v0 = np.array([data.quat.x, data.quat.y, data.quat.z])
    R_init = quat2rot(s0, v0)
    sliderWindow.update_idletasks()
    sliderWindow.update()

    # calculate the orientation of the pitch frame, f1
    f1_pose = R_init @ Rz(-angles.theta_2) @ Rx(np.pi / 2)
    # calculate the orientation of the pitch frame, f0
    f0_pose = f1_pose @ Rz(angles.theta_1)

    fig.clear()
    ax = plt.axes(projection="3d")
    ax.set_xlim([-1.5, 1.5])
    ax.set_ylim([-1.5, 1.5])
    ax.set_zlim([-1.5, 1.5])
    ax.set_ylabel('East')
    ax.set_zlabel('Down')
    ax.set_xlabel('North')
    ax.view_init(-120, -2)
    for i in range(3):
        vector = R_init[:, i]
        ax.quiver(origin[0], origin[1], origin[2], vector[0], vector[1], vector[2], color='b')
        ax.text(vector[0], vector[1], vector[2], f2_label[i], color='b')
    for i in range(3):
        vector = f1_pose[:, i]
        ax.quiver(origin[0], origin[1], origin[2], vector[0], vector[1], vector[2], color='g')
        ax.text(vector[0], vector[1], vector[2], f1_label[i], color='g')
    for i in range(3):
        vector = f0_pose[:, i]
        ax.quiver(origin[0], origin[1], origin[2], vector[0], vector[1], vector[2], color = 'r')
        ax.text(vector[0], vector[1], vector[2], f0_label[i], color='r')
    prev1 = angles.theta_1
    prev2 = angles.theta_2
    plt.show(block=False)





