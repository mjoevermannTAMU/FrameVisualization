import numpy as np
import matplotlib.pyplot as plt
from Functions import Rx, Ry, Rz, quat2rot
import tkinter as tk
from vnpy import *
import time

# TODO figure out how to update plot without resetting the view
#plot settings
origin = [0,0,0]
f2_label = ['x_2: mass', 'y_2', 'z_2']
f1_label = ['x_1', 'y_1', 'z_1: Pipe']
f0_label = ['x_0: down', 'y_0', 'z_0']
fig = plt.figure()

# todo how an I going to find th_1?

# set up a slider window
# sliderWindow = tk.Tk()
# sliderWindow.geometry('300x140')
# sliderWindow.title('Encoder Value Slider')
# encoder_value = tk.DoubleVar()
# tip_value = tk.DoubleVar()

class Angles:
    theta_0 = 0.0
    theta_1 = 0.0
    theta_2 = 0.0
    def update_th1(self, x):
        self.theta_1 = float(x)
        print(f'Theta 1 was updated to {self.theta_1*180/np.pi}')
    def update_th2(self, x):
        self.theta_2 = float(x)
        print(f'Theta 2 was updated to {self.theta_2*180/np.pi}')
    def update_th0(self, x):
        self.theta_0 = float(x)
        print(f'Theta 0 was updated to {self.theta_0*180/np.pi}')

angles = Angles()
vn = VnSensor()
vn.connect('COM5', 921600) # comm port may change on each machine

# val = tk.Scale(sliderWindow, from_=-60, to=60, variable=encoder_value, orient='horizontal', label='Set Steer Angle (deg)', length=300, command=angles.update_th2)
# val.pack()
# val = tk.Scale(sliderWindow, from_=-90, to=90, variable=tip_value, orient='horizontal', label='Set Drive Angle (deg)', length=300, command=angles.update_th1)
# val.pack()

# loop the animation
ax = plt.axes(projection="3d")
ax.set_xlim([-1.5, 1.5])
ax.set_ylim([-1.5, 1.5])
ax.set_zlim([-1.5, 1.5])
ax.set_ylabel('East')
ax.set_zlabel('Down')
ax.set_xlabel('North')
ax.view_init(-120, -2)
while True:
    # read in the pendulum orientation from the VN
    data = vn.read_quaternion_magnetic_acceleration_and_angular_rates()
    s0 = data.quat.w
    v0 = np.array([data.quat.x, data.quat.y, data.quat.z])
    R_init = quat2rot(s0, v0)
    # sliderWindow.update_idletasks()
    # sliderWindow.update()
    # calculate th1
    angles.update_th1(np.arccos(R_init[2,2]))
    angles.update_th0(np.arccos(R_init[0,2]/np.sin(angles.theta_1)))
    angles.update_th2(np.arcsin(R_init[2,0] / np.sin(angles.theta_1)))
    # calculate the orientation of the pitch frame, f1
    f1_pose = Rz(angles.theta_0) @ Rx(-np.pi/2) @ Rz(angles.theta_1) @ Rx(np.pi/2)
    # calculate the orientation of the pitch frame, f0
    f0_pose = Rz(angles.theta_0) @ Rx(-np.pi/2)
    f2_pose = Rz(angles.theta_0) @ Rx(-np.pi/2) @ Rz(angles.theta_1) @ Rx(np.pi/2) @ Rz(angles.theta_2)


    for i in range(3):
        vector = f2_pose[:, i]
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
    plt.show(block=False)
    time.sleep(1)





