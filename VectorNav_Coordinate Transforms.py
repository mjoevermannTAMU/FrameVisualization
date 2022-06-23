import numpy as np
import matplotlib.pyplot as plt
from Functions import Rx, Ry, Rz, quat2rot
import tkinter as tk

#plot settings
origin = [0,0,0]
f2_label = ['x_2: mass', 'y_2', 'z_2']
f1_label = ['x_1', 'y_1', 'z_1: Lamprey']
f0_label = ['x_0', 'y_0', 'z_0: Pipe']
fig = plt.figure()

# set up a slider window
sliderWindow = tk.Tk()
sliderWindow.geometry('300x300')
sliderWindow.title('Encoder Value Slider')
encoder_value = tk.DoubleVar()
tip_value = tk.DoubleVar()
yaw_value = tk.DoubleVar()
elev_value = tk.DoubleVar()
azim_value = tk.DoubleVar()

class Angles:
    theta_0 = 0.0
    theta_1 = 0.0
    theta_2 = 0.0
    elevation = 0.0
    azimuth = 0.0
    def update_th1(self, x):
        self.theta_1 = float(x)*np.pi/180
        print(f'Theta 1 was updated to {self.theta_1}')
    def update_th2(self, x):
        self.theta_2 = float(x)*np.pi/180
        print(f'Theta 2 was updated to {self.theta_2}')
    def update_th0(self, x):
        self.theta_0 = float(x)*np.pi/180
        print(f'Theta 0 was updated to {self.theta_0}')
    def update_elev(self, x):
        self.elevation = float(x)
        print(f'Theta 0 was updated to {self.elevation}')
    def update_azi(self, x):
        self.azimuth = float(x)
        print(f'Theta 0 was updated to {self.azimuth}')
angles = Angles()

val = tk.Scale(sliderWindow, from_=-179, to=-91, variable=elev_value, orient='horizontal', label='Set Elevation Angle (deg)', length=300, command=angles.update_elev)
val.set(-160) # initialize slider value
val.pack()
val = tk.Scale(sliderWindow, from_=-180, to=180, variable=azim_value, orient='horizontal', label='Set Azimuth Angle (deg)', length=300, command=angles.update_azi)
val.pack()
val = tk.Scale(sliderWindow, from_=-90, to=90, variable=yaw_value, orient='horizontal', label='Set Yaw Angle (deg)',
               length=300, command=angles.update_th0)
val.pack()
val = tk.Scale(sliderWindow, from_=-90, to=90, variable=tip_value, orient='horizontal', label='Set Drive Angle (deg)', length=300, command=angles.update_th1)
val.pack()
val = tk.Scale(sliderWindow, from_=-60, to=60, variable=encoder_value, orient='horizontal', label='Set Steer Angle (deg)', length=300, command=angles.update_th2)
val.pack()

# loop the animation
while True:
    # read in the pendulum orientation from the trackbars
    sliderWindow.update_idletasks()
    sliderWindow.update()

    # calculate the orientation of the frames, f0, f1, f2
    f0_pose = Rz(angles.theta_0) @ Ry(-np.pi/2)
    f1_pose = f0_pose @ Rz(angles.theta_1) @ Rx(np.pi / 2)
    f2_pose = f1_pose @ Rz(angles.theta_2)

    fig.clear()
    ax = plt.axes(projection="3d")
    ax.set_xlim([-1.5, 1.5])
    ax.set_ylim([-1.5, 1.5])
    ax.set_zlim([-1.5, 1.5])
    ax.set_ylabel('East')
    ax.set_zlabel('Down')
    ax.set_xlabel('North')
    ax.view_init(angles.elevation, angles.azimuth)
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





