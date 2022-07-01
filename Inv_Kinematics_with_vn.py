import numpy as np
import matplotlib.pyplot as plt
from Functions import Rx, Ry, Rz, quat2rot
import tkinter as tk
from vnpy import *

#plot settings
origin = [0,0,0]
f2_label = ['x_2: Mass', 'y_2', 'z_2']
f1_label = ['x_1', 'y_1', 'z_1: Lamprey']
f0_label = ['x_0', 'y_0', 'z_0: Pipe']
fb_label = ['North', 'East', 'Down']
fig = plt.figure()

# set up a slider window
sliderWindow = tk.Tk()
sliderWindow.geometry('300x300')
sliderWindow.title('Encoder Value Slider')
elev_value = tk.DoubleVar()
azim_value = tk.DoubleVar()
encoder_value = tk.DoubleVar()

# create class to update anf track angles
class Angles:
    theta_0 = 0.0
    theta_1 = 0.0
    theta_2 = 0.0
    elevation = 0.0
    azimuth = 0.0
    status = True
    def update_th1(self, x):
        self.theta_1 = float(x)
        print(f'Drive was updated to {self.theta_1*180/np.pi}')
    def update_th2(self, x):
        self.theta_2 = float(x)*np.pi/180
        print(f'Steering was updated to {self.theta_2*180/np.pi}')
    def update_th0(self, x):
        self.theta_0 = float(x)
        print(f'Yaw was updated to {self.theta_0*180/np.pi}')
    def update_elev(self, x):
        self.elevation = float(x)
        print(f'Elevation was updated to {self.elevation}')
    def update_azi(self, x):
        self.azimuth = float(x)
        print(f'Azimuth was updated to {self.azimuth}')
    def stop_status(self):
        self.status = False
angles = Angles()

# initialize the vn sensor
vn = VnSensor()
vn.connect('COM5', 921600) # COM# will change with computer. This one works from the from of my Anker docking station

# Trackbars
val = tk.Scale(sliderWindow, from_=-179, to=-91, variable=elev_value, orient='horizontal', label='Set Elevation Angle (deg)', length=300, command=angles.update_elev)
val.set(-160) # initialize slider value
val.pack()
val = tk.Scale(sliderWindow, from_=-180, to=180, variable=azim_value, orient='horizontal', label='Set Azimuth Angle (deg)', length=300, command=angles.update_azi)
val.pack()
val = tk.Scale(sliderWindow, from_=-60, to=60, variable=encoder_value, orient='horizontal', label='Set Steer Angle (deg)', length=300, command=angles.update_th2)
val.pack()
B = tk.Button(sliderWindow, text="Stop Program", command=angles.stop_status)
B.pack()
# loop the animation
while angles.status:
    # read in the viewing orientation from the trackbars
    sliderWindow.update_idletasks()
    sliderWindow.update()
    # read in the pendulum orientation from the vn
    data = vn.read_quaternion_magnetic_acceleration_and_angular_rates()
    f2_pose = quat2rot(data.quat.w , np.array([data.quat.x, data.quat.y, data.quat.z])) # converts vn data to rot frame aka desired pose


    # calculate the frame positions
    # undo steering to reference f1
    f1_pose = f2_pose @ Rz(-angles.theta_2)

    # find angle between f1 and down, this is drive angle
    angles.update_th1(np.arccos(np.dot(f1_pose[:,0], np.array([0,0,1]))/ (np.linalg.norm(f1_pose[:,0]))))
    f0_pose = f1_pose @ Rx(-np.pi/2) @ Rz(-angles.theta_1)
    #fb_pose = f0_pose @ Ry(np.pi/2) @ Rz(angles.theta_0)

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
    #for i in range(3):
    #     vector = fb_pose[:, i]
    #     ax.quiver(origin[0], origin[1], origin[2], vector[0], vector[1], vector[2], color = 'k')
    #     ax.text(vector[0], vector[1], vector[2], fb_label[i], color='k')
    plt.show(block=False)