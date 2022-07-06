import numpy as np
import matplotlib.pyplot as plt
from Functions import Rx, Ry, Rz, quat2rot
import tkinter as tk
from vnpy import *
'''This file is similar to Inv_Kinematics_with_vn.py in that is should have the same structure 
however it will attempt to use and map 2 VN sensors instead of one'''

#plot settings
origin = [0,0,0]
f2_label = ['x_2: Mass', 'y_2', 'z_2'] # the body labels of the pendulum
vn1_label = ['x_vn1', 'y_vn1', 'z_vn1']
vn2_label = ['x_vn2', 'y_vn2', 'z_vn2']
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
    theta_0 = 0.0 # pipe yawing
    theta_1 = 0.0 # drive angle
    theta_2 = 0.0 # steer angle
    elevation = 0.0 # graph view options
    azimuth = 0.0
    status = True
    pipe_angle_z = 0.0
    pipe_angle_y = 0.0
    def update_th1(self, x):
        self.theta_1 = float(x)
        print(f'Drive was updated to {self.theta_1*180/np.pi}')
    def update_th2(self, x):
        self.theta_2 = float(x)*np.pi/180
        print(f'Steering was updated to {self.theta_2*180/np.pi}')
    def update_th0(self, x):
        self.theta_0 = float(x)
        print(f'Pipe Yaw was updated to {self.theta_0*180/np.pi}')
    def update_elev(self, x):
        self.elevation = float(x)
        print(f'Elevation was updated to {self.elevation}')
    def update_azi(self, x):
        self.azimuth = float(x)
        print(f'Azimuth was updated to {self.azimuth}')
    def stop_status(self):
        self.status = False
    def update_pipe_z(self, x):
        self.pipe_angle_z = round(x*180/np.pi, 2)
    def update_pipe_y(self, x):
        self.pipe_angle_y = round(x*180/np.pi, 2)
    def get_message(self):
        print(f"Pipe_z_angle: {self.pipe_angle_z} \nPipe_y_angle: {self.pipe_angle_y}\nDrive Angle: {self.theta_1*180/np.pi}")
angles = Angles()

# initialize the first vn sensor
vn1 = VnSensor()
vn1.connect('COM5', 921600) # COM# will change with computer. This one works from the from of my Anker docking station
vn2 = VnSensor()
vn2.connect('COM8', 921600)

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
f2_pose = np.zeros((3,3))
while angles.status:
    # read in the viewing orientation from the trackbars
    sliderWindow.update_idletasks()
    sliderWindow.update()
    # read in the pendulum orientation from the vn
    data1 = vn1.read_quaternion_magnetic_acceleration_and_angular_rates()
    data2 = vn2.read_quaternion_magnetic_acceleration_and_angular_rates()
    vn1_pose = quat2rot(data1.quat.w , np.array([data1.quat.x, data1.quat.y, data1.quat.z])) # converts vn data to rot frame aka desired pose
    vn2_pose = quat2rot(data2.quat.w , np.array([data2.quat.x, data2.quat.y, data2.quat.z]))

    # average the direction vectors of the sensors, this will be f2
    f2_pose[:,0] = (vn1_pose[:,0] + vn2_pose[:, 0])*0.5 # the y and z components are opposed
    f2_pose[:,1] = (vn1_pose[:,1] - vn2_pose[:, 1])*0.5
    f2_pose[:,2] = (vn1_pose[:,2] - vn2_pose[:, 2])*0.5

    # calculate the frame positions
    # undo steering to reference f1
    f1_pose = f2_pose @ Rz(-angles.theta_2)

    # find angle between f1 and down, this is drive angle (related to torque)
    angles.update_th1(round(np.arccos(np.dot(f1_pose[:,0], np.array([0,0,1]))/ (np.linalg.norm(f1_pose[:,0]))), 2))
    f0_pose = f1_pose @ Rx(-np.pi/2) @ Rz(-angles.theta_1)

    # find angle between pipe (z_f0) and horizontal, this should be related to turning angle
    angles.update_pipe_z(np.arccos(np.dot(f0_pose[:, 2], np.array([0, 0, 1])) / (np.linalg.norm(f0_pose[:, 2]))))
    #angles.update_pipe_y(np.arccos(np.dot(f0_pose[:, 2], np.array([0, 1, 0])) / (np.linalg.norm(f0_pose[:, 2]))))
    # fb_pose = f0_pose @ Ry(np.pi/2) @ Rz(angles.theta_0)

    fig.clear()
    ax = plt.axes(projection="3d")
    ax.set_xlim([-1.5, 1.5])
    ax.set_ylim([-1.5, 1.5])
    ax.set_zlim([-1.5, 1.5])
    ax.set_ylabel('East')
    ax.set_zlabel('Down')
    ax.set_xlabel('North')
    ax.view_init(angles.elevation, angles.azimuth)
    # for i in range(3):
    #     vector = vn1_pose[:, i]
    #     ax.quiver(origin[0], origin[1], origin[2], vector[0], vector[1], vector[2], color='m')
    #     ax.text(vector[0], vector[1], vector[2], vn1_label[i], color='m')
    # for i in range(3):
    #     vector = vn2_pose[:, i]
    #     ax.quiver(origin[0], origin[1], origin[2], vector[0], vector[1], vector[2], color='c')
    #     ax.text(vector[0], vector[1], vector[2], vn2_label[i], color='c')
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
    plt.title(f"Pipe_z_angle: {angles.pipe_angle_z} \nPipe_y_angle: {angles.pipe_angle_y}\nDrive Angle: {round(angles.theta_1*180/np.pi,2)}")
    plt.show(block=False)