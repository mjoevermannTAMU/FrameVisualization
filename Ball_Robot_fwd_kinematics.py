import numpy as np
import matplotlib.pyplot as plt
from Functions import Rx, Ry, Rz, quat2rot
import tkinter as tk
import csv
import time

data = open('scary_data.csv')
type(data)
csvreader = csv.reader(data)

data_rows = []
for row in csvreader:
    data_rows.append(row)

# plot settings
origin = [0, 0, 0]
f2_label = ['x_2: Mass', 'y_2', 'z_2']
f1_label = ['x_1', 'y_1', 'z_1: Lamprey']
f0_label = ['x_0', 'y_0', 'z_0: Pipe']

vector_figure = plt.figure()


class Angles:
    theta_0 = 0.0
    theta_1 = 0.0
    theta_2 = 0.0
    elevation = 0.0
    azimuth = 45.0
    status = True
    cycle = False
    
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
        
    def stop_status(self):
        self.status = False
        
    def start_data_cycle(self):
        self.cycle = True

angles = Angles()

class SliderWindow:
    sliderWindow = tk.Tk()
    sliderWindow.geometry('500x700')
    sliderWindow.title('Angle Sliders')
    encoder_value = tk.DoubleVar()
    tip_value = tk.DoubleVar()
    yaw_value = tk.DoubleVar()
    elev_value = tk.DoubleVar()
    azim_value = tk.DoubleVar()

    val = tk.Scale(sliderWindow, from_=-179, to=-91, variable=elev_value, orient='horizontal',
                   label='Set Elevation Angle (deg)', length=300, command=angles.update_elev)
    val.set(-160)  # initialize slider value
    val.pack()
    val = tk.Scale(sliderWindow, from_=-180, to=180, variable=azim_value, orient='horizontal',
                   label='Set Azimuth Angle (deg)', length=300, command=angles.update_azi)
    val.pack()
    val = tk.Scale(sliderWindow, from_=-90, to=90, variable=yaw_value, orient='horizontal', label='Set Yaw Angle (deg)',
                   length=300, command=angles.update_th0)
    val.pack()
    val = tk.Scale(sliderWindow, from_=-90, to=90, variable=tip_value, orient='horizontal',
                   label='Set Drive Angle (deg)', length=300, command=angles.update_th1)
    val.pack()
    val = tk.Scale(sliderWindow, from_=-60, to=60, variable=encoder_value, orient='horizontal',
                   label='Set Steer Angle (deg)', length=300, command=angles.update_th2)
    val.pack()
    B = tk.Button(sliderWindow, text="Stop Program", command=angles.stop_status)
    B.pack()
    start = tk.Button(sliderWindow, text="Start data cycle", command=angles.start_data_cycle)
    start.pack()

    def update_window(self):
        self.sliderWindow.update_idletasks()
        self.sliderWindow.update()

slider_window = SliderWindow()

# loop the animation
row_count = 1
frame_interval = 50

while angles.status:
    # read in the pendulum orientation from the trackbars
    slider_window.update_window()
    if angles.cycle:
        if (len(data_rows) - row_count) >= frame_interval:
            drive_angle = float(data_rows[row_count][1]) * 180/np.pi
            angles.update_th1(drive_angle)

            steer_angle = float(data_rows[row_count][2]) * 180/np.pi
            angles.update_th2(steer_angle)
            row_count += frame_interval

        elif (len(data_rows) - row_count - 1) < frame_interval:
            drive_angle = float(data_rows[row_count][1]) * 180 / np.pi
            angles.update_th1(drive_angle)

            steer_angle = float(data_rows[row_count][2]) * 180 / np.pi
            angles.update_th2(steer_angle)
            row_count += 1
            
        else:
            break

    #update vector figure
    vector_figure.clear()
    ax = vector_figure.add_subplot(111, projection="3d")
    ax.set_title("3D plot")
    ax.set_xlim([-1.5, 1.5])
    ax.set_ylim([-1.5, 1.5])
    ax.set_zlim([-1.5, 1.5])
    ax.set_ylabel('East', color='black')
    ax.set_zlabel('Down')
    ax.set_xlabel('North')
    ax.view_init(angles.elevation, angles.azimuth)

    # calculate the orientation of the frames, f0, f1, f2
    f0_pose = Rz(angles.theta_0) @ Ry(-np.pi / 2)
    f1_pose = f0_pose @ Rz(angles.theta_1) @ Rx(np.pi / 2)
    f2_pose = f1_pose @ Rz(angles.theta_2)

    for i in range(3):
        mass_vector = f2_pose[:, i]
        ax.quiver(origin[0], origin[1], origin[2], mass_vector[0], mass_vector[1], mass_vector[2], color='b')
        ax.text(mass_vector[0], mass_vector[1], mass_vector[2], f2_label[i], color='b')

        lamprey_vector = f1_pose[:, i]
        ax.quiver(origin[0], origin[1], origin[2], lamprey_vector[0], lamprey_vector[1], lamprey_vector[2], color='g')
        ax.text(lamprey_vector[0], lamprey_vector[1], lamprey_vector[2], f1_label[i], color='g')

        pipe_vector = f0_pose[:, i]
        ax.quiver(origin[0], origin[1], origin[2], pipe_vector[0], pipe_vector[1], pipe_vector[2], color='r')
        ax.text(pipe_vector[0], pipe_vector[1], pipe_vector[2], f0_label[i], color='r')

    vector_figure.show()
    # print("current line: " + str(row_count))
