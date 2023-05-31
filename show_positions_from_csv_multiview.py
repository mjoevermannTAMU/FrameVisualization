import math

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import *
import matplotlib
import csv
import time

plt.ion() #suposedly interactive mode

data = open('scary_data_good_part.csv')
type(data)
csvreader = csv.reader(data)

data_rows = []
for row in csvreader:
    data_rows.append(row)

row_count = 1  # row you're on
frame_interval = 20  # runs every 50th frame

times, drive_angles, pipe_angles = [], [], []

for row in data_rows:
    if (row_count > 1 and row_count % frame_interval == 0):
        times.append(row[0])
        drive_angles.append(row[1])
        pipe_angles.append(row[2])
    row_count += 1

class EasyBox:
    def __init__(self, center,rotation_center, width, height, depth, axs, color = (0.3,0.3,1,1)):
        self.center = np.array(center)
        self.width = width
        self.height = height
        self.depth = depth
        self.axs = axs
        self.x_rotation = 0
        self.y_rotation = 0
        self.z_rotation = 0
        self.color = color
        self.rotation_center = np.array(rotation_center)

    def plot(self):
        half_width = self.width / 2
        half_height = self.height / 2
        half_depth = self.depth / 2

        box_x = np.array([-1, 1, 1, -1, -1, 1, 1, -1]) * half_width + self.center[0]
        box_y = np.array([1, 1, -1, -1, 1, 1, -1, -1]) * half_height + self.center[1]
        box_z = np.array([1, 1, 1, 1, -1, -1, -1, -1]) * half_depth + self.center[2]

        self.x_rotation = float(self.x_rotation)
        self.y_rotation = float(self.y_rotation)
        self.z_rotation = float(self.z_rotation)

        # Apply rotations
        rotation_matrix_x = np.array([[1, 0, 0],
                                      [0, np.cos(np.radians(self.x_rotation)), -np.sin(np.radians(self.x_rotation))],
                                      [0, np.sin(np.radians(self.x_rotation)), np.cos(np.radians(self.x_rotation))]])
        rotation_matrix_y = np.array([[np.cos(np.radians(self.y_rotation)), 0, np.sin(np.radians(self.y_rotation))],
                                      [0, 1, 0],
                                      [-np.sin(np.radians(self.y_rotation)), 0, np.cos(np.radians(self.y_rotation))]])
        rotation_matrix_z = np.array([[np.cos(np.radians(self.z_rotation)), -np.sin(np.radians(self.z_rotation)), 0],
                                      [np.sin(np.radians(self.z_rotation)), np.cos(np.radians(self.z_rotation)), 0],
                                      [0, 0, 1]])

        #update to rotations
        box_x, box_y, box_z = np.dot(np.array([box_x, box_y, box_z]).T - np.array(self.rotation_center), rotation_matrix_x).T
        box_x, box_y, box_z = np.dot(np.array([box_x, box_y, box_z]).T, rotation_matrix_y).T
        box_x, box_y, box_z = np.dot(np.array([box_x, box_y, box_z]).T, rotation_matrix_z).T

        box_x += self.center[0]
        box_y += self.center[1]
        box_z += self.center[2]

        # Update to rotations
        # rotated_xyz = np.dot(np.array([box_x, box_y, box_z]).T - self.rotation_center, rotation_matrix_x)
        # rotated_xyz = np.dot(rotated_xyz, rotation_matrix_y)
        # rotated_xyz = np.dot(rotated_xyz, rotation_matrix_z)
        #
        # box_x, box_y, box_z = rotated_xyz.T + self.rotation_center

        # Define the faces of the box
        faces = [
            [0, 1, 3, 2],  # Bottom face
            [4, 5, 7, 6],
            [0, 1, 4, 5],  # Front face
            [1, 2, 5, 6],  # Right face
            [2, 3, 6, 7],  # Back face
            [3, 0, 7, 4]]  # Left face"""


        # Plot the surfaces of the box
        #for face in faces:
            #self.ax.plot_surface(box_x[face], box_y[face], box_z[face], color='r')

        # Define the RGB color
        color_rgb = self.color # RGB values normalized to [0, 1]

        # Plot the surfaces of the box with the specified RGB color
        for face in faces:
            x = box_x[face]
            y = box_y[face]
            z = box_z[face]

            vertices = np.column_stack([x, y, z])

            for ax in self.axs:
                ax.plot_surface(
                    x.reshape((2, 2)),
                    y.reshape((2, 2)),
                    z.reshape((2, 2)),
                    color=color_rgb,
                    edgecolors="k"
                )
                max_dim = np.max([self.width, self.height, self.depth])
                ax.set_xlim(self.center[0] - max_dim, self.center[0] + max_dim)
                ax.set_ylim(self.center[1] - max_dim, self.center[1] + max_dim)
                ax.set_zlim(self.center[2] - max_dim, self.center[2] + max_dim)
                ax.set_aspect('equal')



class EasySphere:
    def __init__(self, center, radius, axs):
        self.center = np.array(center)
        self.radius = radius
        self.axs = axs
        self.y_rotation = 90
        self.x_rotation = 0
        self.z_rotation = 0

    def plot(self, x_rotation=0, y_rotation=0, z_rotation=0):
        u = np.linspace(np.pi/2.0, 2.5 * np.pi, 30)
        v = np.linspace(0, np.pi, 30)

        x = self.center[0] + self.radius * np.outer(np.cos(u), np.sin(v))
        y = self.center[1] + self.radius * np.outer(np.sin(u), np.sin(v))
        z = self.center[2] + self.radius * np.outer(np.ones(np.size(u)), np.cos(v))

        # Apply rotations
        rotation_matrix_x = np.array([[1, 0, 0],
                                      [0, np.cos(np.radians(self.x_rotation)), -np.sin(np.radians(self.x_rotation))],
                                      [0, np.sin(np.radians(self.x_rotation)), np.cos(np.radians(self.x_rotation))]])
        rotation_matrix_y = np.array([[np.cos(np.radians(self.y_rotation)), 0, np.sin(np.radians(self.y_rotation))],
                                      [0, 1, 0],
                                      [-np.sin(np.radians(self.y_rotation)), 0, np.cos(np.radians(self.y_rotation))]])
        rotation_matrix_z = np.array([[np.cos(np.radians(self.z_rotation)), -np.sin(np.radians(self.z_rotation)), 0],
                                      [np.sin(np.radians(self.z_rotation)), np.cos(np.radians(self.z_rotation)), 0],
                                      [0, 0, 1]])

        x, y, z = np.dot(np.array([x, y, z]).T - [self.center], rotation_matrix_z).T
        x, y, z = np.dot(np.array([x, y, z]).T, rotation_matrix_y).T
        x, y, z = np.dot(np.array([x, y, z]).T, rotation_matrix_x).T

        for ax in self.axs:
            ax.plot_surface(x, y, z, color='none', edgecolor=(0.1,0.1,0.1,0.2))

            max_range = self.radius * 1.5
            ax.set_xlim(self.center[0] - max_range, self.center[0] + max_range)
            ax.set_ylim(self.center[1] - max_range, self.center[1] + max_range)
            ax.set_zlim(self.center[2] - max_range, self.center[2] + max_range)
            ax.set_aspect('equal')

class EasyDrawer:
    def __init__(self, figsize=(8,6)):
        self.fig_number = 4
        self.figs = []
        self.axs = []
        names = ["orthogonal view", "top view", "front view", "side view"]
        for i in range(self.fig_number):
            self.figs.append(plt.figure(names[i], figsize=figsize))
            self.axs.append(self.figs[i].add_subplot(111, projection='3d'))
        self.boxes = []
        self.spheres = []
        self.sphere_rotation_angle = 0

    def add_box(self, center,rotation_center, width, height, depth, color = (0.3,0.3,1,1)):
        new_box = EasyBox(center=center,rotation_center=rotation_center, width=width, height=height, depth=depth, axs=self.axs, color=color)
        self.boxes.append(new_box)
        return new_box

    def add_sphere(self, center, radius):
        new_sphere = EasySphere(center=center, radius=radius, axs=self.axs)
        self.spheres.append(new_sphere)
        return new_sphere

    def update_objects(self):
        for box in self.boxes:
            box.plot()
        for sphere in self.spheres:
            sphere.plot(sphere.x_rotation, sphere.y_rotation, sphere.z_rotation)

    def show_objects(self):
        for i in range(self.fig_number):
            current_ax = self.axs[i]
            current_ax.cla()  # Clear the previous plot

        self.update_objects()

        for i in range(self.fig_number):
            current_ax = self.axs[i]

            current_ax.set_xlabel('X')
            current_ax.set_ylabel('Y')
            current_ax.set_zlabel('Z')

            current_ax.set_xlim(-12, 12)
            current_ax.set_ylim(-12, 12)
            current_ax.set_zlim(-12, 12)

            drive_angle = -round(float(x_angle),2)
            drive_text = "Drive Angle: " + str(drive_angle)
            pipe_angle = round(float(z_angle - 90), 2)
            pipe_text = "Pipe Angle: " + str(pipe_angle)
            current_ax.text2D(x=-0.1, y=-0.11, s=drive_text, fontsize = 10)
            current_ax.text2D(x=0.03, y=-0.11, s=pipe_text, fontsize=10)

            text_time = "time (s?): " + str(round(float(time), 2))
            current_ax.text2D(x=-0.1, y=0.1, s=text_time, fontsize=10)

            update_rotation(0)

        self.axs[0].view_init(elev=30, azim=130)
        self.axs[1].view_init(elev=90, azim=90)
        self.axs[2].view_init(elev=0, azim=90)
        self.axs[3].view_init(elev=0, azim=180)

        for i in range(self.fig_number):
            self.figs[i].show()

        plt.pause(pause_time)


def time_to_seconds(time_str):
    print("time:", time_str)
    seconds, milliseconds = time_str.split(':')
    total_seconds = float(seconds) * 60 + float(milliseconds)
    return total_seconds

# Create a function to update the sphere's y_rotation based on the slider value
def update_rotation(val):
    for box in model.boxes:
        box.x_rotation = x_angle

    pendulum2.z_rotation = (float(z_angle)-90) * -math.cos(math.radians(float(x_angle)-90))
    pendulum2.y_rotation = (float(z_angle)-90) * -math.sin(math.radians(float(x_angle)-90))

# Create an instance of EasyDrawer
model = EasyDrawer(figsize=(4, 4))

#model.fig.show()
#plt.ion()

sphere = model.add_sphere(center=[0, 0, 12], radius=12)
pendulum1 = model.add_box(center=[0, 0, 0], rotation_center=[0,0,1.5], width=2, height=2, depth=2, color=(1,0.3,0.3,0.5))
pendulum2 = model.add_box(center=[0, 0, 0], rotation_center=[0,0,5], width=2, height=2, depth=6, color=(1,0.3,0.3,0.5))
rod = model.add_box(center=[0, 0, 0], rotation_center=[0,0,0], width=24, height=1, depth=1,color=(0.5,0.5,1,0.3))
rod.x_rotation = 90

x_angle, y_angle, z_angle = 90.0, 90.0, 90.0

time = 0
first_time = time_to_seconds(times[0])
pause_time = 0.000000001

for frame in range(len(times)):
    #model.fig.clear()
    # Update the slider value to the current angle
    x_angle = float(math.degrees(float(drive_angles[frame])))
    z_angle = float(math.degrees(float(pipe_angles[frame]))) + 90
    time = time_to_seconds(times[frame]) - first_time

    model.show_objects()