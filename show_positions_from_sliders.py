import math

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider
import csv
import time

data = open('scary_data.csv')
type(data)
csvreader = csv.reader(data)

data_rows = []
for row in csvreader:
    data_rows.append(row)

row_count = 1  # row you're on
frame_interval = 50  # runs every 50th frame

times, drive_angles, pipe_angles = [], [], []

for row in data_rows:
    if (row_count % frame_interval == 0):
        times.append(row[0])
        drive_angles.append(row[1])
        pipe_angles.append(row[2])
    row_count += 1

class Vector3:
    def __init__(self, x = 0, y = 0, z = 0):
        self.x = x
        self.y = y
        self.z = z
    def __str__(self):
        return f"Coordinates: (x={self.x}, y={self.y}, z={self.z})"

class EasyBox:
    def __init__(self, center,rotation_center, size, rotation=Vector3(0,0,0), ax="none", color = (0.3,0.3,1,1), name = "[unnamed box]"):
        self.center = center
        self.rotation_center = rotation_center
        self.size = size
        self.rotation = rotation
        self.ax = ax
        self.color = color
        self.name = name

    def __str__(self):
        return f"Box Object {self.name}: " \
               f"\n\tvariables" \
               f"\n\t\tcenter: (x={self.center.x}, y={self.center.y}, z={self.center.z})" \
               f"\n\t\trotation center: (x={self.rotation_center.x}, y={self.rotation_center.y}, z={self.rotation_center.z})" \
               f"\n\t\trotation: (x={self.rotation.x}, y={self.rotation.y}, z={self.rotation.z})" \
               f"\n\tfixed" \
               f"\n\t\tsize: (x={self.size.x}, y={self.size.y}, z={self.size.z})" \
               f"\n\t\tcolor: {self.color}" \
               f")"

    def update_position_and_rotation(self):

        #update positions
        half_width = self.size.x / 2
        half_height = self.size.y / 2
        half_depth = self.size.z / 2

        box_x = np.array([-1, 1, 1, -1, -1, 1, 1, -1]) * half_width + np.array(self.center.x)
        box_y = np.array([1, 1, -1, -1, 1, 1, -1, -1]) * half_height + np.array(self.center.y)
        box_z = np.array([1, 1, 1, 1, -1, -1, -1, -1]) * half_depth + np.array(self.center.z)

        #update rotation

        rotation_matrix_x = np.array([[1, 0, 0],
                                      [0, np.cos(np.radians(self.rotation.x)), -np.sin(np.radians(self.rotation.x))],
                                      [0, np.sin(np.radians(self.rotation.x)), np.cos(np.radians(self.rotation.x))]])
        rotation_matrix_y = np.array([[np.cos(np.radians(self.rotation.y)), 0, np.sin(np.radians(self.rotation.y))],
                                      [0, 1, 0],
                                      [-np.sin(np.radians(self.rotation.y)), 0, np.cos(np.radians(self.rotation.y))]])
        rotation_matrix_z = np.array([[np.cos(np.radians(self.rotation.z)), -np.sin(np.radians(self.rotation.z)), 0],
                                      [np.sin(np.radians(self.rotation.z)), np.cos(np.radians(self.rotation.z)), 0],
                                      [0, 0, 1]])

        # update to rotations
        box_x, box_y, box_z = np.dot(np.array([box_x, box_y, box_z]).T - np.array([self.rotation_center.x, self.rotation_center.y, self.rotation_center.z]),
                                     rotation_matrix_x).T
        box_x, box_y, box_z = np.dot(np.array([box_x, box_y, box_z]).T, rotation_matrix_y).T
        box_x, box_y, box_z = np.dot(np.array([box_x, box_y, box_z]).T, rotation_matrix_z).T

        box_x += np.array(self.center.x)
        box_y += np.array(self.center.y)
        box_z += np.array(self.center.z)

        return box_x, box_y, box_z

    def plot(self):

        box_x, box_y, box_z = self.update_position_and_rotation()

        # Define the faces of the box
        faces = [
            [0, 1, 3, 2],  # Bottom face
            [4, 5, 7, 6],
            [0, 1, 4, 5],  # Front face
            [1, 2, 5, 6],  # Right face
            [2, 3, 6, 7],  # Back face
            [3, 0, 7, 4]]  # Left face"""

        # Plot the surfaces of the box with the specified RGB color
        for face in faces:
            x = box_x[face]
            y = box_y[face]
            z = box_z[face]

            self.ax.plot_surface(
                x.reshape((2, 2)),
                y.reshape((2, 2)),
                z.reshape((2, 2)),
                color=self.color,
                edgecolors="k"
            )

        max_dim = np.max([self.size.x, self.size.y, self.size.z])
        self.ax.set_xlim(np.array(self.center.x) - max_dim, np.array(self.center.x) + max_dim)
        self.ax.set_ylim(np.array(self.center.y) - max_dim, np.array(self.center.y) + max_dim)
        self.ax.set_zlim(np.array(self.center.z) - max_dim, np.array(self.center.z) + max_dim)
        self.ax.set_aspect('equal')

class EasySphere:

    # def __init__(self, center,  radius, ax="none"):
    def __init__(self, center, radius, rotation=Vector3(0, 0, 0), ax="none", color="none", edgecolor = (0.1,0.1,0.1,0.2), name="[unnamed sphere]", plot_density = [30,30]):
        self.center = center
        self.radius = radius
        self.rotation = rotation
        self.rotation_center = center #to specify a differnet rotation center it must be set explicitly
        self.ax = ax
        self.color = color
        self.edgecolor = edgecolor
        self.name = name
        self.plot_density = plot_density

    def __str__(self):
        return f"Sphere Object {self.name}: " \
               f"\n\tvariables" \
               f"\n\t\tcenter: (x={self.center.x}, y={self.center.y}, z={self.center.z})" \
               f"\n\t\trotation center: (x={self.rotation_center.x}, y={self.rotation_center.y}, z={self.rotation_center.z})" \
               f"\n\t\trotation: (x={self.rotation.x}, y={self.rotation.y}, z={self.rotation.z})" \
               f"\n\tfixed" \
               f"\n\t\tradius: {self.radius}" \
               f"\n\t\tcolor: {self.color}" \
               f")"

    def plot(self):
        u = np.linspace(np.pi/2.0, 2.5 * np.pi, self.plot_density[0]) #x
        v = np.linspace(0, np.pi, self.plot_density[1]) #y

        x = np.array(self.center.x) + self.radius * np.outer(np.cos(u), np.sin(v))
        y = np.array(self.center.y) + self.radius * np.outer(np.sin(u), np.sin(v))
        z = np.array(self.center.z) + self.radius * np.outer(np.ones(np.size(u)), np.cos(v))

        # Apply rotations
        rotation_matrix_x = np.array([[1, 0, 0],
                                      [0, np.cos(np.radians(self.rotation.x)), -np.sin(np.radians(self.rotation.x))],
                                      [0, np.sin(np.radians(self.rotation.x)), np.cos(np.radians(self.rotation.x))]])
        rotation_matrix_y = np.array([[np.cos(np.radians(self.rotation.y)), 0, np.sin(np.radians(self.rotation.y))],
                                      [0, 1, 0],
                                      [-np.sin(np.radians(self.rotation.y)), 0, np.cos(np.radians(self.rotation.y))]])
        rotation_matrix_z = np.array([[np.cos(np.radians(self.rotation.z)), -np.sin(np.radians(self.rotation.z)), 0],
                                      [np.sin(np.radians(self.rotation.z)), np.cos(np.radians(self.rotation.z)), 0],
                                      [0, 0, 1]])

        x, y, z = np.dot(np.array([x, y, z]).T - [np.array([self.center.x, self.center.y, self.center.z])], rotation_matrix_z).T
        x, y, z = np.dot(np.array([x, y, z]).T, rotation_matrix_y).T
        x, y, z = np.dot(np.array([x, y, z]).T, rotation_matrix_x).T

        x = x + self.center.x
        y = y + self.center.y
        z = z + self.center.z

        self.ax.plot_surface(x, y, z, color=self.color, edgecolor=self.edgecolor)

        max_range = self.radius * 1.5
        self.ax.set_xlim(np.array(self.center.x) - max_range, np.array(self.center.x) + max_range)
        self.ax.set_ylim(np.array(self.center.y) - max_range, np.array(self.center.y) + max_range)
        self.ax.set_zlim(np.array(self.center.z) - max_range, np.array(self.center.z) + max_range)
        self.ax.set_aspect('equal')

class EasyDrawer:
    def __init__(self, figsize):
        self.fig = plt.figure(figsize=figsize)
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.boxes = []
        self.spheres = []
        self.sphere_rotation_angle = 0

    def add_box(self, new_box):
        new_box.ax = self.ax
        self.boxes.append(new_box)
        return new_box

    def add_sphere(self, new_sphere):
        new_sphere.ax = self.ax
        self.spheres.append(new_sphere)
        return new_sphere

    def update_objects(self):
        for box in self.boxes:
            box.plot()
        for sphere in self.spheres:
            sphere.plot()

    def show_objects(self):
        model.ax.cla()  # Clear the previous plot
        self.update_objects()

        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')

        self.ax.set_xlim(-12, 12)
        self.ax.set_ylim(-12, 12)
        self.ax.set_zlim(-12, 12)

        #self.ax.view_init(elev=90, azim=0)
        plt.show()

# Create a function to update the sphere's y_rotation based on the slider value
def update_rotation(val):
    for box in model.boxes:
        box.rotation.x = x_rotation_slider.val

    pendulum2.rotation.z = (z_rotation_slider.val-90) * -math.cos(math.radians(x_rotation_slider.val-90))
    pendulum2.rotation.y = (z_rotation_slider.val-90) * -math.sin(math.radians(x_rotation_slider.val-90))

    model.show_objects()

# def update_rotation(val):
#     for sphere in model.spheres:
#         sphere.rotation.x = x_rotation_slider.val
#         sphere.rotation.y = y_rotation_slider.val
#         sphere.rotation.z = z_rotation_slider.val
#     model.show_objects()

# Create an instance of EasyDrawer
model = EasyDrawer(figsize=(8, 8))

sphere = model.add_sphere(EasySphere(center=Vector3(0, 0, 0), radius=12, name = "shell"))
pendulum1 = model.add_box(EasyBox(center= Vector3(0, 0, 0), rotation_center=Vector3(0,0,1.5), size = Vector3(2,2,2), color=(1,0.3,0.3,0.5), name="pendulum 1"))
pendulum2 = model.add_box(EasyBox(center= Vector3(0, 0, 0), rotation_center=Vector3(0,0,5), size = Vector3(2,2,5), color=(1,0.3,0.3,0.5)))
print(sphere)
# rod = model.add_box(center=[0, 0, 0], rotation_center=[0,0,0], width=24, height=1, depth=1,color=(0.5,0.5,1,0.3))
# rod.x_rotation = 90

y_rotation_ax = plt.axes([0.2, 0.1, 0.6, 0.05])
y_rotation_slider = Slider(y_rotation_ax, 'Y Rotation Angle', 0, 180, valinit=90)

# Create a slider for x_rotation
x_rotation_ax = plt.axes([0.2, 0.05, 0.6, 0.05])
x_rotation_slider = Slider(x_rotation_ax, 'X Rotation Angle', 0, 180, valinit=90)

z_rotation_ax = plt.axes([0.2, 0.0, 0.6, 0.05])
z_rotation_slider = Slider(z_rotation_ax, 'Z Rotation Angle', 0, 180, valinit=90)

y_rotation_slider.on_changed(update_rotation)
x_rotation_slider.on_changed(update_rotation)
z_rotation_slider.on_changed(update_rotation)

for frame in range(len(times)):
    #model.fig.clear()
    # Update the slider value to the current angle
    x_rotation_slider.value = drive_angles[frame]
    z_rotation_slider.value = pipe_angles[frame]

    # Wait for a short duration to allow for slider update
    time.sleep(0.1)

    # Perform any desired operations with the updated slider value
    # ...
    model.show_objects()