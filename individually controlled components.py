import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider

class EasyBox:
    def __init__(self, center,rotation_center, width, height, depth, ax="none"):
        self.center = np.array(center)
        self.width = width
        self.height = height
        self.depth = depth
        self.ax = ax
        self.x_rotation = 0
        self.y_rotation = 0
        self.z_rotation = 0

        self.rotation_center = np.array(rotation_center)

    def plot(self):
        half_width = self.width / 2
        half_height = self.height / 2
        half_depth = self.depth / 2

        box_x = np.array([-1, 1, 1, -1, -1, 1, 1, -1]) * half_width + self.center[0]
        box_y = np.array([1, 1, -1, -1, 1, 1, -1, -1]) * half_height + self.center[1]
        box_z = np.array([1, 1, 1, 1, -1, -1, -1, -1]) * half_depth + self.center[2]

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
        color_rgb = (0.2, 0.4, 0.6)  # RGB values normalized to [0, 1]

        # Plot the surfaces of the box with the specified RGB color
        for face in faces:
            x = box_x[face]
            y = box_y[face]
            z = box_z[face]

            vertices = np.column_stack([x, y, z])
            self.ax.plot_surface(
                x.reshape((2, 2)),
                y.reshape((2, 2)),
                z.reshape((2, 2)),
                color=color_rgb,
                edgecolors="k"
            )

        max_dim = np.max([self.width, self.height, self.depth])
        self.ax.set_xlim(self.center[0] - max_dim, self.center[0] + max_dim)
        self.ax.set_ylim(self.center[1] - max_dim, self.center[1] + max_dim)
        self.ax.set_zlim(self.center[2] - max_dim, self.center[2] + max_dim)
        self.ax.set_aspect('equal')

class EasySphere:
    def __init__(self, center, radius, ax="none"):
        self.center = np.array(center)
        self.radius = radius
        self.ax = ax
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

        self.ax.plot_surface(x, y, z, color='none', edgecolor=(0.1,0.1,0.1,0.2))

        max_range = self.radius * 1.5
        self.ax.set_xlim(self.center[0] - max_range, self.center[0] + max_range)
        self.ax.set_ylim(self.center[1] - max_range, self.center[1] + max_range)
        self.ax.set_zlim(self.center[2] - max_range, self.center[2] + max_range)
        self.ax.set_aspect('equal')

class EasyDrawer:
    def __init__(self, figsize=(8,6)):
        self.fig = plt.figure(figsize=figsize)
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.boxes = []
        self.spheres = []
        self.sphere_rotation_angle = 0

    def add_box(self, center,rotation_center, width, height, depth):
        new_box = EasyBox(center=center,rotation_center=rotation_center, width=width, height=height, depth=depth, ax=self.ax)
        self.boxes.append(new_box)
        return new_box

    def add_sphere(self, center, radius):
        new_sphere = EasySphere(center=center, radius=radius, ax=self.ax)
        self.spheres.append(new_sphere)
        return new_sphere

    def update_objects(self):
        for box in self.boxes:
            box.plot()
        for sphere in self.spheres:
            sphere.plot(sphere.x_rotation, sphere.y_rotation, sphere.z_rotation)

        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')

        self.ax.set_xlim(-12, 12)
        self.ax.set_ylim(-12, 12)
        self.ax.set_zlim(-12, 12)

    def show_objects(self):
        self.update_objects()
        #self.ax.view_init(elev=90, azim=0)
        plt.show()

# Create a function to update the sphere's y_rotation based on the slider value
def update_rotation(val):
    model.ax.cla()  # Clear the previous plot

    #print("sliders", y_rotation_slider.val, x_rotation_slider.val)
    #sphere.y_rotation = y_rotation_slider.val
    #sphere.x_rotation = -x_rotation_slider.val
    #sphere.z_rotation = z_rotation_slider.val # z does the same as x

    for box in model.boxes:
        box.x_rotation = x_rotation_slider.val
        #box.y_rotation = y_rotation_slider.val+ 90
        #box.z_rotation = z_rotation_slider.val

    model.show_objects()
    plt.show()

#width is x, height is y, depth is z
#unit is inches
#model = EasyDrawer(figsize=(10,10))

#sphere = model.add_sphere(center=[0, 0, 12], radius=12)
#model.add_box(center=[0, 0, 12], width=2, height=5, depth=6)


# Create an instance of EasyDrawer
model = EasyDrawer(figsize=(8, 8))

# Add a sphere to the model
sphere = model.add_sphere(center=[0, 0, 12], radius=12)

pendulum1 = model.add_box(center=[0, 0, 0], rotation_center=[0,0,1.5], width=2, height=2, depth=2)

pendulum2 = model.add_box(center=[0, 0, 0], rotation_center=[0,0,4], width=2, height=2, depth=4)


rod = model.add_box(center=[0, 0, 0], rotation_center=[0,0,0], width=24, height=1, depth=1)
rod.x_rotation = 90



#y
y_rotation_ax = plt.axes([0.2, 0.1, 0.6, 0.05])
y_rotation_slider = Slider(y_rotation_ax, 'Y Rotation Angle', 0, 360, valinit=90)

# Create a slider for x_rotation
x_rotation_ax = plt.axes([0.2, 0.05, 0.6, 0.05])
x_rotation_slider = Slider(x_rotation_ax, 'X Rotation Angle', 0, 360, valinit=0)

z_rotation_ax = plt.axes([0.2, 0.0, 0.6, 0.05])
z_rotation_slider = Slider(z_rotation_ax, 'Z Rotation Angle', 0, 360, valinit=0)

y_rotation_slider.on_changed(update_rotation)
x_rotation_slider.on_changed(update_rotation)
z_rotation_slider.on_changed(update_rotation)


# Initial plot
model.show_objects()
plt.show()
update_rotation()