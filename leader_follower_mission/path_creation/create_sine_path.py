#This script creates time-dependent path
#Then the path is converted to an equally-spaced path setpoints
#Additionally yaw-rate/heading change of the path in x-y plane is checked

from scipy.interpolate import interp1d
import numpy as np
from scipy.interpolate import splprep, splev
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math

def interpolate_3d_path(path,vel,dt):
    # Extract x, y, z coordinates from the path
    x = path[:, 0]
    y = path[:, 1]
    z = path[:, 2]

    # Calculate the total length of the path
    path_length = np.sum(np.sqrt(np.diff(x)**2 + np.diff(y)**2 + np.diff(z)**2))

    # Calculate the number of setpoints required based on spacing
    spacing = vel*dt
    num_setpoints = int(path_length / spacing)

    # Create a parameterization of the path based on distance
    distance = np.cumsum(np.sqrt(np.diff(x)**2 + np.diff(y)**2 + np.diff(z)**2))
    distance = np.insert(distance, 0, 0)  # Add initial distance as 0
    parameterization = np.linspace(0, distance[-1], num_setpoints)

    # Interpolate x, y, z coordinates using parameterization
    interp_x = interp1d(distance, x)(parameterization)
    interp_y = interp1d(distance, y)(parameterization)
    interp_z = interp1d(distance, z)(parameterization)

    # Create a new set of equally spaced setpoints
    setpoints = np.column_stack((interp_x, interp_y, interp_z))

    return setpoints

def calculate_heading_change(points, time_diff, max_dh):
    num_points = len(points)
    
    # Check if there are enough points to calculate heading change
    if num_points < 2:
        raise ValueError("At least 2 points are required to calculate heading change.")
    
    # Calculate the differences between consecutive points
    deltas = np.diff(points, axis=0)
    
    # Calculate the heading angles of the differences
    headings = np.arctan2(deltas[:, 1], deltas[:, 0])
    
    # Calculate the time intervals between consecutive points
    # time_diffs = np.diff(time)
    
    # Calculate the heading change rates (angular velocities)
    heading_change = np.diff(headings) / time_diff

    for i in heading_change:
        if(abs(i)>max_dh):
            print("Violating:",i)
    
    return heading_change

def generate_sinusoidal_path(amplitude, frequency, time):
    t = np.arange(0,time,0.2)  # Time parameter
    x = t  # X-values representing frequency
    y = amplitude * np.sin(2*math.pi*frequency*t)  # Y-values representing amplitude
    z = np.full_like(t, 6)  # Constant z-value of 6

    setpoints = np.column_stack((x, y, z))  # Combine x, y, z into setpoints

    return setpoints

def generate_semicircular_path(r):
    t = np.arange(0,180,0.2)  # Time parameter
    x = -r*np.cos(np.radians(t)) + r  # X-values representing frequency
    y = r*np.sin(np.radians(t))  # Y-values representing amplitude
    z = np.full_like(t, 6)  # Constant z-value of 6

    setpoints = np.column_stack((x, y, z))  # Combine x, y, z into setpoints

    return setpoints

vel_bound = 10
formation_dist = 50 #max formation dist from leader
dt=1/5 #update rate
vel = 5 #velocity of motion

yaw_rate_max=vel_bound/formation_dist
heading_change_max = yaw_rate_max*dt

sine=generate_sinusoidal_path(20,1/300,300)
sine_equal=interpolate_3d_path(sine,vel,dt)

scircle=generate_semicircular_path(75)
scircle_equal=interpolate_3d_path(scircle,vel,dt)

# Create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the original data points and the spline curve
# ax.scatter(x, y, z, color='red', label='Original Data')
ax.scatter(scircle_equal[:,0], scircle_equal[:,1], scircle_equal[:,2], 'bo')

# Set labels and title
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('3D Spline Path')
ax.set_xlim(-200,200)
ax.set_ylim(-200,200)

# Add a legend
ax.legend()

# print('vel:',vel)
# print('dt:',dt)
# print('distance:',curve_length)
# print('no_of_points:',no_of_points)
# print('yaw_rate_max:',yaw_rate_max)
# print('heading_change_max:',heading_change_max)

# Show the plot
calculate_heading_change(sine_equal,dt,heading_change_max)
plt.show()

# np.savetxt('circle.csv', np.column_stack((x_new,y_new,z_new)), delimiter=',')