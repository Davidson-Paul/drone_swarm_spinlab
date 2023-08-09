#This script creates spline path over given few setpoints
#The output is an equally-spaced path setpoints
#Additionally yaw-rate/heading change of the path in x-y plane is checked

import numpy as np
from scipy.interpolate import splprep, splev
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def calculate_heading_change(points, time_diff, max_dh):
    num_points = len(points)
    
    # Check if there are enough points to calculate heading change
    if num_points < 2:
        raise ValueError("At least 2 points are required to calculate heading change.")
    
    # Calculate the differences between consecutive points
    deltas = np.diff(points, axis=0)
    
    # Calculate the heading angles of the differences
    headings = np.arctan2(deltas[:, 1], deltas[:, 0])
    headings = np.insert(headings,0,0)
    
    # Calculate the time intervals between consecutive points
    # time_diffs = np.diff(time)
    
    # Calculate the heading change rates (angular velocities)
    heading_change = np.diff(headings) / time_diff

    for i in heading_change:
        if(abs(i)>max_dh):
            print("Violating:",i)
    
    return headings,heading_change

vel_bound = 10
formation_dist = 50 #max formation dist from leader
dt=1/5 #update rate
vel = 5 #velocity of motion

yaw_rate_max=vel_bound/formation_dist
heading_change_max = yaw_rate_max*dt

# Generate some sample 3D data points
x = np.array([20, 96, 262, 338])
y = np.array([0, -20, 20, 0])
z = np.array([6,6,6,6])

# Fit a spline to the data
tck, u = splprep([x, y, z], s=10)

# Calculate the total length of the spline curve
u_new = np.linspace(0, 1, 1000)
curve_points = np.array(splev(u_new, tck)).T
curve_length = np.sum(np.sqrt(np.sum(np.diff(curve_points, axis=0) ** 2, axis=1)))

no_of_divisions = np.round(curve_length/(vel*dt),0)
no_of_points = no_of_divisions+1

target_distance = curve_length / no_of_divisions  # Divide curve into 10 equal distances

# Generate points at regular intervals along the spline curve
distances = np.linspace(0, curve_length, num=no_of_points)
u_interp = np.interp(distances, np.cumsum(np.sqrt(np.sum(np.diff(curve_points, axis=0) ** 2, axis=1))), np.linspace(0, 1, curve_points.shape[0] - 1))
x_new, y_new, z_new = splev(u_interp, tck)

# Create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the original data points and the spline curve
ax.scatter(x, y, z, color='red', label='Original Data')
ax.plot(x_new, y_new, z_new, color='blue', label='Spline Curve')

# Set labels and title
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('3D Spline Path')
ax.set_xlim(-50,250)
ax.set_ylim(-50,250)

# Add a legend
ax.legend()

print('vel:',vel)
print('dt:',dt)
print('distance:',curve_length)
print('no_of_points:',no_of_points)
print('yaw_rate_max:',yaw_rate_max)
print('heading_change_max:',heading_change_max)

# Show the plot
headings,heading_change = calculate_heading_change(np.column_stack((x_new,y_new)),dt,heading_change_max)
plt.show()

np.savetxt('path.csv', np.column_stack((x_new,y_new,z_new,headings)), delimiter=',')