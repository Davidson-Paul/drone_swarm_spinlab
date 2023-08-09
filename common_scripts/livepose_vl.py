#!/usr/bin/env python3

#Script plots the locations of each drone with /fury_1/ as origin
#Also Virtual Leader location is plotted (in local coordinates)
#Drone Distances are measured using global GPS coordinates

import rospy
import matplotlib.pyplot as plt
import numpy as np
from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion
from collections import deque
import time
from gazebo_msgs.msg import ModelStates
from mavros_msgs.msg import HomePosition
from sensor_msgs.msg import NavSatFix
import pymap3d as pm
from pygame import Vector3
from std_msgs.msg import Float64
import math
from iq_gnc.msg import VL_local

#------------------------------------------
#fury_1's NED is considered as global frame
        #N->xd:-ve y in gazebo
        #E->yd:+ve x in gazebo
        #D->zd:+ve z in gazebo
#------------------------------------------


# Use in SITL only
# drone1_loc_pose = Pose()
# drone2_loc_pose = Pose()
# drone3_loc_pose = Pose()
# drone4_loc_pose = Pose()
# drone5_loc_pose = Pose()

# Use for global frame
drone1_home_pos = HomePosition()

drone1_gpose = NavSatFix()
drone2_gpose = NavSatFix()
drone3_gpose = NavSatFix()
drone4_gpose = NavSatFix()
drone5_gpose = NavSatFix()
pr = Vector3()

drone1_heading=Float64()
drone2_heading=Float64()
drone3_heading=Float64()
drone4_heading=Float64()
drone5_heading=Float64()

drone1_trace = deque(maxlen=50)  # Stores the past positions (5 seconds * 10 Hz = 50 samples)
drone2_trace = deque(maxlen=50)
drone3_trace = deque(maxlen=50)
drone4_trace = deque(maxlen=50)
drone5_trace = deque(maxlen=50)
vl_trace = deque(maxlen=50)

def drone1_loc_pose_callback(data):
    global drone1_loc_pose
    drone1_loc_pose = data.pose

def drone2_loc_pose_callback(data):
    global drone2_loc_pose
    drone2_loc_pose = data.pose

def drone3_loc_pose_callback(data):
    global drone3_loc_pose
    drone3_loc_pose = data.pose

def drone4_loc_pose_callback(data):
    global drone4_loc_pose
    drone4_loc_pose = data.pose

def drone5_loc_pose_callback(data):
    global drone5_loc_pose
    drone5_loc_pose = data.pose

def drone1_home_position_callback(msg):
    global drone1_home_pos
    drone1_home_pos = msg.geo

def drone1_global_callback(data):
    global drone1_gpose
    drone1_gpose = data

def drone2_global_callback(data):
    global drone2_gpose
    drone2_gpose = data

def drone3_global_callback(data):
    global drone3_gpose
    drone3_gpose = data

def drone4_global_callback(data):
    global drone4_gpose
    drone4_gpose = data

def drone5_global_callback(data):
    global drone5_gpose
    drone5_gpose = data

def drone1_heading_callback(msg):
    global drone1_heading
    drone1_heading=msg

def drone2_heading_callback(msg):
    global drone2_heading
    drone2_heading=msg

def drone3_heading_callback(msg):
    global drone3_heading
    drone3_heading=msg

def drone4_heading_callback(msg):
    global drone4_heading
    drone4_heading=msg

def drone5_heading_callback(msg):
    global drone5_heading
    drone5_heading=msg

def vl_position_callback(msg):
    global pr
    pr = msg.position
    # self.vr = np.array([msg.linear_velocity.x,msg.linear_velocity.y,msg.linear_velocity.z])
    # self.ar = np.array([msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z])

# def gazpose_callback(data):
#     global drone1_gpose
#     global drone2_gpose
#     global drone3_gpose
#     global drone4_gpose
#     global drone5_gpose

#     for i in range(len(data.name)):
#         model_name=data.name[i]
#         if(model_name=='fury_1'):
#             drone1_gpose = data.pose[i]
#         elif(model_name=='fury_2'):
#             drone2_gpose = data.pose[i]
#         elif(model_name=='fury_3'):
#             drone3_gpose = data.pose[i]
#         elif(model_name=='fury_4'):
#             drone4_gpose = data.pose[i]
#         elif(model_name=='fury_5'):
#             drone5_gpose = data.pose[i]

def plot_drones():
    plt.ion()
    fig, ax = plt.subplots()

    # Set axis limits and aspect ratio
    ax.set_xlim(-15, 15)
    ax.set_ylim(-15, 15)
    ax.set_aspect('equal', adjustable='box')

    # Set up gridlines
    ax.set_xticks(np.arange(-15, 16, 5))
    ax.set_yticks(np.arange(-15, 16, 5))
    ax.grid(which='both')

    # Initialize time variable
    last_time = time.time()

    while not rospy.is_shutdown():
        #fury_1's NED is considered as global frame
        #N-xd:-ve y in gazebo
        #E-yd:+ve x in gazebo
        #D-zd:+ve z in gazebo
        (x1,y1,z1) = pm.geodetic2enu(drone1_gpose.latitude, drone1_gpose.longitude, drone1_gpose.altitude, drone1_home_pos.latitude, drone1_home_pos.longitude, drone1_home_pos.altitude)
        (x2,y2,z2) = pm.geodetic2enu(drone2_gpose.latitude, drone2_gpose.longitude, drone2_gpose.altitude, drone1_home_pos.latitude, drone1_home_pos.longitude, drone1_home_pos.altitude)
        (x3,y3,z3) = pm.geodetic2enu(drone3_gpose.latitude, drone3_gpose.longitude, drone3_gpose.altitude, drone1_home_pos.latitude, drone1_home_pos.longitude, drone1_home_pos.altitude)
        (x4,y4,z4) = pm.geodetic2enu(drone4_gpose.latitude, drone4_gpose.longitude, drone4_gpose.altitude, drone1_home_pos.latitude, drone1_home_pos.longitude, drone1_home_pos.altitude)
        (x5,y5,z5) = pm.geodetic2enu(drone5_gpose.latitude, drone5_gpose.longitude, drone5_gpose.altitude, drone1_home_pos.latitude, drone1_home_pos.longitude, drone1_home_pos.altitude)

        #Change the assignment for local/global
        drone1_position = Vector3(x1, y1, z1)
        drone2_position = Vector3(x2, y2, z2)
        drone3_position = Vector3(x3, y3, z3)
        drone4_position = Vector3(x4, y4, z4)
        drone5_position = Vector3(x5, y5, z5)

        print(drone1_position)
        print(drone2_position)
        print(drone3_position)
        print(drone4_position)
        print(drone5_position)
        print(pr)
        print()

        if drone1_position:
            ax.clear()

            # Reset axis limits, aspect ratio, and gridlines after clearing the plot
            ax.set_xlim(-300, 300)
            ax.set_ylim(-300, 300)
            ax.set_aspect('equal', adjustable='box')
            ax.set_xticks(np.arange(-300, 300, 20))
            ax.set_yticks(np.arange(-300, 300, 20))
            ax.grid(which='both')

            current_time = time.time()
            if current_time - last_time > 0.1:  # Store a new position every 0.1 seconds (10 Hz)
                drone1_trace.append(drone1_position)
                drone2_trace.append(drone2_position)
                drone3_trace.append(drone3_position)
                drone4_trace.append(drone4_position)
                drone5_trace.append(drone5_position)
                vl_trace.append(pr)
                last_time = current_time

            # Plot past traces
            for i, (p1, p2, p3, p4, p5, vl) in enumerate(
                    zip(drone1_trace, drone2_trace, drone3_trace, drone4_trace, drone5_trace, vl_trace)):
                alpha = i / len(drone1_trace)
                ax.scatter(p1.x, p1.y, c=[[0, 0, 1, alpha]], s=5)  # Drone 1 in blue
                ax.scatter(p2.x, p2.y, c=[[1, 0, 0, alpha]], s=5)  # Drone 2 in red
                ax.scatter(p3.x, p3.y, c=[[0, 1, 0, alpha]], s=5)  # Drone 3 in green
                ax.scatter(p4.x, p4.y, c=[[1, 1, 0, alpha]], s=5)  # Drone 4 in yellow
                ax.scatter(p5.x, p5.y, c=[[1, 0, 1, alpha]], s=5)  # Drone 5 in magenta
                ax.scatter(vl.x, vl.y, c=[[0, 1, 1, alpha]], s=5)

            # ... (rest of the code inside the while loop remains the same)

            #For SITL (yaw calculated in NED frame)
            # Get Euler angles from quaternions 
            # _, _, drone1_yaw = euler_from_quaternion(
            #     [drone1_loc_pose.orientation.x, drone1_loc_pose.orientation.y, drone1_loc_pose.orientation.z,
            #      drone1_loc_pose.orientation.w])
            # _, _, drone2_yaw = euler_from_quaternion(
            #     [drone2_loc_pose.orientation.x, drone2_loc_pose.orientation.y, drone2_loc_pose.orientation.z,
            #      drone2_loc_pose.orientation.w])
            # _, _, drone3_yaw = euler_from_quaternion(
            #     [drone3_loc_pose.orientation.x, drone3_loc_pose.orientation.y, drone3_loc_pose.orientation.z,
            #      drone3_loc_pose.orientation.w])
            # _, _, drone4_yaw = euler_from_quaternion(
            #     [drone4_loc_pose.orientation.x, drone4_loc_pose.orientation.y, drone4_loc_pose.orientation.z,
            #      drone4_loc_pose.orientation.w])
            # _, _, drone5_yaw = euler_from_quaternion(
            #     [drone5_loc_pose.orientation.x, drone5_loc_pose.orientation.y, drone5_loc_pose.orientation.z,
            #      drone5_loc_pose.orientation.w])

            #For global
            drone1_yaw=math.radians(90.0-float(drone1_heading.data))
            drone2_yaw=math.radians(90.0-float(drone2_heading.data))
            drone3_yaw=math.radians(90.0-float(drone3_heading.data))
            drone4_yaw=math.radians(90.0-float(drone4_heading.data))
            drone5_yaw=math.radians(90.0-float(drone5_heading.data))

            # Calculate arrow coordinates
            drone1_arrow_dx = 5*np.cos(drone1_yaw)
            drone1_arrow_dy = 5*np.sin(drone1_yaw)
            drone2_arrow_dx = 5*np.cos(drone2_yaw)
            drone2_arrow_dy = 5*np.sin(drone2_yaw)
            drone3_arrow_dx = 5*np.cos(drone3_yaw)
            drone3_arrow_dy = 5*np.sin(drone3_yaw)
            drone4_arrow_dx = 5*np.cos(drone4_yaw)
            drone4_arrow_dy = 5*np.sin(drone4_yaw)
            drone5_arrow_dx = 5*np.cos(drone5_yaw)
            drone5_arrow_dy = 5*np.sin(drone5_yaw)

            # Plot drone positions and heading arrows
            ax.scatter(drone1_position.x, drone1_position.y, c='blue', label='Drone 1')
            ax.arrow(drone1_position.x, drone1_position.y, drone1_arrow_dx, drone1_arrow_dy, color='blue',
                     head_width=2, head_length=0.5)
            
            ax.scatter(drone2_position.x, drone2_position.y, c='red', label='Drone 2')
            ax.arrow(drone2_position.x, drone2_position.y, drone2_arrow_dx, drone2_arrow_dy, color='red',
                     head_width=2, head_length=0.5)
            
            ax.scatter(drone3_position.x, drone3_position.y, c='green', label='Drone 3')
            ax.arrow(drone3_position.x, drone3_position.y, drone3_arrow_dx, drone3_arrow_dy, color='green',
                     head_width=2, head_length=0.5)
            
            ax.scatter(drone4_position.x, drone4_position.y, c='yellow', label='Drone 4')
            ax.arrow(drone4_position.x, drone4_position.y, drone4_arrow_dx, drone4_arrow_dy, color='yellow',
                     head_width=2, head_length=0.5)
            
            ax.scatter(drone5_position.x, drone5_position.y, c='magenta', label='Drone 5')
            ax.arrow(drone5_position.x, drone5_position.y, drone5_arrow_dx, drone5_arrow_dy, color='magenta',
                     head_width=2, head_length=0.5)
            
            ax.scatter(pr.x, pr.y, c='cyan', label='VL')

            ax.set_xlabel('X (m)')
            ax.set_ylabel('Y (m)')

            # Positioning the labels
            ax.xaxis.set_label_coords(1.05, 0.5)
            ax.yaxis.set_label_coords(0.5, 1.05)

            ax.legend()
            plt.pause(0.001)

    plt.ioff()
    plt.show()


def main():
    rospy.init_node('drone_tracing')

    #for use in SITL only
    # rospy.Subscriber('fury_1/mavros/local_position/pose', PoseStamped, drone1_loc_pose_callback)
    # rospy.Subscriber('fury_2/mavros/local_position/pose', PoseStamped, drone2_loc_pose_callback)
    # rospy.Subscriber('fury_3/mavros/local_position/pose', PoseStamped, drone3_loc_pose_callback)
    # rospy.Subscriber('fury_4/mavros/local_position/pose', PoseStamped, drone4_loc_pose_callback)
    # rospy.Subscriber('fury_5/mavros/local_position/pose', PoseStamped, drone5_loc_pose_callback)


    #For global frame
    rospy.Subscriber('fury_1/mavros/global_position/global', NavSatFix, drone1_global_callback)
    rospy.Subscriber('fury_2/mavros/global_position/global', NavSatFix, drone2_global_callback)
    rospy.Subscriber('fury_3/mavros/global_position/global', NavSatFix, drone3_global_callback)
    rospy.Subscriber('fury_4/mavros/global_position/global', NavSatFix, drone4_global_callback)
    rospy.Subscriber('fury_5/mavros/global_position/global', NavSatFix, drone5_global_callback)

    rospy.Subscriber('/fury_1/mavros/global_position/compass_hdg', Float64, drone1_heading_callback)
    rospy.Subscriber('/fury_2/mavros/global_position/compass_hdg', Float64, drone2_heading_callback)
    rospy.Subscriber('/fury_3/mavros/global_position/compass_hdg', Float64, drone3_heading_callback)
    rospy.Subscriber('/fury_4/mavros/global_position/compass_hdg', Float64, drone4_heading_callback)
    rospy.Subscriber('/fury_5/mavros/global_position/compass_hdg', Float64, drone5_heading_callback)

    rospy.Subscriber('fury_1/mavros/home_position/home', HomePosition, drone1_home_position_callback)
    rospy.wait_for_message('fury_1/mavros/home_position/home', HomePosition)

    rospy.Subscriber('/VL_states', VL_local, vl_position_callback)


    # rospy.Subscriber('/gazebo/model_states', ModelStates, gazpose_callback)

    plot_drones()


if __name__ == '__main__':
    main()
