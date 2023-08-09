#!/usr/bin/env python3

#Script for plotting the spacing between the drone's actual position and its VC

import rospy
from geometry_msgs.msg import PoseStamped
import math
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button
from gazebo_msgs.msg import ModelStates
from mavros_msgs.msg import HomePosition
from sensor_msgs.msg import NavSatFix
import pymap3d as pm
from pygame import Vector3
from iq_gnc.msg import virtual_centre

drone1_gpose = NavSatFix()
drone1_vc_pose = Vector3()


class DistanceCollector:
    def __init__(self):
        self.node = rospy.init_node('distance_collector', anonymous=True)

        #Subscribing to lat,long,alt topic
        rospy.Subscriber('fury_1/mavros/global_position/global', NavSatFix, self.drone1_global_callback)
        rospy.Subscriber('fury_1/virtual_centre', virtual_centre, self.drone1_vc_callback)

        #Considering /fury_1/ as origin
        #Hence subscribing to its home position
        rospy.Subscriber('fury_1/mavros/home_position/home', HomePosition, self.drone1_home_position_callback)
        rospy.wait_for_message('fury_1/mavros/home_position/home', HomePosition)

        #Plotting at 5Hz rate
        self.rate=rospy.Rate(5)
        self.dt=0.2

        self.position1 = None

        self.err_distance = []

        self.start_time = None
        self.end_time = None


    #Function to calculate distance bw points in x-y plane
    def get_distance(self, pos1, pos2):
        if pos1 is not None and pos2 is not None:
            dx = pos1.x - pos2.x
            dy = pos1.y - pos2.y
            distance = math.sqrt(dx ** 2 + dy ** 2)
            return distance
        return None
    
    
    #Callback functions
    def drone1_global_callback(self, data):
        global drone1_gpose
        drone1_gpose = data

    def drone1_vc_callback(self, data):
        global drone1_vc_pose
        drone1_vc_pose = data.position

    def drone1_home_position_callback(self, msg):
        global drone1_home_pos
        drone1_home_pos = msg.geo

    #Plotting
    def update_plot(self, frame):

        #Getting ENU values from GPS data relative to /fury_1/ home_position
        (x1, y1, z1) = pm.geodetic2enu(
            drone1_gpose.latitude,
            drone1_gpose.longitude,
            drone1_gpose.altitude,
            drone1_home_pos.latitude,
            drone1_home_pos.longitude,
            drone1_home_pos.altitude,
        )

        self.position1 = Vector3(x1, y1, z1)

        distance12 = self.get_distance(self.position1, drone1_vc_pose)

        if distance12 is not None:
            self.err_distance.append(distance12)

        self.ax.clear()
        # self.ax.set_xlim([0, 150])
        # self.ax.set_ylim([0, 5])
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Distance (m)')
        self.ax.grid(True)
        if self.err_distance:
            self.ax.plot(
                [(t - 0)*self.dt for t in range(len(self.err_distance))],
                self.err_distance,
                label="VP to Fury_1: {:.2f} m".format(distance12),
            )
        self.ax.legend()

    #Clearing the plot
    def clear_plot(self, event):
        self.err_distance = []

    def run(self):
        fig = plt.figure()
        self.ax = fig.add_subplot(111)

        ani = FuncAnimation(fig, self.update_plot, interval=200) #Interval set according to 5Hz rate

        # Add clear button
        clear_button_ax = fig.add_axes([0.81, 0.9, 0.1, 0.05])
        clear_button = Button(clear_button_ax, 'Clear')
        clear_button.on_clicked(self.clear_plot)

        plt.show()


if __name__ == '__main__':
    collector = DistanceCollector()
    collector.run()
