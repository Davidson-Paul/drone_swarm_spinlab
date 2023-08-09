#!/usr/bin/env python3

#Virtual Centre - VC
#Script for plotting the deviation of the drones' VC wrt the actual VC
#Distances are measured in local inertial coordinates

import rospy
import math
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button
from iq_gnc.msg import virtual_centre, VL_local
from pygame import Vector3

vl_pose = Vector3()
drone1_vc_pose = Vector3()
drone2_vc_pose = Vector3()
drone3_vc_pose = Vector3()
drone4_vc_pose = Vector3()
drone5_vc_pose = Vector3()


class VC_DistanceCollector:
    def __init__(self):
        self.node = rospy.init_node('distance_collector', anonymous=True)
        
        #Subscribing to actual VC
        rospy.Subscriber('/VL_states', VL_local, self.vl_callback)

        #Subscribing to each drone's own view of virtual centre(VC)
        rospy.Subscriber('fury_1/virtual_centre', virtual_centre, self.drone1_callback)
        rospy.Subscriber('fury_2/virtual_centre', virtual_centre, self.drone2_callback)
        rospy.Subscriber('fury_3/virtual_centre', virtual_centre, self.drone3_callback)
        rospy.Subscriber('fury_4/virtual_centre', virtual_centre, self.drone4_callback)
        rospy.Subscriber('fury_5/virtual_centre', virtual_centre, self.drone5_callback)

        #Plotting at 5Hz rate
        self.rate=rospy.Rate(5)
        self.dt=0.2

        self.distance_data11 = []
        self.distance_data12 = []
        self.distance_data13 = []
        self.distance_data14 = []
        self.distance_data15 = []
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
    def vl_callback(self,data):
        global vl_pose
        vl_pose = data.position

    def drone1_callback(self, data):
        global drone1_vc_pose
        drone1_vc_pose = data.position

    def drone2_callback(self, data):
        global drone2_vc_pose
        drone2_vc_pose = data.position

    def drone3_callback(self, data):
        global drone3_vc_pose
        drone3_vc_pose = data.position

    def drone4_callback(self, data):
        global drone4_vc_pose
        drone4_vc_pose = data.position

    def drone5_callback(self, data):
        global drone5_vc_pose
        drone5_vc_pose = data.position


    def update_plot(self, frame):
        distance11 = self.get_distance(vl_pose, drone1_vc_pose)
        distance12 = self.get_distance(vl_pose, drone2_vc_pose)
        distance13 = self.get_distance(vl_pose, drone3_vc_pose)
        distance14 = self.get_distance(vl_pose, drone4_vc_pose)
        distance15 = self.get_distance(vl_pose, drone5_vc_pose)

        if distance11 is not None:
            self.distance_data11.append(distance11)
        if distance12 is not None:
            self.distance_data12.append(distance12)
        if distance13 is not None:
            self.distance_data13.append(distance13)
        if distance14 is not None:
            self.distance_data14.append(distance14)
        if distance15 is not None:
            self.distance_data15.append(distance15)

        self.ax.clear()
        # self.ax.set_xlim([0, 150])
        # self.ax.set_ylim([0, 5])
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Distance (m)')
        self.ax.grid(True)
        if self.distance_data11:
            self.ax.plot(
                [(t - 0)*self.dt for t in range(len(self.distance_data11))],
                self.distance_data11,
                label="VC VL to Fury_1: {:.2f} m".format(distance11),
            )
        if self.distance_data12:
            self.ax.plot(
                [(t - 0)*self.dt for t in range(len(self.distance_data12))],
                self.distance_data12,
                label="VC VL to Fury_2: {:.2f} m".format(distance12),
            )
        if self.distance_data13:
            self.ax.plot(
                [(t - 0)*self.dt for t in range(len(self.distance_data13))],
                self.distance_data13,
                label="VC VL to Fury_3: {:.2f} m".format(distance13),
            )
        if self.distance_data14:
            self.ax.plot(
                [(t - 0)*self.dt for t in range(len(self.distance_data14))],
                self.distance_data14,
                label="VC VL to Fury_4: {:.2f} m".format(distance14),
            )
        if self.distance_data15:
            self.ax.plot(
                [(t - 0)*self.dt for t in range(len(self.distance_data15))],
                self.distance_data15,
                label="VC VL to Fury_5: {:.2f} m".format(distance15),
            )
        self.ax.legend()

    def clear_plot(self, event):
        self.distance_data11 = []
        self.distance_data12 = []
        self.distance_data13 = []
        self.distance_data14 = []
        self.distance_data15 = []

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
    collector = VC_DistanceCollector()
    collector.run()
