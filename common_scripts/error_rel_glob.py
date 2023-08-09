#!/usr/bin/env python3

#Script for plotting the rel. spacing of the drones wrt the leader
#Distances are measured using global GPS coordinates

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

drone1_gpose = NavSatFix()
drone2_gpose = NavSatFix()
drone3_gpose = NavSatFix()
drone4_gpose = NavSatFix()
drone5_gpose = NavSatFix()


class DistanceCollector:
    def __init__(self):
        self.node = rospy.init_node('distance_collector', anonymous=True)
        
        #Subscribing to lat,long,alt topic
        rospy.Subscriber('fury_1/mavros/global_position/global', NavSatFix, self.drone1_global_callback)
        rospy.Subscriber('fury_2/mavros/global_position/global', NavSatFix, self.drone2_global_callback)
        rospy.Subscriber('fury_3/mavros/global_position/global', NavSatFix, self.drone3_global_callback)
        rospy.Subscriber('fury_4/mavros/global_position/global', NavSatFix, self.drone4_global_callback)
        rospy.Subscriber('fury_5/mavros/global_position/global', NavSatFix, self.drone5_global_callback)

        #Considering /fury_1/ as origin
        #Hence subscribing to its home position
        rospy.Subscriber('fury_1/mavros/home_position/home', HomePosition, self.drone1_home_position_callback)
        rospy.wait_for_message('fury_1/mavros/home_position/home', HomePosition)

        #Plotting at 5Hz rate
        self.rate=rospy.Rate(5)
        self.dt=0.2

        self.position1 = None
        self.position2 = None
        self.position3 = None
        self.position4 = None
        self.position5 = None
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
    def drone1_global_callback(self, data):
        global drone1_gpose
        drone1_gpose = data

    def drone2_global_callback(self, data):
        global drone2_gpose
        drone2_gpose = data

    def drone3_global_callback(self, data):
        global drone3_gpose
        drone3_gpose = data

    def drone4_global_callback(self, data):
        global drone4_gpose
        drone4_gpose = data

    def drone5_global_callback(self, data):
        global drone5_gpose
        drone5_gpose = data

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
        (x2, y2, z2) = pm.geodetic2enu(
            drone2_gpose.latitude,
            drone2_gpose.longitude,
            drone2_gpose.altitude,
            drone1_home_pos.latitude,
            drone1_home_pos.longitude,
            drone1_home_pos.altitude,
        )
        (x3, y3, z3) = pm.geodetic2enu(
            drone3_gpose.latitude,
            drone3_gpose.longitude,
            drone3_gpose.altitude,
            drone1_home_pos.latitude,
            drone1_home_pos.longitude,
            drone1_home_pos.altitude,
        )
        (x4, y4, z4) = pm.geodetic2enu(
            drone4_gpose.latitude,
            drone4_gpose.longitude,
            drone4_gpose.altitude,
            drone1_home_pos.latitude,
            drone1_home_pos.longitude,
            drone1_home_pos.altitude,
        )
        (x5, y5, z5) = pm.geodetic2enu(
            drone5_gpose.latitude,
            drone5_gpose.longitude,
            drone5_gpose.altitude,
            drone1_home_pos.latitude,
            drone1_home_pos.longitude,
            drone1_home_pos.altitude,
        )

        self.position1 = Vector3(x1, y1, z1)
        self.position2 = Vector3(x2, y2, z2)
        self.position3 = Vector3(x3, y3, z3)
        self.position4 = Vector3(x4, y4, z4)
        self.position5 = Vector3(x5, y5, z5)

        #Offsets are set according to the V-shape formation
        distance12 = self.get_distance(self.position1, self.position2) - 25.0
        distance13 = self.get_distance(self.position1, self.position3) - 25.0
        distance14 = self.get_distance(self.position1, self.position4) - 50.0
        distance15 = self.get_distance(self.position1, self.position5) - 50.0

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
        if self.distance_data12:
            self.ax.plot(
                [(t - 0)*self.dt for t in range(len(self.distance_data12))],
                self.distance_data12,
                label="Fury_1 to Fury_2: {:.2f} m".format(distance12),
            )
        if self.distance_data13:
            self.ax.plot(
                [(t - 0)*self.dt for t in range(len(self.distance_data13))],
                self.distance_data13,
                label="Fury_1 to Fury_3: {:.2f} m".format(distance13),
            )
        if self.distance_data14:
            self.ax.plot(
                [(t - 0)*self.dt for t in range(len(self.distance_data14))],
                self.distance_data14,
                label="Fury_1 to Fury_4: {:.2f} m".format(distance14),
            )
        if self.distance_data15:
            self.ax.plot(
                [(t - 0)*self.dt for t in range(len(self.distance_data15))],
                self.distance_data15,
                label="Fury_1 to Fury_5: {:.2f} m".format(distance15),
            )
        self.ax.legend()

    #Clearing the plot
    def clear_plot(self, event):
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
    collector = DistanceCollector()
    collector.run()
