#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import PoseStamped, TwistStamped
from tf.transformations import euler_from_quaternion
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button
import math
import numpy as np

class plotter:
    def __init__(self):
        rospy.init_node("yaw_plot_d1")

        drone = '/ditto_1/'

        # rospy.Subscriber(f'{drone}mavros/global_position/global', NavSatFix, self.glob_position_callback)
        # rospy.Subscriber(f'{drone}mavros/global_position/rel_alt', Float64, self.rel_alt_callback)
        rospy.Subscriber(f'{drone}mavros/global_position/compass_hdg', Float64, self.heading_callback)
        rospy.Subscriber(f'{drone}mavros/global_position/local', Odometry, self.glob_loc_callback)

        rospy.Subscriber(f'{drone}mavros/local_position/pose', PoseStamped, self.local_position_callback)
        # rospy.Subscriber(f'{drone}/mavros/local_position/velocity_local', TwistStamped, self.local_velocity_callback)

        # rospy.Subscriber(f'{drone}/mavros/imu/data', Imu, self.acceleration_callback)

        #GPS
        self.lat = []
        self.long = []
        self.alt = []
        self.rel_alt = []

        #Position
        self.glob_x = []
        self.glob_y = []
        self.glob_z = []

        self.loc_x = []
        self.loc_y = []
        self.loc_z = []

        #Velocity
        self.glob_vx = []
        self.glob_vy = []
        self.glob_vz = []

        self.loc_vx = []
        self.loc_vy = []
        self.loc_vz = []

        #Body-acceleration
        self.ax = []
        self.ay = []
        self.az = []

        #Yaw
        self.yaw_mag = []
        self.yaw_glob_quat = []
        self.yaw_loc_quat = []

        #Plot update rate
        self.dt=0.2
        self.interval = 200

    def update_plot(self,frame):
        self.ax.clear()
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Yaw (deg)')
        self.ax.grid(True)

        self.ax.plot([(t - 0)*self.dt for t in range(len(self.yaw_mag))],self.yaw_mag,label="Yaw_Magn")
        self.ax.plot([(t - 0)*self.dt for t in range(len(self.yaw_glob_quat))],self.yaw_glob_quat,label="Yaw_glob_quat")
        self.ax.plot([(t - 0)*self.dt for t in range(len(self.yaw_loc_quat))],self.yaw_loc_quat,label="Yaw_loc_quat")

        self.ax.legend()

    def clear_plot(self, event):
        #GPS
        self.lat = []
        self.long = []
        self.alt = []
        self.rel_alt = []

        #Position
        self.glob_x = []
        self.glob_y = []
        self.glob_z = []

        self.loc_x = []
        self.loc_y = []
        self.loc_z = []

        #Velocity
        self.glob_vx = []
        self.glob_vy = []
        self.glob_vz = []

        self.loc_vx = []
        self.loc_vy = []
        self.loc_vz = []

        #Body-acceleration
        self.ax = []
        self.ay = []
        self.az = []

        #Yaw
        self.yaw_mag = []
        self.yaw_glob_quat = []
        self.yaw_loc_quat = []

    def run(self):
        fig1 = plt.figure()
        self.ax = fig1.add_subplot(111)

        ani = FuncAnimation(fig1, self.update_plot, interval=self.interval) #Interval set according to 5Hz rate

        # Add clear button
        clear_button_ax = fig1.add_axes([0.81, 0.9, 0.1, 0.05])
        clear_button = Button(clear_button_ax, 'Clear')
        clear_button.on_clicked(self.clear_plot)

        plt.show()

    def heading_callback(self,msg):
        yaw=(float(90.0-msg.data))
        if (yaw>180):
            yaw-=360
        if (yaw<-180):
            yaw+=360
        self.yaw_mag.append(yaw)

    def glob_loc_callback(self,msg):
        (roll, pitch, yaw) = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.yaw_glob_quat.append(math.degrees(yaw))

    def local_position_callback(self,msg):
        (roll, pitch, yaw) = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        self.yaw_loc_quat.append(math.degrees(yaw))

    def rel_alt_callback(self,msg):
        self.rel_alt.append(msg.data)

    def save_data(self):
        column1 = np.array(self.yaw_mag)
        column2 = np.array(self.yaw_glob_quat)
        column3 = np.array(self.yaw_loc_quat)
        np.savetxt('yaw_data.csv', np.column_stack((column1,column2)), delimiter=',')

if __name__ == '__main__':
    yaw_plot = plotter()
    yaw_plot.run()
    yaw_plot.save_data()