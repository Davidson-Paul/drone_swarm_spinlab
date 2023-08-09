#!/usr/bin/env python3

import math
import pymap3d as pm

import rospy
from sensor_msgs.msg import Imu, NavSatFix
from mavros_msgs.msg import GlobalPositionTarget
from geometry_msgs.msg import Vector3, Odometry
from std_msgs.msg import Float64

class DroneController:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('global_old_controller_D2')

        # ------------------------
        #Updating target globally
        # ------------------------
        self.leader_position=NavSatFix()
        self.leader_rel_alt = 0.0
        self.leader_heading=Float64()
        self.leader_vel = Vector3()
        self.leader_accel = Vector3()

        rospy.Subscriber('/ditto_1/mavros/global_position/global', NavSatFix, self.leader_glob_pos_callback)
        rospy.Subscriber('/ditto_1/mavros/global_position/rel_alt', Float64, self.leader_rel_alt_callback)
        rospy.Subscriber('/ditto_1/mavros/global_position/compass_hdg', Float64, self.leader_heading_callback)
        rospy.Subscriber('/ditto_1/mavros/global_position/local', Odometry, self.leader_glob_vel_callback)
        rospy.Subscriber('ditto_1/mavros/imu/data', Imu, self.leader_glob_acceleration_callback)

        self.glob_setpoint_pub = rospy.Publisher('/ditto_2/mavros/setpoint_raw/global',GlobalPositionTarget,queue_size=10)

        # Set the loop rate to 10 Hz       
        rate = rospy.Rate(10)


        while not rospy.is_shutdown():
            follow_distance = 10  # 1 meter behind
            left_distance = 0  # 1 meter to the left
            alt_distance = 5

            # ------------------------
            #Updating target globally
            # ------------------------
            # Extrapolate the leader's position by predicting its future position based on its velocity and acceleration
            dt = 0.10  # time interval in seconds 0.1 
            dx = self.leader_vel.x * dt + 0.5 * self.leader_accel.x * dt ** 2  # calculate distance traveled in x direction based on velocity and acceleration
            dy = self.leader_vel.y * dt + 0.5 * self.leader_accel.y * dt ** 2  # calculate distance traveled in y direction based on velocity and acceleration
            distance = math.sqrt(dx**2 + dy**2)  # calculate total distance traveled

            yaw = math.radians(90.0-float(self.leader_heading.data))

            ox = - follow_distance * math.cos(yaw) + distance * math.cos(yaw) - left_distance * math.sin(yaw)  # predict future x position of leader
            oy = - follow_distance * math.sin(yaw) + distance * math.sin(yaw) + left_distance * math.cos(yaw) # predict future y position of leader
            oz = 0 #+ self.leader_vel.z * dt + 0.5 * self.leader_accel.z * dt ** 2  # predict future z position of leader

            #converting the offsets wrt to leader global frame
            tlat,tlong,talt=pm.enu2geodetic(ox,oy,oz,self.leader_position.latitude,self.leader_position.longitude,self.leader_position.altitude)


            # Extrapolate the leader's velocity based on its accelerationS
            e = 0.00 #error compensation 0.05
            vx = self.leader_vel.x + self.leader_accel.x * dt + e
            vy = self.leader_vel.y + self.leader_accel.y * dt + e
            vz = self.leader_vel.z #+ self.leader_accel.z * dt
            target_vel = Vector3(vx, vy, vz)

            setpoint_msg = GlobalPositionTarget()
            setpoint_msg.header.stamp = rospy.Time.now()
            setpoint_msg.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_REL_ALT #height set from ground 
            setpoint_msg.type_mask = GlobalPositionTarget.IGNORE_AFX + GlobalPositionTarget.IGNORE_AFY + GlobalPositionTarget.IGNORE_AFZ + GlobalPositionTarget.IGNORE_YAW_RATE #+ GlobalPositionTarget.IGNORE_VZ + GlobalPositionTarget.IGNORE_VX  + GlobalPositionTarget.IGNORE_VY + GlobalPositionTarget.IGNORE_VZ
            setpoint_msg.header.frame_id = 'map'
            setpoint_msg.latitude = tlat
            setpoint_msg.longitude = tlong
            setpoint_msg.altitude = alt_distance + self.leader_rel_alt
            setpoint_msg.velocity =  target_vel
            setpoint_msg.acceleration_or_force = self.leader_accel
            setpoint_msg.yaw = math.radians(90.0-float(self.leader_heading.data))

            # self.glob_setpoint_pub.publish(setpoint_msg)


            print("Leader_pos:",self.leader_position)
            print("Leader_vel:",math.sqrt(self.leader_vel.x**2+self.leader_vel.y**2+self.leader_vel.z**2))

            target_pos=Vector3(tlat,tlong,6)
            print("Target_pos: ", target_pos)
            vel_f2=math.sqrt(target_vel.x**2+target_vel.y**2+target_vel.z**2)
            print("Target_vel_f2:",vel_f2)

            rate.sleep()


    # ------------------------
    #Updating target globally
    # ------------------------
    def leader_glob_pos_callback(self,msg):
        # get the global LLA of leader
        self.leader_position=msg

    def leader_rel_alt_callback(self,msg):
        self.leader_rel_alt = msg.data

    def leader_heading_callback(self,msg):
        # get the compass heading of leader
        self.leader_heading=msg

    def leader_glob_vel_callback(self,msg):
        # Get the local velocity of the leader drone
        self.leader_vel=msg.twist.twist.linear

    def leader_glob_acceleration_callback(self, msg):
        # Get the global acceleration of the leader drone
        self.leader_accel = msg.linear_acceleration
            
if __name__ == '__main__':
    try:
        controller = DroneController()
    except rospy.ROSInterruptException:
        pass