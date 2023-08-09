#!/usr/bin/env python3

import rospy
import pymap3d as pm
from mavros_msgs.msg import PositionTarget, HomePosition
from tf.transformations import euler_from_quaternion
import math
from geometry_msgs.msg import Vector3
from iq_gnc.msg import DroneStates_local
import numpy as np

class DroneController:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('controller_D2_local')

        self.drone_name = rospy.get_namespace()

        self.leader_states = DroneStates_local()
        self.vel_actual = Vector3()

        rospy.Subscriber('/ditto_1/drone_states',DroneStates_local,self.leader_states_callback)
        self.glob_setpoint_pub = rospy.Publisher(f'{self.drone_name}mavros/setpoint_raw/local',PositionTarget,queue_size=10)

        # Subscribe to the home position of the leader drone
        rospy.Subscriber('ditto_1/mavros/home_position/home', HomePosition, self.leader_home_position_callback)
        rospy.Subscriber('ditto_2/mavros/home_position/home', HomePosition, self.follower_home_position_callback)

        # Wait for topics to become available
        rospy.wait_for_message('ditto_1/mavros/home_position/home', HomePosition)
        rospy.wait_for_message('ditto_2/mavros/home_position/home', HomePosition)

        # Set the loop rate to 10 Hz       
        self.rate = rospy.Rate(5)


        while not rospy.is_shutdown():
            follow_distance = 10  # 1 meter behind
            left_distance = 0  # 1 meter to the left
            alt_distance = 5

            # ------------------------
            #Updating target locally
            # ------------------------  
            # Get Leader Orient from quaternions
            leader_orientation  = self.leader_orient

            # calculating offset distance in global coordinates
            offset = pm.geodetic2enu(self.leader_home_pos.latitude, self.leader_home_pos.longitude, self.leader_home_pos.altitude, self.follower_home_pos.latitude, self.follower_home_pos.longitude, self.follower_home_pos.altitude)
            # print("inside offset: ", offset)
            (o_x, o_y, o_z) = offset

            print("offset: ", o_x, o_y)
            
            # Calculate Leader's heading in radians
            orientation_list = [leader_orientation.x, leader_orientation.y, leader_orientation.z, leader_orientation.w]
            (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

            # Extrapolate the leader's position by predicting its future position based on its velocity and acceleration
            dt = 0.10  # time interval in seconds 0.1 
            dx = self.leader_states.linear_velocity.x * dt + 0.5 * self.leader_states.linear_acceleration.x * dt ** 2  # calculate distance traveled in x direction based on velocity and acceleration
            dy = self.leader_states.linear_velocity.y * dt + 0.5 * self.leader_states.linear_acceleration.y * dt ** 2  # calculate distance traveled in y direction based on velocity and acceleration
            distance = math.sqrt(dx**2 + dy**2)  # calculate total distance traveled

            tx = self.leader_states.position.x - follow_distance * math.cos(yaw) + distance * math.cos(yaw) - left_distance * math.sin(yaw) + o_x  # predict future x position of leader
            ty = self.leader_states.position.y - follow_distance * math.sin(yaw) + distance * math.sin(yaw) + left_distance * math.cos(yaw) + o_y # predict future y position of leader
            tz = self.leader_states.position.z + alt_distance #+ self.leader_states.linear_velocity.z * dt + 0.5 * self.leader_states.linear_acceleration.z * dt ** 2  # predict future z position of leader
            target_pos = Vector3(tx, ty, tz)
            
            # Extrapolate the leader's velocity based on its accelerationS
            e = 0.00 #error compensation 0.05
            vx = self.leader_states.linear_velocity.x + self.leader_states.linear_acceleration.x * dt + e
            vy = self.leader_states.linear_velocity.y + self.leader_states.linear_acceleration.y * dt + e
            vz = self.leader_states.linear_velocity.z #+ self.leader_states.linear_acceleration.z * dt
            target_vel = Vector3(vx, vy, vz)

            # Update the setpoint message with the predicted position and velocity of the leader
            setpoint_msg = PositionTarget()
            setpoint_msg.header.stamp = rospy.Time.now()
            setpoint_msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            setpoint_msg.type_mask = PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + PositionTarget.IGNORE_YAW_RATE  #+ PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ
            setpoint_msg.header.frame_id = 'map'
            setpoint_msg.position = target_pos
            setpoint_msg.velocity =  target_vel
            setpoint_msg.acceleration_or_force = self.leader_states.linear_acceleration
            setpoint_msg.yaw = yaw

            # Publish the setpoint message
            # self.setpoint_pub.publish(setpoint_msg)

            print("leader_states.position:",self.leader_states.position)
            print("leader_states.linear_velocity:",math.sqrt(self.leader_states.linear_velocity.x**2+self.leader_states.linear_velocity.y**2+self.leader_states.linear_velocity.z**2))

            print("Target_pos: ", target_pos)
            vel_f2=math.sqrt(target_vel.x**2+target_vel.y**2+target_vel.z**2)
            print("Target_vel_f2:",vel_f2)

            self.rate.sleep()


    # ------------------------
    #Updating target locally
    # ------------------------
    def leader_states_callback(self, msg):
        # Get all leader's info
        self.leader_states = msg

    def leader_home_position_callback(self, msg):
        # Get the local position of the leader drone
        self.leader_home_pos = msg.geo

    def follower_home_position_callback(self, msg):
        # Get the local position of the follower drone
        self.follower_home_pos = msg.geo


if __name__ == '__main__':
    try:
        controller = DroneController()
    except rospy.ROSInterruptException:
        pass