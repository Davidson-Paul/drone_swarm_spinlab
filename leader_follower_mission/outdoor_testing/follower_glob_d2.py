#!/usr/bin/env python3

import rospy
import math
import pymap3d as pm
from geometry_msgs.msg import Vector3
from mavros_msgs.msg import GlobalPositionTarget
from iq_gnc.msg import DroneStates
from nav_msgs.msg import Odometry

# vel_exceed=0

class DroneController:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('follower_controller')

        self.drone_name = '/ditto_2/'

        self.leader_states = DroneStates()
        # self.vel_actual = Vector3()

        rospy.Subscriber('/ditto_1/drone_states',DroneStates,self.leader_states_callback)
        # rospy.Subscriber(f'{self.drone_name}mavros/global_position/local',Odometry,self.actual_vel_callback)
        self.glob_setpoint_pub = rospy.Publisher(f'{self.drone_name}mavros/setpoint_raw/global',GlobalPositionTarget,queue_size=10)

        # Set the loop rate to 10 Hz       
        self.rate = rospy.Rate(5)

    def control_loop(self):

        droneid = 2

        follow_distance = 10  # 1 meter behind
        left_distance = 0  # 1 meter to the left
        alt_distance = 5 # 1 metre above

        while not rospy.is_shutdown():
            # Extrapolate the leader's position by predicting its future position based on its velocity and acceleration
            dt = 0.10  # time interval in seconds 0.1 
            dx = self.leader_states.linear_velocity.x * dt + 0.5 * self.leader_states.linear_acceleration.x * dt ** 2  # calculate distance traveled in x direction based on velocity and acceleration
            dy = self.leader_states.linear_velocity.y * dt + 0.5 * self.leader_states.linear_acceleration.y * dt ** 2  # calculate distance traveled in y direction based on velocity and acceleration
            distance = math.sqrt(dx**2 + dy**2)  # calculate total distance traveled

            yaw = math.radians(90.0-self.leader_states.heading)

            ox = - follow_distance * math.cos(yaw) + distance * math.cos(yaw) - left_distance * math.sin(yaw)  # predict future x position of leader
            oy = - follow_distance * math.sin(yaw) + distance * math.sin(yaw) + left_distance * math.cos(yaw) # predict future y position of leader
            oz = 0 #+ self.leader_states.linear_velocity.z * dt + 0.5 * self.leader_states.linear_acceleration.z * dt ** 2  # predict future z position of leader

            #converting the offsets wrt to leader global frame
            tlat,tlong,talt=pm.enu2geodetic(ox,oy,oz,self.leader_states.latitude,self.leader_states.longitude,self.leader_states.rel_altitude)


            # Extrapolate the leader's velocity based on its accelerationS
            e = 0.00 #error compensation 0.05
            vx = self.leader_states.linear_velocity.x + self.leader_states.linear_acceleration.x * dt + e
            vy = self.leader_states.linear_velocity.y + self.leader_states.linear_acceleration.y * dt + e
            vz = self.leader_states.linear_velocity.z #+ self.leader_states.linear_acceleration.z * dt
            target_vel = Vector3(vx, vy, vz)

            setpoint_msg = GlobalPositionTarget()
            setpoint_msg.header.stamp = rospy.Time.now()
            setpoint_msg.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_REL_ALT #height set from ground 
            setpoint_msg.type_mask = GlobalPositionTarget.IGNORE_AFX + GlobalPositionTarget.IGNORE_AFY + GlobalPositionTarget.IGNORE_AFZ + GlobalPositionTarget.IGNORE_YAW_RATE
            setpoint_msg.header.frame_id = 'map'
            setpoint_msg.latitude = tlat
            setpoint_msg.longitude = tlong
            setpoint_msg.altitude = alt_distance + self.leader_states.rel_altitude
            setpoint_msg.velocity =  target_vel
            # setpoint_msg.acceleration_or_force = self.leader_states.linear_acceleration
            setpoint_msg.yaw = yaw

            self.glob_setpoint_pub.publish(setpoint_msg)

            leader_pos=Vector3(self.leader_states.latitude,self.leader_states.longitude,self.leader_states.rel_altitude)
            print("Leader_pos:",leader_pos)
            print("leader_vel:",math.sqrt(self.leader_states.linear_velocity.x**2+self.leader_states.linear_velocity.y**2+self.leader_states.linear_velocity.z**2))

            target_pos=Vector3(setpoint_msg.latitude,setpoint_msg.longitude,setpoint_msg.altitude)
            print("Target_pos: ", target_pos)
            vel_f=math.sqrt(target_vel.x**2+target_vel.y**2+target_vel.z**2)
            print("Target_vel_f"+str(droneid)+":")
            print(target_vel)
            print(vel_f)



            # if(math.sqrt(self.vel_actual.x**2+self.vel_actual.y**2+self.vel_actual.z**2)>10):
            #     global vel_exceed
            #     vel_exceed+=1
            #     print("vel-limit exceeded")

            self.rate.sleep()

        # print('Exceeded vf'+str(droneid)+':',vel_exceed)

    def leader_states_callback(self, msg):
        # Get all leader's info
        self.leader_states = msg

    # def actual_vel_callback(self,msg):
    #     self.vel_actual = msg.twist.twist.linear

if __name__ == '__main__':
    try:
        controller = DroneController()
        controller.control_loop()
    except rospy.ROSInterruptException:
        # print('Exceeded vf'+str(controller.droneid)+':',vel_exceed)
        pass