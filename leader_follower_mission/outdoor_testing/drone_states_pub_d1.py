#!/usr/bin/env python3

from iq_gnc.msg import DroneStates
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Vector3
import rospy

class drone_states_pub:
    def __init__(self):
        rospy.init_node("drone_states_pub_d1")

        drone = '/ditto_1/'

        rospy.Subscriber(f'{drone}mavros/global_position/global', NavSatFix, self.glob_pos_callback)
        rospy.Subscriber(f'{drone}mavros/global_position/rel_alt', Float64, self.rel_alt_callback)
        rospy.Subscriber(f'{drone}mavros/global_position/compass_hdg', Float64, self.heading_callback)
        rospy.Subscriber(f'{drone}mavros/global_position/local',Odometry,self.glob_vel_callback)
        # rospy.Subscriber(f'{drone}mavros/imu/data', Imu, self.glob_acc_callback)

        self.topic_pub = rospy.Publisher(f'{drone}drone_states',DroneStates,queue_size=10)

        self.prev_time = rospy.Time.now().to_sec

        self.dt_sub = 0.1
        self.rate_sub = rospy.Rate(10)

        self.dt_pub = 0.2
        self.rate_pub = rospy.Rate(5)

        self.topic = DroneStates()
        self.prev_vel = Vector3()
        self.current_vel = Vector3()

        #Publishing loop
        while not rospy.is_shutdown():
            self.topic_pub.publish(self.topic)
            self.rate_pub.sleep()

            print(f'vel_{drone}:',(self.current_vel.x**2 + self.current_vel.y**2 + self.current_vel.z**2)**0.5)

    def glob_pos_callback(self,msg):
        self.topic.header = msg.header
        # self.topic.seq = msg.header.seq
        # self.topic.stamp = msg.header.stamp
        # self.topic.frame_id = msg.header.frame_id
        self.topic.latitude = msg.latitude
        self.topic.longitude = msg.longitude
    
    def rel_alt_callback(self,msg):
        self.topic.rel_altitude = msg.data

    def heading_callback(self,msg):
        self.topic.heading = msg.data

    def glob_vel_callback(self,msg):
        self.current_time = rospy.Time.now().to_sec
        self.current_vel = msg.twist.twist.linear

        #For every dt_sub seconds
        self.topic.linear_velocity = self.current_vel
        self.dt_sub = (self.current_time-self.prev_time)
        self.topic.linear_acceleration = Vector3( (self.current_vel.x-self.prev_vel.x)/self.dt_sub, 
                                                (self.current_vel.y-self.prev_vel.y)/self.dt_sub,
                                                (self.current_vel.z-self.prev_vel.z)/self.dt_sub )
        self.prev_vel = self.current_vel
        self.prev_time = self.current_time

    # def glob_acc_callback(self,msg):
    #     self.topic.linear_acceleration = msg.linear_acceleration

if __name__ == '__main__':
    try:
        fury = drone_states_pub()
    except rospy.ROSInterruptException:
        pass
