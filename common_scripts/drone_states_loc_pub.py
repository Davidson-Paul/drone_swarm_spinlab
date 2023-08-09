#!/usr/bin/env python3

# This script publishes the DroneStates_local message

from iq_gnc.msg import DroneStates_local
from geometry_msgs.msg import Vector3, PoseStamped, TwistStamped
from tf.transformations import euler_from_quaternion
import rospy

class drone_states_loc_pub:
    def __init__(self):
        rospy.init_node("drone_states_loc_pub_all")

        #Getting drone name from launch file
        drone = rospy.get_namespace()

        #Getting fused local states from pre-defined mavros topics
        rospy.Subscriber(f'{drone}mavros/local_position/pose', PoseStamped, self.pos_callback)
        rospy.Subscriber(f'{drone}mavros/local_position/velocity_local', TwistStamped, self.vel_callback)

        #Publishing to the custom topic
        self.topic_pub = rospy.Publisher(f'{drone}drone_states_local',DroneStates_local,queue_size=10)

        #Considering that we are getting mavros topics at 20Hz rate
        self.dt_sub = 0.05
        self.rate_sub = rospy.Rate(20)

        #Publishing at 5Hz rate
        self.dt_pub = 0.2
        self.rate_pub = rospy.Rate(5)

        self.topic = DroneStates_local()
        self.prev_vel = Vector3()
        self.current_vel = Vector3()

        #Publishing loop
        while not rospy.is_shutdown():
            self.topic_pub.publish(self.topic)
            self.rate_pub.sleep()

            print(f'vel_{drone}:',(self.current_vel.x**2 + self.current_vel.y**2 + self.current_vel.z**2)**0.5)

    #Callback functions
    def pos_callback(self,msg):
        self.topic.header = msg.header
        self.topic.position = msg.pose.position
        (roll, pitch, self.topic.yaw) = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])

    def vel_callback(self,msg):
        self.current_vel = msg.twist.linear

        #For every dt_sub seconds
        self.topic.linear_velocity = self.current_vel

        #Calculation of acceleration
        self.topic.linear_acceleration = Vector3( (self.current_vel.x-self.prev_vel.x)/self.dt_sub, 
                                                (self.current_vel.y-self.prev_vel.y)/self.dt_sub,
                                                (self.current_vel.z-self.prev_vel.z)/self.dt_sub )
        self.prev_vel = self.current_vel


if __name__ == '__main__':
    try:
        fury = drone_states_loc_pub()
    except rospy.ROSInterruptException:
        pass