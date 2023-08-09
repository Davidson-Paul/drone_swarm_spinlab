#!/usr/bin/env python3

#Formation Control script
#Using Single Integrator Consensus Tracking Algorithm
#Formation is maintained without considering orientation
#Setpoints are published in local coordinates

import rospy
from mavros_msgs.msg import PositionTarget
from math import sin, cos
import math
import time
# from pygame import Vector3
from geometry_msgs.msg import Vector3
from iq_gnc.msg import DroneStates_local
import numpy as np
from iq_gnc.msg import VL_local

class si_ct_controller:
    def __init__(self):
        rospy.init_node("si_CT")

        #Getting parameters from launch file
        self.drone_name = rospy.get_namespace()
        droneid = rospy.get_param(f"{self.drone_name}si_ct_local/id")

        #Subscribing to each drones' states
        rospy.Subscriber('/fury_1/drone_states_local', DroneStates_local, self.f1_callback)
        rospy.Subscriber('/fury_2/drone_states_local', DroneStates_local, self.f2_callback)
        rospy.Subscriber('/fury_3/drone_states_local', DroneStates_local, self.f3_callback)
        rospy.Subscriber('/fury_4/drone_states_local', DroneStates_local, self.f4_callback)
        rospy.Subscriber('/fury_5/drone_states_local', DroneStates_local, self.f5_callback)
        
        #Subscribing to Virtual Leader (VL) topic
        rospy.Subscriber('/VL_states', VL_local, self.vl_callback)

        self.setpoint_pub = rospy.Publisher(f'{self.drone_name}mavros/setpoint_raw/local', PositionTarget, queue_size=10)

        self.pf1=np.empty((3),dtype=np.float64)
        self.pf2=np.empty((3),dtype=np.float64)
        self.pf3=np.empty((3),dtype=np.float64)
        self.pf4=np.empty((3),dtype=np.float64)
        self.pf5=np.empty((3),dtype=np.float64)
        self.pr=np.empty((3),dtype=np.float64)

        #Initialising current drone's position
        p=np.empty((3),dtype=np.float64)

        self.vf1=np.empty((3),dtype=np.float64)
        self.vf2=np.empty((3),dtype=np.float64)
        self.vf3=np.empty((3),dtype=np.float64)
        self.vf4=np.empty((3),dtype=np.float64)
        self.vf5=np.empty((3),dtype=np.float64)
        self.vr=np.empty((3),dtype=np.float64)

        #Initialising current drone's velocity
        v=np.empty((3),dtype=np.float64)

        # self.af1=np.empty((3),dtype=np.float64)
        # self.af2=np.empty((3),dtype=np.float64)
        # self.af3=np.empty((3),dtype=np.float64)
        # self.af4=np.empty((3),dtype=np.float64)
        # self.af5=np.empty((3),dtype=np.float64)
        # self.ar=np.empty((3),dtype=np.float64)

        # #Initialising current drone's acceleration
        # acc=np.empty((3),dtype=np.float64)

        #The script is made to run at specific timestamps with 5Hz rate
        rate = rospy.Rate(1000)
        self.dt = 0.2

        a=21.65
        b=12.5

        t=0

        while not rospy.is_shutdown():
            current_time = rospy.Time.now()  # Get current time

            # Synchronize publishing with a specific time interval
            # Timing for 5 Hz rate - blocks runs at timestamp: .0, .2, .4, .6, .8
            if (current_time.to_sec()*5) % 1.0 < 0.05:

                #Getting the adjacency matrix
                adj_mat = np.loadtxt('/home/davidson/catkin_ws/src/iq_gnc/adjacency_matrix.txt', dtype=int)
                
                #Considering formation with no orientation
                yaw=0
                yaw_rate=0

                #relative deviation wrt to VL position (maintaing formation)
                d1=np.array([0,0,0])
                d2=np.array([-a*cos(yaw)-b*sin(yaw),-a*sin(yaw)+b*cos(yaw),0])
                d3=np.array([-a*cos(yaw)+b*sin(yaw),-a*sin(yaw)-b*cos(yaw),0])
                d4=np.array([-2*a*cos(yaw)-2*b*sin(yaw),-2*a*sin(yaw)+2*b*cos(yaw),0])
                d5=np.array([-2*a*cos(yaw)+2*b*sin(yaw),-2*a*sin(yaw)-2*b*cos(yaw),0])

                # #derivative of relative deviation
                # dd1=np.array([0 , 0 , 0])
                # dd2=np.array([a*yaw_rate*sin(yaw)-b*yaw_rate*cos(yaw) , -a*yaw_rate*cos(yaw)-b*yaw_rate*sin(yaw) , 0])
                # dd3=np.array([a*yaw_rate*sin(yaw)+b*yaw_rate*cos(yaw) , -a*yaw_rate*cos(yaw)+b*yaw_rate*sin(yaw) , 0])
                # dd4=np.array([2*a*yaw_rate*sin(yaw)-2*b*yaw_rate*cos(yaw) , -2*a*yaw_rate*cos(yaw)-2*b*yaw_rate*sin(yaw) , 0])
                # dd5=np.array([2*a*yaw_rate*sin(yaw)+2*b*yaw_rate*cos(yaw) , -2*a*yaw_rate*cos(yaw)+2*b*yaw_rate*sin(yaw) , 0])

                #assignning adjacency matrix entries for each agent
                A = adj_mat[(droneid-1),:]

                if(droneid==1):
                    p=self.pf1
                    v=self.vf1
                    d=d1
                elif(droneid==2):
                    p=self.pf2
                    v=self.vf2
                    d=d2
                elif(droneid==3):
                    p=self.pf3
                    v=self.vf3
                    d=d3
                elif(droneid==4):
                    p=self.pf4
                    v=self.vf4
                    d=d4
                elif(droneid==5):
                    p=self.pf5
                    v=self.vf5
                    d=d5

                n=np.sum(A)

                a0=A[0]
                a1=A[1]
                a2=A[2]
                a3=A[3]
                a4=A[4]
                a5=A[5]

                #DI consensus alg
                vel = (1/n)*(a0*(self.vf1-0.1*((p-d)-(self.pf1-d1)))
                            +a1*(self.vf2-0.1*((p-d)-(self.pf2-d2)))
                            +a2*(self.vf3-0.1*((p-d)-(self.pf3-d3)))
                            +a3*(self.vf4-0.1*((p-d)-(self.pf4-d4)))
                            +a4*(self.vf5-0.1*((p-d)-(self.pf5-d5)))
                            +a5*(self.vr-0.1*((p-d)-self.pr)))
                
                print('t:',t)
                print("vr:",self.vr)
                print("A:",A)
                print('v'+str(droneid)+':',vel)

                setpoint_msg = PositionTarget()
                setpoint_msg.header.stamp = rospy.Time.now()
                setpoint_msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
                setpoint_msg.type_mask = PositionTarget.IGNORE_YAW_RATE + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ
                setpoint_msg.header.frame_id = 'map'
                setpoint_msg.position = Vector3(0,0,0)
                setpoint_msg.velocity = Vector3(vel[0],vel[1],vel[2])
                setpoint_msg.acceleration_or_force = Vector3(0,0,0)
                setpoint_msg.yaw = yaw

                # print(setpoint_msg)

                # Publish the setpoint message after some time buffer
                time.sleep(0.09)
                self.setpoint_pub.publish(setpoint_msg)
                t+=self.dt
                time.sleep(0.09)

            rate.sleep()


    #Callback functions
    #Offsets are added to each drone's local position, inorder to change to global frame
    def f1_callback(self,msg):
        self.pf1=np.array([msg.position.x + 0 ,msg.position.y + 0])
        self.vf1=np.array([msg.linear_velocity.x, msg.linear_velocity.y])
        # self.af1=np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
    
    def f2_callback(self,msg):
        self.pf2=np.array([msg.position.x + -21.65 ,msg.position.y + 12.5])
        self.vf2=np.array([msg.linear_velocity.x, msg.linear_velocity.y])
        # self.af2=np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])

    def f3_callback(self,msg):
        self.pf3=np.array([msg.position.x + -21.65 ,msg.position.y + -12.5])
        self.vf3=np.array([msg.linear_velocity.x, msg.linear_velocity.y])
        # self.af3=np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])

    def f4_callback(self,msg):
        self.pf4=np.array([msg.position.x + -43.3,msg.position.y + 25])
        self.vf4=np.array([msg.linear_velocity.x, msg.linear_velocity.y])
        # self.af4=np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])

    def f5_callback(self,msg):
        self.pf5=np.array([msg.position.x + -43.3,msg.position.y + -25])
        self.vf5=np.array([msg.linear_velocity.x, msg.linear_velocity.y])
        # self.af5=np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])

    #Comment if VL data is used directly
    def vl_callback(self, msg):
        self.pr = np.array([msg.position.x,msg.position.y])
        self.vr = np.array([msg.linear_velocity.x,msg.linear_velocity.y])
        # self.ar = np.array([msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z])



if __name__ == '__main__':
    try:
        fsi = si_ct_controller()
    except rospy.ROSInterruptException:
        pass
