#!/usr/bin/env python3

import rospy
import pymap3d as pm
from mavros_msgs.msg import PositionTarget, HomePosition
from tf.transformations import euler_from_quaternion
import math
from geometry_msgs.msg import Vector3
from iq_gnc.msg import DroneStates_local
import numpy as np
from iq_gnc.msg import virtual_centre, VL_local

class DroneController:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('ext_vc_local')

        #Get parameters from launch file
        self.drone_name = rospy.get_namespace()
        droneid = rospy.get_param(f"{self.drone_name}extrapol/id")

        #Subscribing to each drone's own view of virtual centre(VC)
        rospy.Subscriber('/fury_1/virtual_centre', virtual_centre, self.f1_vc_callback)
        rospy.Subscriber('/fury_2/virtual_centre', virtual_centre, self.f2_vc_callback)
        rospy.Subscriber('/fury_3/virtual_centre', virtual_centre, self.f3_vc_callback)
        rospy.Subscriber('/fury_4/virtual_centre', virtual_centre, self.f4_vc_callback)
        rospy.Subscriber('/fury_5/virtual_centre', virtual_centre, self.f5_vc_callback)
        
        #Subscribing to VL topic (the virtual centre coordinates sent by the GCS to the swarm)
        rospy.Subscriber('/VL_states', VL_local, self.vl_callback)

        #Publishing to the drone's local setpoint
        self.drone_setpoint_pub = rospy.Publisher(f'{self.drone_name}mavros/setpoint_raw/local', PositionTarget, queue_size=10)

        self.drone_setpoint_msg = PositionTarget()

        #Initialising current drone's states of VC
        self.vc_p=np.array([0,0,6],dtype=np.float64)
        self.vc_v=np.array([0,0,0],dtype=np.float64)
        self.vc_acc=np.array([0,0,0],dtype=np.float64)
        self.vc_y = 0.0
        self.vc_w = 0.0

        #relative deviation wrt to VC position and orientation (maintaing formation)
        self.d1=np.array([0,0,0],dtype=np.float64)
        self.d2=np.array([-21.65,12.5,0],dtype=np.float64)
        self.d3=np.array([-21.65,-12.5,0],dtype=np.float64)
        self.d4=np.array([-43.3,25,0],dtype=np.float64)
        self.d5=np.array([-43.3,-25,0],dtype=np.float64)

        #Current drone's relative deviation
        self.d=np.array([0,0,0],dtype=np.float64)

        # Set the loop rate to 5 Hz       
        self.rate = rospy.Rate(5)
        self.dt = 0.2

        while not rospy.is_shutdown():

            #Setting current drone's deviation wrt to droneid in launch file
            if(self.droneid==1):
                self.vc_p=self.vc_pf1
                self.vc_v=self.vc_vf1
                self.vc_acc=self.vc_af1
                self.vc_y=self.vc_yf1
                self.vc_w=self.vc_wf1
                self.d=self.d1
            elif(self.droneid==2):
                self.vc_p=self.vc_pf2
                self.vc_v=self.vc_vf2
                self.vc_acc=self.vc_af2
                self.vc_y=self.vc_yf2
                self.vc_w=self.vc_wf2
                self.d=self.d2
            elif(self.droneid==3):
                self.vc_p=self.vc_pf3
                self.vc_v=self.vc_vf3
                self.vc_acc=self.vc_af3
                self.vc_y=self.vc_yf3
                self.vc_w=self.vc_wf3
                self.d=self.d3
            elif(self.droneid==4):
                self.vc_p=self.vc_pf4
                self.vc_v=self.vc_vf4
                self.vc_acc=self.vc_af4
                self.vc_y=self.vc_yf4
                self.vc_w=self.vc_wf4
                self.d=self.d4
            elif(self.droneid==5):
                self.vc_p=self.vc_pf5
                self.vc_v=self.vc_vf5
                self.vc_acc=self.vc_af5
                self.vc_y=self.vc_yf5
                self.vc_w=self.vc_wf5
                self.d=self.d5   

            #Get local position of the drone wrt virtual centre
            r = self.vc_p + np.matmul(np.array([[np.cos(self.vc_y),-np.sin(self.vc_y),0],[np.sin(self.vc_y),np.cos(self.vc_y),0],[0,0,1]]),self.d) - self.d #adding offset of 'self.d' for global origin to drone-starting origin 

            #Extrapolating
            dx = self.vc_v[0]*self.dt + 0.5*self.vc_acc[0]*self.dt**2
            dy = self.vc_v[1]*self.dt + 0.5*self.vc_acc[1]*self.dt**2
            dist = math.sqrt(dx**2 + dy**2)

            r_ext = r+np.array([dist*np.cos(self.vc_y),dist*np.sin(self.vc_y),0])

            v_ext = self.vc_v + self.vc_acc*self.dt

            # Update the setpoint message with the predicted position and velocity of the leader
            self.drone_setpoint_msg.header.stamp = rospy.Time.now()
            self.drone_setpoint_msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            self.drone_setpoint_msg.type_mask = PositionTarget.IGNORE_YAW_RATE + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ
            self.drone_setpoint_msg.header.frame_id = 'map'
            self.drone_setpoint_msg.position = Vector3(r_ext[0],r_ext[1],r_ext[2])
            self.drone_setpoint_msg.velocity = Vector3(v_ext[0],v_ext[1],v_ext[2])
            self.drone_setpoint_msg.yaw = self.vc_y

            print('setpoint'+str(self.droneid)+':',r)

            # Publish the setpoint message
            self.drone_setpoint_pub.publish(self.drone_setpoint_msg)

            self.rate.sleep()


    #VC Callback functions
    def f1_vc_callback(self,msg):
        self.vc_pf1=np.array([msg.position.x, msg.position.y, msg.position.z])
        self.vc_vf1=np.array([msg.linear_velocity.x, msg.linear_velocity.y, msg.linear_velocity.z])
        self.vc_af1=np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        self.vc_yf1=msg.yaw
        self.vc_wf1=msg.yaw_rate
    
    def f2_vc_callback(self,msg):
        self.vc_pf2=np.array([msg.position.x, msg.position.y, msg.position.z])
        self.vc_vf2=np.array([msg.linear_velocity.x, msg.linear_velocity.y, msg.linear_velocity.z])
        self.vc_af2=np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        self.vc_yf2=msg.yaw
        self.vc_wf2=msg.yaw_rate

    def f3_vc_callback(self,msg):
        self.vc_pf3=np.array([msg.position.x, msg.position.y, msg.position.z])
        self.vc_vf3=np.array([msg.linear_velocity.x, msg.linear_velocity.y, msg.linear_velocity.z])
        self.vc_af3=np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        self.vc_yf3=msg.yaw
        self.vc_wf3=msg.yaw_rate

    def f4_vc_callback(self,msg):
        self.vc_pf4=np.array([msg.position.x, msg.position.y, msg.position.z])
        self.vc_vf4=np.array([msg.linear_velocity.x, msg.linear_velocity.y, msg.linear_velocity.z])
        self.vc_af4=np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        self.vc_yf4=msg.yaw
        self.vc_wf4=msg.yaw_rate

    def f5_vc_callback(self,msg):
        self.vc_pf5=np.array([msg.position.x, msg.position.y, msg.position.z])
        self.vc_vf5=np.array([msg.linear_velocity.x, msg.linear_velocity.y, msg.linear_velocity.z])
        self.vc_af5=np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        self.vc_yf5=msg.yaw
        self.vc_wf5=msg.yaw_rate

    def vl_callback(self, msg):
        self.pr = np.array([msg.position.x,msg.position.y,msg.position.z])
        self.vr = np.array([msg.linear_velocity.x,msg.linear_velocity.y,msg.linear_velocity.z])
        self.ar = np.array([msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z])
        self.yr = msg.yaw
        self.wr = msg.yaw_rate


if __name__ == '__main__':
    try:
        controller = DroneController()
    except rospy.ROSInterruptException:
        pass