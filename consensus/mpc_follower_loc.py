#!/usr/bin/env python3

#Script for a drone to follow/track its VC using MPC
#DI model is considered in each axis and so acceleration input is given to the drone

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
        rospy.init_node('mpc_follower_local')

        #Get parameters from launch file
        # self.drone_name = rospy.get_namespace()
        # self.droneid = rospy.get_param(f"{self.drone_name}mpc_follower_local/id")
        self.drone_name = '/fury_1/'
        self.droneid = 1

        #Subscribing to each drone's own view of virtual centre(VC)
        rospy.Subscriber('/fury_1/virtual_centre', virtual_centre, self.f1_vc_callback)
        rospy.Subscriber('/fury_2/virtual_centre', virtual_centre, self.f2_vc_callback)
        rospy.Subscriber('/fury_3/virtual_centre', virtual_centre, self.f3_vc_callback)
        rospy.Subscriber('/fury_4/virtual_centre', virtual_centre, self.f4_vc_callback)
        rospy.Subscriber('/fury_5/virtual_centre', virtual_centre, self.f5_vc_callback)

        #Subscribing to each drones' states
        rospy.Subscriber('/fury_1/drone_states_local', DroneStates_local, self.f1_callback)
        rospy.Subscriber('/fury_2/drone_states_local', DroneStates_local, self.f2_callback)
        rospy.Subscriber('/fury_3/drone_states_local', DroneStates_local, self.f3_callback)
        rospy.Subscriber('/fury_4/drone_states_local', DroneStates_local, self.f4_callback)
        rospy.Subscriber('/fury_5/drone_states_local', DroneStates_local, self.f5_callback)

        #Publishing to the drone's local setpoint
        self.drone_setpoint_pub = rospy.Publisher(f'{self.drone_name}mavros/setpoint_raw/local', PositionTarget, queue_size=10)

        self.drone_setpoint_msg = PositionTarget()

        #Initialising each drone's own position of VC
        self.vc_pf1=np.array([0,0,6],dtype=np.float64)
        self.vc_pf2=np.array([0,0,6],dtype=np.float64)
        self.vc_pf3=np.array([0,0,6],dtype=np.float64)
        self.vc_pf4=np.array([0,0,6],dtype=np.float64)
        self.vc_pf5=np.array([0,0,6],dtype=np.float64)
        #Initialising each drone's own velocity of VC
        self.vc_vf1=np.array([0,0,0],dtype=np.float64)
        self.vc_vf2=np.array([0,0,0],dtype=np.float64)
        self.vc_vf3=np.array([0,0,0],dtype=np.float64)
        self.vc_vf4=np.array([0,0,0],dtype=np.float64)
        self.vc_vf5=np.array([0,0,0],dtype=np.float64)
        #Initialising each drone's own acc of VC
        self.vc_af1=np.array([0,0,0],dtype=np.float64)
        self.vc_af2=np.array([0,0,0],dtype=np.float64)
        self.vc_af3=np.array([0,0,0],dtype=np.float64)
        self.vc_af4=np.array([0,0,0],dtype=np.float64)
        self.vc_af5=np.array([0,0,0],dtype=np.float64)
        #Initialising each drone's own yaw of VC
        self.vc_yf1=0.0
        self.vc_yf2=0.0
        self.vc_yf3=0.0
        self.vc_yf4=0.0
        self.vc_yf5=0.0
        #Initialising each drone's own yaw_rate of VC
        self.vc_wf1=0.0
        self.vc_wf2=0.0
        self.vc_wf3=0.0
        self.vc_wf4=0.0
        self.vc_wf5=0.0

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

        #Initialising
        self.pf1=np.empty((3),dtype=np.float64)
        self.pf2=np.empty((3),dtype=np.float64)
        self.pf3=np.empty((3),dtype=np.float64)
        self.pf4=np.empty((3),dtype=np.float64)
        self.pf5=np.empty((3),dtype=np.float64)

        self.vf1=np.empty((3),dtype=np.float64)
        self.vf2=np.empty((3),dtype=np.float64)
        self.vf3=np.empty((3),dtype=np.float64)
        self.vf4=np.empty((3),dtype=np.float64)
        self.vf5=np.empty((3),dtype=np.float64)

        self.af1=np.empty((3),dtype=np.float64)
        self.af2=np.empty((3),dtype=np.float64)
        self.af3=np.empty((3),dtype=np.float64)
        self.af4=np.empty((3),dtype=np.float64)
        self.af5=np.empty((3),dtype=np.float64)

        #Initialising current drone's states
        self.p=np.empty((3),dtype=np.float64)
        self.v=np.empty((3),dtype=np.float64)
        acc=np.empty((3),dtype=np.float64)

        self.I=np.eye(2)
        self.Z=np.array([[0],[0]])

        #double integrator system model, with acc as input
        self.A=np.array([[1,0.2],[0,1]])
        self.B=np.array([[0],[1]])
        self.C=self.I
        self.Nc=6
        self.Np=6

        #system parameters
        self.F=self.A
        self.Rs=self.I
        self.phi=np.hstack((self.B,self.Z,self.Z,self.Z,self.Z,self.Z))
        for i in range(self.Np-1):
            self.F=np.vstack((self.F,np.linalg.matrix_power(self.A,(i+2))))
            self.Rs=np.vstack((self.Rs,self.I))
            
            temp1=np.linalg.matrix_power(self.A,i+1)@self.B
            for j in range (i+1):
                if(j<self.Nc-1):
                    temp1=np.hstack((temp1,np.linalg.matrix_power(self.A,i-j)@self.B))
            for k in range (self.Nc-i-2):
                temp1=np.hstack((temp1,self.Z))

            self.phi=np.vstack((self.phi,temp1))

        self.SP = np.array([[0],[0]])      

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

                self.p=self.pf1
                self.v=self.vf1
            elif(self.droneid==2):
                self.vc_p=self.vc_pf2
                self.vc_v=self.vc_vf2
                self.vc_acc=self.vc_af2
                self.vc_y=self.vc_yf2
                self.vc_w=self.vc_wf2
                self.d=self.d2

                self.p=self.pf2
                self.v=self.vf2
            elif(self.droneid==3):
                self.vc_p=self.vc_pf3
                self.vc_v=self.vc_vf3
                self.vc_acc=self.vc_af3
                self.vc_y=self.vc_yf3
                self.vc_w=self.vc_wf3
                self.d=self.d3

                self.p=self.pf3
                self.v=self.vf3
            elif(self.droneid==4):
                self.vc_p=self.vc_pf4
                self.vc_v=self.vc_vf4
                self.vc_acc=self.vc_af4
                self.vc_y=self.vc_yf4
                self.vc_w=self.vc_wf4
                self.d=self.d4

                self.p=self.pf4
                self.v=self.vf4
            elif(self.droneid==5):
                self.vc_p=self.vc_pf5
                self.vc_v=self.vc_vf5
                self.vc_acc=self.vc_af5
                self.vc_y=self.vc_yf5
                self.vc_w=self.vc_wf5
                self.d=self.d5

                self.p=self.pf5
                self.v=self.vf5


            #Calculating the optimised acc input
            Xx=np.array([[self.p[0]-self.d[0]],[self.v[0]]])
            Rx=np.array([[self.vc_p[0]],[self.vc_v[0]]])
            Ex=Xx-Rx
            Ux=np.linalg.inv(self.phi.T@self.phi)@self.phi.T@(self.Rs@self.SP-self.F@Ex)
            print(Ux)
            ax=Ux[0]+self.vc_acc[0]
            print(ax)

            Xy=np.array([[self.p[1]-self.d[1]],[self.v[1]]])
            Ry=np.array([[self.vc_p[1]],[self.vc_v[1]]])
            Ey=Xy-Ry
            Uy=np.linalg.inv(self.phi.T@self.phi)@self.phi.T@(self.Rs@self.SP-self.F@Ey)
            ay=Uy[0]+self.vc_acc[1]

            Xz=np.array([[self.p[2]-self.d[2]],[self.v[2]]])
            Rz=np.array([[self.vc_p[2]],[self.vc_v[2]]])
            Ez=Xz-Rz
            Uz=np.linalg.inv(self.phi.T@self.phi)@self.phi.T@(self.Rs@self.SP-self.F@Ez)
            az=Uz[0]+self.vc_acc[2]

            # Update the setpoint message with the predicted position and velocity of the leader
            self.drone_setpoint_msg.header.stamp = rospy.Time.now()
            self.drone_setpoint_msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            self.drone_setpoint_msg.type_mask = PositionTarget.IGNORE_YAW_RATE + PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ + PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ
            self.drone_setpoint_msg.header.frame_id = 'map'
            self.drone_setpoint_msg.acceleration_or_force = Vector3(ax,ay,az)
            self.drone_setpoint_msg.yaw = self.vc_y

            print("error:",Ex,Ey,Ez)
            print('vc_acc'+str(self.droneid)+':',self.vc_acc)
            print('acc'+str(self.droneid)+':',np.array([ax,ay,az]))

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

    #Offsets are added to each drone's local position, inorder to change to global frame
    def f1_callback(self,msg):
        self.pf1=np.array([msg.position.x + 0 ,msg.position.y + 0 ,msg.position.z])
        self.vf1=np.array([msg.linear_velocity.x, msg.linear_velocity.y, msg.linear_velocity.z])
        self.af1=np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
    
    def f2_callback(self,msg):
        self.pf2=np.array([msg.position.x + -21.65 ,msg.position.y + 12.5 ,msg.position.z])
        self.vf2=np.array([msg.linear_velocity.x, msg.linear_velocity.y, msg.linear_velocity.z])
        self.af2=np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])

    def f3_callback(self,msg):
        self.pf3=np.array([msg.position.x + -21.65 ,msg.position.y + -12.5,msg.position.z])
        self.vf3=np.array([msg.linear_velocity.x, msg.linear_velocity.y, msg.linear_velocity.z])
        self.af3=np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])

    def f4_callback(self,msg):
        self.pf4=np.array([msg.position.x + -43.3,msg.position.y + 25,msg.position.z])
        self.vf4=np.array([msg.linear_velocity.x, msg.linear_velocity.y, msg.linear_velocity.z])
        self.af4=np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])

    def f5_callback(self,msg):
        self.pf5=np.array([msg.position.x + -43.3,msg.position.y + -25,msg.position.z])
        self.vf5=np.array([msg.linear_velocity.x, msg.linear_velocity.y, msg.linear_velocity.z])
        self.af5=np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])


if __name__ == '__main__':
    try:
        controller = DroneController()
    except rospy.ROSInterruptException:
        pass