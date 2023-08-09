#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import PositionTarget
from math import sin, cos
import math
# from pygame import Vector3
from geometry_msgs.msg import Vector3
from iq_gnc.msg import DroneStates_local
import numpy as np
from iq_gnc.msg import VL_local

class di_ext_controller:
    def __init__(self):
        rospy.init_node("di_ext_vl")

        self.drone_name = rospy.get_namespace()
        droneid = rospy.get_param(f"{self.drone_name}di_ext_vl/id")

        rospy.Subscriber('/fury_1/drone_states_local', DroneStates_local, self.f1_callback)
        rospy.Subscriber('/fury_2/drone_states_local', DroneStates_local, self.f2_callback)
        rospy.Subscriber('/fury_3/drone_states_local', DroneStates_local, self.f3_callback)
        rospy.Subscriber('/fury_4/drone_states_local', DroneStates_local, self.f4_callback)
        rospy.Subscriber('/fury_5/drone_states_local', DroneStates_local, self.f5_callback)
        #Subscribing to VL topic
        #Check below to to use VL data directly without separate topic
        rospy.Subscriber('/VL_states', VL_local, self.vl_callback)

        self.setpoint_pub = rospy.Publisher(f'{self.drone_name}mavros/setpoint_raw/local', PositionTarget, queue_size=10)

        self.pf1=np.empty((3),dtype=np.float64)
        self.pf2=np.empty((3),dtype=np.float64)
        self.pf3=np.empty((3),dtype=np.float64)
        self.pf4=np.empty((3),dtype=np.float64)
        self.pf5=np.empty((3),dtype=np.float64)
        self.pfr=np.empty((3),dtype=np.float64)

        p=np.empty((3),dtype=np.float64)

        self.vf1=np.empty((3),dtype=np.float64)
        self.vf2=np.empty((3),dtype=np.float64)
        self.vf3=np.empty((3),dtype=np.float64)
        self.vf4=np.empty((3),dtype=np.float64)
        self.vf5=np.empty((3),dtype=np.float64)
        self.vfr=np.empty((3),dtype=np.float64)

        v=np.empty((3),dtype=np.float64)

        self.af1=np.empty((3),dtype=np.float64)
        self.af2=np.empty((3),dtype=np.float64)
        self.af3=np.empty((3),dtype=np.float64)
        self.af4=np.empty((3),dtype=np.float64)
        self.af5=np.empty((3),dtype=np.float64)
        self.afr=np.empty((3),dtype=np.float64)

        acc=np.empty((3),dtype=np.float64)

        rate = rospy.Rate(5)

        a=21.65
        b=12.5

        amp=20

        t=0
        sim_time=0
        shuffle_time=0
        dt=0.2
        fc=1/160
        yaw_old=0


        # arr = np.array([0, 0, 1, 1])
        #adjacency matrix entries
        # A=np.array([0,1,1,0,0,0])   
        adj_mat = np.loadtxt('/home/davidson/catkin_ws/src/iq_gnc/adjacency_matrix.txt', dtype=int)

        while not rospy.is_shutdown():

            ##Uncomment this to use VL data directly, without subscribing to separate VL topic
            # if(sim_time<0):
            #     xr=np.array([sim_time,amp*sin(2*math.pi*fc*t),5])
            #     vr=np.array([1,amp*2*math.pi*fc*cos(2*math.pi*fc*t),0])
            #     ar=np.array([0,-amp*(2*math.pi*fc)**2*sin(2*math.pi*fc*t),0])
            # elif(sim_time<0):
            #     xr=np.array([sim_time,0,5])
            #     vr=np.array([1,0,0])
            #     ar=np.array([0,0,0])
            # else:
            #     xr=np.array([0,0,5])
            #     vr=np.array([0,0,0])
            #     ar=np.array([0,0,0])


            yaw=0
            yaw_rate=0

            #relative deviation wrt to VL position (maintaing formation)
            d1=np.array([0,0,0])
            d2=np.array([-a*cos(yaw)-b*sin(yaw),-a*sin(yaw)+b*cos(yaw),0])
            d3=np.array([-a*cos(yaw)+b*sin(yaw),-a*sin(yaw)-b*cos(yaw),0])
            d4=np.array([-2*a*cos(yaw)-2*b*sin(yaw),-2*a*sin(yaw)+2*b*cos(yaw),0])
            d5=np.array([-2*a*cos(yaw)+2*b*sin(yaw),-2*a*sin(yaw)-2*b*cos(yaw),0])

            #derivative of relative deviation
            dd1=np.array([0 , 0 , 0])
            dd2=np.array([a*yaw_rate*sin(yaw)-b*yaw_rate*cos(yaw) , -a*yaw_rate*cos(yaw)-b*yaw_rate*sin(yaw) , 0])
            dd3=np.array([a*yaw_rate*sin(yaw)+b*yaw_rate*cos(yaw) , -a*yaw_rate*cos(yaw)+b*yaw_rate*sin(yaw) , 0])
            dd4=np.array([2*a*yaw_rate*sin(yaw)-2*b*yaw_rate*cos(yaw) , -2*a*yaw_rate*cos(yaw)-2*b*yaw_rate*sin(yaw) , 0])
            dd5=np.array([2*a*yaw_rate*sin(yaw)+2*b*yaw_rate*cos(yaw) , -2*a*yaw_rate*cos(yaw)+2*b*yaw_rate*sin(yaw) , 0])

            # #change who receives VL data every 5s
            # if(t<5 and t>=0):
            #     A[5]=(droneid==1)
            # elif(t<10 and t>=5):                
            #     A[5]=(droneid==2)
            # elif(t<15 and t>=10):                
            #     A[5]=(droneid==3)
            # elif(t<20 and t>=15):                
            #     A[5]=(droneid==4)
            # elif(t<25 and t>=20):                
            #     A[5]=(droneid==5)

            # A[5]=(droneid==1) #only f1 gets message from VL 

            #assignning adjacency matrix entries for each agent
            A = adj_mat[(droneid-1),:]

            if(droneid==1):
                p=self.pf1
                v=self.vf1
                d=d1
                # A[0:5]=[0,arr[0],arr[1],arr[2],arr[3]]
            elif(droneid==2):
                p=self.pf2
                v=self.vf2
                d=d2
                # A[0:5]=[arr[0],0,arr[1],arr[2],arr[3]]
            elif(droneid==3):
                p=self.pf3
                v=self.vf3
                d=d3
                # A[0:5]=[arr[0],arr[1],0,arr[2],arr[3]]
            elif(droneid==4):
                p=self.pf4
                v=self.vf4
                d=d4
                # A[0:5]=[arr[0],arr[1],arr[2],0,arr[3]]
            elif(droneid==5):
                p=self.pf5
                v=self.vf5
                d=d5
                # A[0:5]=[arr[0],arr[1],arr[2],arr[3],0]

            n=np.sum(A)

            # print("p:",p)
            # print("v:",v)
            # print(p-v)

            a0=A[0]
            a1=A[1]
            a2=A[2]
            a3=A[3]
            a4=A[4]
            a5=A[5]
            # print(a0*( self.af1 -0.1*((p-d1)-(self.pf1-d1)) - 1*(v-self.vf1) )
            #             + a1*( self.af2 -0.1*((p-d1)-(self.pf2-d2)) - 1*(v-self.vf2) ))

            #Extrapolation
            dxr = self.vfr[0]*dt + 0.5 * self.afr[0]* dt**2
            dyr = self.vfr[1]*dt + 0.5 * self.afr[1]* dt**2
            distr = math.sqrt(dxr**2 + dyr**2)
            pr = np.array([self.pfr[0] + distr*math.cos(yaw), self.pfr[1] + distr*math.sin(yaw), self.pfr[2]])
            vr = np.array([self.vfr[0] + self.afr[0]*dt, self.vfr[1] + self.afr[1]*dt, self.vfr[2]])

            #DI consensus alg
            acc = (1/n)*(a0*(self.af1-0.1*((p-d)-(self.pf1-d1))-1.2*(v-self.vf1))
                         +a1*(self.af2-0.1*((p-d)-(self.pf2-d2))-1.2*(v-self.vf2))
                         +a2*(self.af3-0.1*((p-d)-(self.pf3-d3))-1.2*(v-self.vf3))
                         +a3*(self.af4-0.1*((p-d)-(self.pf4-d4))-1.2*(v-self.vf4))
                         +a4*(self.af5-0.1*((p-d)-(self.pf5-d5))-1.2*(v-self.vf5))
                         +a5*(self.afr-0.1*((p-d)-pr)-1.2*(v-vr)))
            
            print('t:',sim_time)
            print("vr:",self.vfr)
            print("ar:",self.afr)
            print("A:",A)
            print('af'+str(droneid)+':',acc)

            # Update the setpoint message with the predicted position and velocity of the leader
            setpoint_msg = PositionTarget()
            setpoint_msg.header.stamp = rospy.Time.now()
            setpoint_msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            setpoint_msg.type_mask = PositionTarget.IGNORE_YAW_RATE + PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ + PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ
            setpoint_msg.header.frame_id = 'map'
            setpoint_msg.position = Vector3(0,0,0)
            setpoint_msg.velocity = Vector3(0,0,0)
            setpoint_msg.acceleration_or_force = Vector3(acc[0],acc[1],acc[2])
            setpoint_msg.yaw = yaw

            # print(setpoint_msg)

            # Publish the setpoint message
            self.setpoint_pub.publish(setpoint_msg)

            sim_time+=dt
            t+=dt
            shuffle_time+=dt
            if t>=25:
                t-=25
            # if shuffle_time>=5:
            #     print("shuffled")
            #     shuffle_time-=5
            #     np.random.shuffle(arr)

            rate.sleep()



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

    #Comment if VL data is used directly
    def vl_callback(self, msg):
        self.pfr = np.array([msg.position.x,msg.position.y,msg.position.z])
        self.vfr = np.array([msg.linear_velocity.x,msg.linear_velocity.y,msg.linear_velocity.z])
        self.afr = np.array([msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z])



if __name__ == '__main__':
    try:
        fdi = di_ext_controller()
    except rospy.ROSInterruptException:
        pass
