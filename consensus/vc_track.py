#!/usr/bin/env python3

import rospy
from mavros_msgs.msg import PositionTarget
import math
from geometry_msgs.msg import Vector3
from iq_gnc.msg import virtual_centre, VL_local
import numpy as np
from scipy.integrate import solve_ivp
from std_msgs.msg import Int32
import time

class vc_track_controller:
    def __init__(self):
        rospy.init_node("vc_track")

        self.drone_name = rospy.get_namespace()
        self.droneid = rospy.get_param(f"{self.drone_name}vc_track/id")

        #Subscribing to each drone's own view of virtual centre(VC)
        rospy.Subscriber('/fury_1/virtual_centre', virtual_centre, self.f1_vc_callback)
        rospy.Subscriber('/fury_2/virtual_centre', virtual_centre, self.f2_vc_callback)
        rospy.Subscriber('/fury_3/virtual_centre', virtual_centre, self.f3_vc_callback)
        rospy.Subscriber('/fury_4/virtual_centre', virtual_centre, self.f4_vc_callback)
        rospy.Subscriber('/fury_5/virtual_centre', virtual_centre, self.f5_vc_callback)
        
        #Subscribing to VL topic (the virtual centre coordinates sent by the GCS to the swarm)
        rospy.Subscriber('/VL_states', VL_local, self.vl_callback)

        #Publishing to the current drone's VC
        self.vc_setpoint_pub = rospy.Publisher(f'{self.drone_name}virtual_centre', virtual_centre, queue_size=10)

        #Publishing to the drone's local setpoint
        self.drone_setpoint_pub = rospy.Publisher(f'{self.drone_name}mavros/setpoint_raw/local', PositionTarget, queue_size=10)

        #Initialising each drone's own position of VC
        self.vc_pf1=np.array([0,0,6],dtype=np.float64)
        self.vc_pf2=np.array([0,0,6],dtype=np.float64)
        self.vc_pf3=np.array([0,0,6],dtype=np.float64)
        self.vc_pf4=np.array([0,0,6],dtype=np.float64)
        self.vc_pf5=np.array([0,0,6],dtype=np.float64)
        self.pr=np.array([0,0,6],dtype=np.float64)

        #Initialising current drone's position of VC
        self.vc_p=np.array([0,0,6],dtype=np.float64)

        #Initialising each drone's own velocity of VC
        self.vc_vf1=np.array([0,0,0],dtype=np.float64)
        self.vc_vf2=np.array([0,0,0],dtype=np.float64)
        self.vc_vf3=np.array([0,0,0],dtype=np.float64)
        self.vc_vf4=np.array([0,0,0],dtype=np.float64)
        self.vc_vf5=np.array([0,0,0],dtype=np.float64)
        self.vr=np.array([0,0,0],dtype=np.float64)

        #Initialising current drone's velocity of VC
        self.vc_v=np.array([0,0,0],dtype=np.float64)

        #Initialising each drone's own acc of VC
        self.vc_af1=np.array([0,0,0],dtype=np.float64)
        self.vc_af2=np.array([0,0,0],dtype=np.float64)
        self.vc_af3=np.array([0,0,0],dtype=np.float64)
        self.vc_af4=np.array([0,0,0],dtype=np.float64)
        self.vc_af5=np.array([0,0,0],dtype=np.float64)
        self.ar=np.array([0,0,0],dtype=np.float64)

        #Initialising current drone's acc of VC
        self.vc_acc=np.array([0,0,0],dtype=np.float64)

        #Initialising each drone's own yaw of VC
        self.vc_yf1=0.0
        self.vc_yf2=0.0
        self.vc_yf3=0.0
        self.vc_yf4=0.0
        self.vc_yf5=0.0
        self.yr=0.0

        #Initialising current drone's yaw of VC
        self.vc_y = 0.0

        #Initialising each drone's own yaw_rate of VC
        self.vc_wf1=0.0
        self.vc_wf2=0.0
        self.vc_wf3=0.0
        self.vc_wf4=0.0
        self.vc_wf5=0.0
        self.wr=0.0

        #Initialising current drone's yaw_rate of VC
        self.vc_w = 0.0

        #Initialising publishing topics
        self.vc_setpoint_msg = virtual_centre()
        self.drone_setpoint_msg = PositionTarget()

        #Loops run at 1000 Hz
        self.rate = rospy.Rate(1000)

        self.start_time = rospy.Time.now()

        #Time-step for integration
        self.dt=0.05

        #relative deviation wrt to VC position and orientation (maintaing formation)
        self.d1=np.array([0,0,0],dtype=np.float64)
        self.d2=np.array([-21.65,12.5,0],dtype=np.float64)
        self.d3=np.array([-21.65,-12.5,0],dtype=np.float64)
        self.d4=np.array([-43.3,25,0],dtype=np.float64)
        self.d5=np.array([-43.3,-25,0],dtype=np.float64)

        #Current drone's relative deviation
        self.d=np.array([0,0,0],dtype=np.float64)

        #Adjacency matrix
        self.adj_mat = np.loadtxt('/home/davidson/catkin_ws/src/iq_gnc/adjacency_matrix.txt', dtype=int)

        #Publishing 1st VC message
        self.vc_setpoint_msg.header.stamp = rospy.Time.now()
        self.vc_setpoint_msg.header.frame_id = 'inertial'
        self.vc_setpoint_msg.position = Vector3(0,0,6)
        self.vc_setpoint_msg.linear_velocity = Vector3(0,0,0)
        self.vc_setpoint_msg.linear_acceleration = Vector3(0,0,0)
        self.vc_setpoint_msg.yaw = 0
        self.vc_setpoint_msg.yaw_rate = 0
        self.vc_setpoint_pub.publish(self.vc_setpoint_msg)
        print('message1')

    def vc_track_loop(self):
        #Loop for tracking the VC

        #Starting all tracking loops of the drones at the same time
        # Access the start_time from the text file
        with open("/home/davidson/catkin_ws/src/iq_gnc/start_time.txt", "r") as file:
            stored_start_time = file.read()

        # Convert the stored start_time to a timestamp
        start_timestamp = time.mktime(time.strptime(stored_start_time, "%Y-%m-%d %H:%M:%S"))

        # Get the current time
        current_timestamp = time.time()

        # Calculate the delay until the start time
        delay = start_timestamp - current_timestamp

        # Check if the delay is positive
        if delay > 0:
            print(f"Waiting for {delay} seconds until {stored_start_time}")
            time.sleep(delay)

        t=0

        while not rospy.is_shutdown():
            current_time = rospy.Time.now()  # Get current time
            # Synchronize publishing with a specific time interval
            #Timimg for 20Hz rate - the block runs for timestamps .05, .10, .15, .20, .25, .30, .35, .40 and so on
            if (current_time.to_sec()*20) % 1.0 < 0.1:

                #Getting the adjecency matrix
                with open('/home/davidson/catkin_ws/src/iq_gnc/adjacency_matrix.txt') as file:
                    first_line = file.readline()
                    if first_line.strip():
                        file.seek(0)  # Reset the file pointer to the beginning
                        self.adj_mat = np.loadtxt(file, dtype=int)

                #assignning adjacency matrix entries for each agent
                self.A = self.adj_mat[(self.droneid-1),:]

                #Setting current drone's values wrt to droneid in launch file
                if(self.droneid==1):
                    self.vc_p=self.vc_pf1
                    self.vc_v=self.vc_vf1
                    self.vc_y=self.vc_yf1
                    self.d=self.d1
                elif(self.droneid==2):
                    self.vc_p=self.vc_pf2
                    self.vc_v=self.vc_vf2
                    self.vc_y=self.vc_yf2
                    self.d=self.d2
                elif(self.droneid==3):
                    self.vc_p=self.vc_pf3
                    self.vc_v=self.vc_vf3
                    self.vc_y=self.vc_yf3
                    self.d=self.d3
                elif(self.droneid==4):
                    self.vc_p=self.vc_pf4
                    self.vc_v=self.vc_vf4
                    self.vc_y=self.vc_yf4
                    self.d=self.d4
                elif(self.droneid==5):
                    self.vc_p=self.vc_pf5
                    self.vc_v=self.vc_vf5
                    self.vc_y=self.vc_yf5
                    self.d=self.d5


                #Solving CT equations
                t_span = [t,t+self.dt]

                #Custom RK-4 one-step integration
                px,vx = self.rk_4(self.di_eq_x, [self.vc_p[0], self.vc_v[0]], t, self.dt)
                self.vc_p[0] = px
                self.vc_v[0] = vx

                py,vy = self.rk_4(self.di_eq_y, [self.vc_p[1], self.vc_v[1]], t, self.dt)
                self.vc_p[1] = py
                self.vc_v[1] = vy

                pz,vz = self.rk_4(self.di_eq_z, [self.vc_p[2], self.vc_v[2]], t, self.dt)
                self.vc_p[2] = pz
                self.vc_v[2] = vz

                #In built ode solver
                sol2 = solve_ivp(self.si_eq, t_span, [self.vc_y], method='RK45')
                self.vc_y = sol2.y[0][-1]

                #Assinging values to publish
                self.vc_setpoint_msg.header.stamp = rospy.Time.now()
                self.vc_setpoint_msg.header.frame_id = 'inertial'
                self.vc_setpoint_msg.position = Vector3(self.vc_p[0],self.vc_p[1],self.vc_p[2])
                self.vc_setpoint_msg.linear_velocity = Vector3(self.vc_v[0],self.vc_v[1],self.vc_v[2])
                self.vc_setpoint_msg.linear_acceleration = Vector3(self.vc_acc[0],self.vc_acc[1],self.vc_acc[2])
                self.vc_setpoint_msg.yaw = self.vc_y
                self.vc_setpoint_msg.yaw_rate = self.vc_w
                
                print('t'+str(self.droneid)+':',t)
                print('self.A'+str(self.droneid)+':',self.A)
                print("vr:",self.vr)
                print('self.vc_v'+str(self.droneid)+':',self.vc_v)
                print("ar:",self.ar)
                print('self.vc_acc'+str(self.droneid)+':',self.vc_acc)
                print('yr:',self.yr)
                print("self.vc_y",self.vc_y)

                # Publish the setpoint message after some buffer time
                time.sleep(0.03)
                self.vc_setpoint_pub.publish(self.vc_setpoint_msg)
                t+=self.dt
                time.sleep(0.01)

            self.rate.sleep()  

    # def trajectory_track_loop(self):
    #     #Loop for single-drone trajectory-tracking wrt its VC

    #     while not rospy.is_shutdown():
    #         current_time = rospy.Time.now()  # Get current time

    #         # Synchronize publishing with a specific time interval
    #         # Timing for 5 Hz rate - blocks runs at timestamp: .0, .2, .4, .6, .8
    #         if (current_time.to_sec()*5) % 1.0 < 0.05:

    #             #Get inertial position of the drone wrt virtual centre
    #             r = self.vc_p + np.matmul(np.array([[np.cos(self.vc_y),-np.sin(self.vc_y),0],[np.sin(self.vc_y),np.cos(self.vc_y),0],[0,0,1]]),self.d) - self.d #adding offset of 'self.d' for global origin to drone-starting origin 

    #             # Update the setpoint message with the predicted position and velocity of the leader
    #             self.drone_setpoint_msg.header.stamp = rospy.Time.now()
    #             self.drone_setpoint_msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
    #             self.drone_setpoint_msg.type_mask = PositionTarget.IGNORE_YAW_RATE + PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ
    #             self.drone_setpoint_msg.header.frame_id = 'map'
    #             self.drone_setpoint_msg.position = Vector3(r[0],r[1],r[2])
    #             self.drone_setpoint_msg.yaw = self.vc_y

    #             print('setpoint'+str(self.droneid)+':',r)

    #             # Publish the setpoint message after some time buffer
    #             time.sleep(0.09)
    #             self.drone_setpoint_pub.publish(self.drone_setpoint_msg)
    #             time.sleep(0.09)

    #         self.rate.sleep()  

    def di_eq_x(self,t,p,v):
        n=np.sum(self.A)

        self.vc_p[0] = p
        self.vc_v[0] = v

        #DI consensus alg to track the virtual centre x-position
        self.vc_acc[0] = (1/n)*(self.A[0]*(self.vc_af1[0]-1.0*(self.vc_p[0]-self.vc_pf1[0])-1.00*(self.vc_v[0]-self.vc_vf1[0]))
                        +self.A[1]*(self.vc_af2[0]-1.0*(self.vc_p[0]-self.vc_pf2[0])-1.00*(self.vc_v[0]-self.vc_vf2[0]))
                        +self.A[2]*(self.vc_af3[0]-1.0*(self.vc_p[0]-self.vc_pf3[0])-1.00*(self.vc_v[0]-self.vc_vf3[0]))
                        +self.A[3]*(self.vc_af4[0]-1.0*(self.vc_p[0]-self.vc_pf4[0])-1.00*(self.vc_v[0]-self.vc_vf4[0]))
                        +self.A[4]*(self.vc_af5[0]-1.0*(self.vc_p[0]-self.vc_pf5[0])-1.00*(self.vc_v[0]-self.vc_vf5[0]))
                        +self.A[5]*(self.ar[0]-1.0*(self.vc_p[0]-self.pr[0])-1.00*(self.vc_v[0]-self.vr[0])))
        return self.vc_acc[0]
    
    def di_eq_y(self,t,p,v):
        n=np.sum(self.A)

        self.vc_p[1] = p
        self.vc_v[1] = v

        #DI consensus alg to track the virtual centre y-position
        self.vc_acc[1] = (1/n)*(self.A[0]*(self.vc_af1[1]-1.0*(self.vc_p[1]-self.vc_pf1[1])-1.00*(self.vc_v[1]-self.vc_vf1[1]))
                        +self.A[1]*(self.vc_af2[1]-1.0*(self.vc_p[1]-self.vc_pf2[1])-1.00*(self.vc_v[1]-self.vc_vf2[1]))
                        +self.A[2]*(self.vc_af3[1]-1.0*(self.vc_p[1]-self.vc_pf3[1])-1.00*(self.vc_v[1]-self.vc_vf3[1]))
                        +self.A[3]*(self.vc_af4[1]-1.0*(self.vc_p[1]-self.vc_pf4[1])-1.00*(self.vc_v[1]-self.vc_vf4[1]))
                        +self.A[4]*(self.vc_af5[1]-1.0*(self.vc_p[1]-self.vc_pf5[1])-1.00*(self.vc_v[1]-self.vc_vf5[1]))
                        +self.A[5]*(self.ar[1]-1.0*(self.vc_p[1]-self.pr[1])-1.00*(self.vc_v[1]-self.vr[1])))
        return self.vc_acc[1]
    
    def di_eq_z(self,t,p,v):
        n=np.sum(self.A)

        self.vc_p[2] = p
        self.vc_v[2] = v

        #DI consensus alg to track the virtual centre z-position
        self.vc_acc[2] = (1/n)*(self.A[0]*(self.vc_af1[2]-1.0*(self.vc_p[2]-self.vc_pf1[2])-1.00*(self.vc_v[2]-self.vc_vf1[2]))
                        +self.A[1]*(self.vc_af2[2]-1.0*(self.vc_p[2]-self.vc_pf2[2])-1.00*(self.vc_v[2]-self.vc_vf2[2]))
                        +self.A[2]*(self.vc_af3[2]-1.0*(self.vc_p[2]-self.vc_pf3[2])-1.00*(self.vc_v[2]-self.vc_vf3[2]))
                        +self.A[3]*(self.vc_af4[2]-1.0*(self.vc_p[2]-self.vc_pf4[2])-1.00*(self.vc_v[2]-self.vc_vf4[2]))
                        +self.A[4]*(self.vc_af5[2]-1.0*(self.vc_p[2]-self.vc_pf5[2])-1.00*(self.vc_v[2]-self.vc_vf5[2]))
                        +self.A[5]*(self.ar[2]-1.0*(self.vc_p[2]-self.pr[2])-1.00*(self.vc_v[2]-self.vr[2])))
        return self.vc_acc[2]
    
    def si_eq(self,t,k):
        n=np.sum(self.A)

        self.vc_y = k

        #SI consensus alg to track the virtual centre orientation
        self.vc_w = (1/n)*(self.A[0]*(self.vc_wf1-1.000*(self.vc_y-self.vc_yf1))
                        +self.A[1]*(self.vc_wf2-1.000*(self.vc_y-self.vc_yf2))
                        +self.A[2]*(self.vc_wf3-1.000*(self.vc_y-self.vc_yf3))
                        +self.A[3]*(self.vc_wf4-1.000*(self.vc_y-self.vc_yf4))
                        +self.A[4]*(self.vc_wf5-1.000*(self.vc_y-self.vc_yf5))
                        +self.A[5]*(self.wr-1.000*(self.vc_y-self.yr)))
        
        return self.vc_w
    
    def rk_4(self,eq,u0,t0,dt):
        #Performing one-step RK-4 integration

        # eq - equation for acceleration
        # u0 - initial condition [position, velocity]
        # t0 - initial time
        # dt - timestep

        x = u0[0]
        v = u0[1]

        k1 = v
        k2 = eq(t0,x,v)

        s1 = v+k2*dt/2
        s2 = eq(t0+dt/2,x+k1*dt/2,v+k2*dt/2)

        l1 = v+s2*dt/2
        l2 = eq(t0+dt/2,x+s1*dt/2,v+s2*dt/2)

        p1 = v+l2*dt
        p2 = eq(t0+dt,x+l1*dt,v+l2*dt)

        x_next = x + (k1+2*s1+2*l1+p1)*dt/6
        v_next = v + (k2+2*s2+2*l2+p2)*dt/6
        return x_next,v_next

    #Callback functions
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
        fury_vc_con = vc_track_controller()
        fury_vc_con.vc_track_loop()

    except rospy.ROSInterruptException:
        pass
