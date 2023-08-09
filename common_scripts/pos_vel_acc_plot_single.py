#!/usr/bin/env python3

#Script to plot setpoint and actual position,velocity and acceleration of single drone

import rospy
from mavros_msgs.msg import GlobalPositionTarget, PositionTarget, HomePosition
from geometry_msgs.msg import Vector3
from iq_gnc.msg import DroneStates, DroneStates_local, VL_local
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pymap3d as pm

class plotter:
    def __init__(self):
        rospy.init_node("plotter")

        rospy.Subscriber('/fury_1/mavros/home_position/home', HomePosition, self.origin_callback)
        rospy.wait_for_message('/fury_1/mavros/home_position/home', HomePosition)

        rospy.Subscriber('/fury_1/mavros/setpoint_raw/global',GlobalPositionTarget,self.f1_glob_setpoint_callback)
        rospy.Subscriber('/fury_1/mavros/setpoint_raw/local',PositionTarget,self.f1_loc_setpoint_callback)
        rospy.Subscriber('/fury_1/drone_states', DroneStates, self.f1_glob_actual_callback)
        rospy.Subscriber('/fury_1/drone_states_local', DroneStates_local, self.f1_loc_actual_callback)
        rospy.Subscriber('/VL_states', VL_local, self.vl_callback)

        self.start_time = rospy.Time.now().to_sec()

        self.f1_sgposition = []
        self.f1_sgvel = []
        self.f1_sgacc = []

        self.f1_slposition = []
        self.f1_slvel = []
        self.f1_slacc = []

        self.f1_agposition = []
        self.f1_agvel = []
        self.f1_agacc = []

        self.f1_alposition = []
        self.f1_alvel = []
        self.f1_alacc = []

        self.f1_time = []

        self.f1_gact = DroneStates()
        self.f1_lact = DroneStates_local()

        self.vl_position = []
        self.vl_vel = []
        self.vl_acc = []
        self.vl_time = []

    def vl_callback(self,msg):
        self.vl_position.append(msg.position)
        self.vl_vel.append(msg.linear_velocity)
        self.vl_acc.append(msg.linear_acceleration)
        self.vl_time.append(msg.header.stamp.to_sec()-self.start_time)

    def origin_callback(self,msg):
        self.home = msg.geo

    def f1_glob_setpoint_callback(self,msg):
        (x1,y1,z1) = pm.geodetic2enu(msg.latitude,msg.longitude,msg.altitude,self.home.latitude,self.home.longitude,self.home.altitude)
        (x2,y2,z2) = pm.geodetic2enu(self.f1_gact.latitude,self.f1_gact.longitude,self.f1_gact.rel_altitude,self.home.latitude,self.home.longitude,self.home.altitude)
        if (x1<500 and x2<500 and y1<500 and y2<500 and z1<500 and z2<500):
            self.f1_sgposition.append(Vector3(x1,y1,z1))
            self.f1_sgvel.append(msg.velocity)
            self.f1_sgacc.append(msg.acceleration_or_force)
            # self.f1_sgyaw.append(msg.yaw)
            self.f1_time.append(msg.header.stamp.to_sec()-self.start_time)

            self.f1_agposition.append(Vector3(x2,y2,z2))
            self.f1_agvel.append(self.f1_gact.linear_velocity)
            self.f1_agacc.append(self.f1_gact.linear_acceleration)
            # self.f1_agyaw.append(90.0-msg.heading)

    def f1_loc_setpoint_callback(self,msg):
        self.f1_slposition.append(msg.position)
        self.f1_slvel.append(msg.velocity)
        self.f1_slacc.append(msg.acceleration_or_force)
        # self.f1_slyaw.append(msg.yaw)
        self.f1_time.append(msg.header.stamp.to_sec()-self.start_time)

        self.f1_alposition.append(self.f1_lact.position)
        self.f1_alvel.append(self.f1_lact.linear_velocity)
        self.f1_alacc.append(self.f1_lact.linear_acceleration)
        # self.f1_alyaw.append(msg.yaw)

    def f1_glob_actual_callback(self,msg):
        self.f1_gact = msg

    def f1_loc_actual_callback(self,msg):
        self.f1_lact = msg

    def plot_all(self):
        while True:
            user_input = input("Enter 'yes' to plot: ")
            if user_input.lower() == "yes":
                if (self.f1_sgposition or self.f1_sgvel or self.f1_sgacc):
                    fig = plt.figure()
                    ax = fig.add_subplot(111, projection='3d')

                    if (self.f1_sgposition):
                        xs1 = [pos.x for pos in self.f1_sgposition]
                        ys1 = [pos.y for pos in self.f1_sgposition]
                        zs1 = [pos.z for pos in self.f1_sgposition]
                        ax.plot(xs1, ys1, zs1, label="f1 set position")
                    xs2 = [pos.x for pos in self.f1_agposition]
                    ys2 = [pos.y for pos in self.f1_agposition]
                    zs2 = [pos.z for pos in self.f1_agposition]
                    ax.plot(xs2, ys2, zs2, label="f1 actual position")
                    ax.legend()

                    ax.set_xlabel("X")
                    ax.set_ylabel("Y")
                    ax.set_zlabel("Z")
                    ax.set_title("f1 Position")
                    # plt.show(block="False")


                    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True)
                    if(self.f1_sgvel):
                        ax1.plot(self.f1_time, [vel.x for vel in self.f1_sgvel], label='f1 Vx control input')
                    ax1.plot(self.f1_time, [vel.x for vel in self.f1_agvel], label='f1 Vx response')
                    ax1.set_ylabel("Vx")
                    ax1.set_title("Velocity Components")
                    ax1.legend()

                    if(self.f1_sgvel):
                        ax2.plot(self.f1_time, [vel.y for vel in self.f1_sgvel], label='f1 Vy control input')
                    ax2.plot(self.f1_time, [vel.y for vel in self.f1_agvel], label='f1 Vy response')
                    ax2.set_ylabel("Vy")
                    ax2.legend()

                    if(self.f1_sgvel):
                        ax3.plot(self.f1_time, [vel.z for vel in self.f1_sgvel], label='f1 Vz control input')
                    ax3.plot(self.f1_time, [vel.z for vel in self.f1_agvel], label='f1 Vz response')
                    ax3.set_xlabel("Time")
                    ax3.set_ylabel("Vz")
                    ax3.legend()

                    # plt.show(block="False")


                    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True)
                    if(self.f1_sgacc):
                        ax1.plot(self.f1_time, [acc.x for acc in self.f1_sgacc], label='f1 Ax control input')
                    ax1.plot(self.f1_time, [acc.x for acc in self.f1_agacc], label='f1 Ax response')
                    ax1.set_ylabel("Ax")
                    ax1.set_title("Acceleration Components")
                    ax1.legend()

                    if(self.f1_sgacc):
                        ax2.plot(self.f1_time, [acc.y for acc in self.f1_sgacc], label='f1 Ay control input')
                    ax2.plot(self.f1_time, [acc.y for acc in self.f1_agacc], label='f1 Ay response')
                    ax2.set_ylabel("Ay")
                    ax2.legend()

                    if(self.f1_sgacc):
                        ax3.plot(self.f1_time, [acc.z for acc in self.f1_sgacc], label='f1 Az control input')
                    ax3.plot(self.f1_time, [acc.z for acc in self.f1_agacc], label='f1 Az response')
                    ax3.set_xlabel("Time")
                    ax3.set_ylabel("Az")
                    ax3.legend()

                    # plt.show(block="False")


                elif (self.f1_slposition or self.f1_slvel or self.f1_slacc):
                    fig = plt.figure()
                    ax = fig.add_subplot(111, projection='3d')

                    if (self.f1_slposition):
                        xs1 = [pos.x for pos in self.f1_slposition]
                        ys1 = [pos.y for pos in self.f1_slposition]
                        zs1 = [pos.z for pos in self.f1_slposition]
                        ax.plot(xs1, ys1, zs1, label="f1 set position")
                    xs2 = [pos.x for pos in self.f1_alposition]
                    ys2 = [pos.y for pos in self.f1_alposition]
                    zs2 = [pos.z for pos in self.f1_alposition]
                    ax.plot(xs2, ys2, zs2, label="f1 actual position")
                    ax.legend()

                    ax.set_xlabel("X")
                    ax.set_ylabel("Y")
                    ax.set_zlabel("Z")
                    ax.set_title("f1 Position")
                    # plt.show(block="False")


                    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True)
                    if(self.f1_slvel):
                        ax1.plot(self.f1_time, [vel.x for vel in self.f1_slvel], label='f1 Vx control input')
                    ax1.plot(self.f1_time, [vel.x for vel in self.f1_alvel], label='f1 Vx response')
                    ax1.set_ylabel("Vx")
                    ax1.set_title("Velocity Components")
                    ax1.legend()

                    if(self.f1_slvel):
                        ax2.plot(self.f1_time, [vel.y for vel in self.f1_slvel], label='f1 Vy control input')
                    ax2.plot(self.f1_time, [vel.y for vel in self.f1_alvel], label='f1 Vy response')
                    ax2.set_ylabel("Vy")
                    ax2.legend()

                    if(self.f1_slvel):
                        ax3.plot(self.f1_time, [vel.z for vel in self.f1_slvel], label='f1 Vz control input')
                    ax3.plot(self.f1_time, [vel.z for vel in self.f1_alvel], label='f1 Vz response')
                    ax3.set_xlabel("Time")
                    ax3.set_ylabel("Vz")
                    ax3.legend()

                    # plt.show(block="False")


                    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True)
                    if(self.f1_slacc):
                        ax1.plot(self.f1_time, [acc.x for acc in self.f1_slacc], label='f1 Ax control input')
                    ax1.plot(self.f1_time, [acc.x for acc in self.f1_alacc], label='f1 Ax response')
                    ax1.set_ylabel("Ax")
                    ax1.set_title("Acceleration Components")
                    ax1.legend()

                    if(self.f1_slacc):
                        ax2.plot(self.f1_time, [acc.y for acc in self.f1_slacc], label='f1 Ay control input')
                    ax2.plot(self.f1_time, [acc.y for acc in self.f1_alacc], label='f1 Ay response')
                    ax2.set_ylabel("Ay")
                    ax2.legend()

                    if(self.f1_slacc):
                        ax3.plot(self.f1_time, [acc.z for acc in self.f1_slacc], label='f1 Az control input')
                    ax3.plot(self.f1_time, [acc.z for acc in self.f1_alacc], label='f1 Az response')
                    ax3.set_xlabel("Time")
                    ax3.set_ylabel("Az")
                    ax3.legend()

                    # plt.show(block="False")

                
                if(self.vl_position):
                    fig = plt.figure()
                    ax = fig.add_subplot(111, projection='3d')

                    xs = [pos.x for pos in self.vl_position]
                    ys = [pos.y for pos in self.vl_position]
                    zs = [pos.z for pos in self.vl_position]
                    ax.plot(xs, ys, zs, label="vl position")
                    ax.legend()

                    ax.set_xlabel("X")
                    ax.set_ylabel("Y")
                    ax.set_zlabel("Z")
                    ax.set_title("VL Position")
                    # plt.show(block="False")

                    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True)

                    ax1.plot(self.vl_time, [vel.x for vel in self.vl_vel], label='VL Vx response')
                    ax1.set_ylabel("Vx")
                    ax1.set_title("Velocity Components")
                    ax1.legend()

                    ax2.plot(self.vl_time, [vel.y for vel in self.vl_vel], label='VL Vy response')
                    ax2.set_ylabel("Vy")
                    ax2.legend()

                    ax3.plot(self.vl_time, [vel.z for vel in self.vl_vel], label='VL Vz response')
                    ax3.set_xlabel("Time")
                    ax3.set_ylabel("Vz")
                    ax3.legend()

                    # plt.show(block="False")

                    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True)

                    ax1.plot(self.vl_time, [acc.x for acc in self.vl_acc], label='VL Ax response')
                    ax1.set_ylabel("Ax")
                    ax1.set_title("Acc Components")
                    ax1.legend()

                    ax2.plot(self.vl_time, [acc.y for acc in self.vl_acc], label='VL Ay response')
                    ax2.set_ylabel("Ay")
                    ax2.legend()

                    ax3.plot(self.vl_time, [acc.z for acc in self.vl_acc], label='VL Az response')
                    ax3.set_xlabel("Time")
                    ax3.set_ylabel("Az")
                    ax3.legend()

                    # plt.show(block="False")

                plt.show()

                break
            else:
                break

if __name__ == '__main__':
    try:
        plot = plotter()
        plot.plot_all()
    except rospy.ROSInterruptException:
        pass