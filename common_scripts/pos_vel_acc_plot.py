#!/usr/bin/env python3
#Script to plot setpoint and actual position,velocity and acceleration of all the drones
#exec() is used

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

        rospy.Subscriber('/VL_states', VL_local, self.vl_callback)

        self.start_time = rospy.Time.now().to_sec()

        self.vl_position = []
        self.vl_vel = []
        self.vl_acc = []
        self.vl_time = []
        self.vlf1 = []
        self.vlf2 = []
        self.vlf3 = []
        self.vlf4 = []
        self.vlf5 = []

        for i in range(1, 6):
            exec(f"rospy.Subscriber('/fury_{i}/mavros/setpoint_raw/global', GlobalPositionTarget, self.f{i}_glob_setpoint_callback)")
            exec(f"rospy.Subscriber('/fury_{i}/mavros/setpoint_raw/local', PositionTarget, self.f{i}_loc_setpoint_callback)")
            exec(f"rospy.Subscriber('/fury_{i}/drone_states', DroneStates, self.f{i}_glob_actual_callback)")
            exec(f"rospy.Subscriber('/fury_{i}/drone_states_local', DroneStates_local, self.f{i}_loc_actual_callback)")

            exec(f"self.f{i}_sgposition = []")
            exec(f"self.f{i}_sgvel = []")
            exec(f"self.f{i}_sgacc = []")

            exec(f"self.f{i}_slposition = []")
            exec(f"self.f{i}_slvel = []")
            exec(f"self.f{i}_slacc = []")

            exec(f"self.f{i}_agposition = []")
            exec(f"self.f{i}_agvel = []")
            exec(f"self.f{i}_agacc = []")

            exec(f"self.f{i}_alposition = []")
            exec(f"self.f{i}_alvel = []")
            exec(f"self.f{i}_alacc = []")

            exec(f"self.f{i}_time = []")

            exec(f"self.f{i}_gact = DroneStates()")
            exec(f"self.f{i}_lact = DroneStates_local()")
    
    def origin_callback(self, msg):
        self.home = msg.geo

    for i in range(1, 6):
        exec(f"def f{i}_glob_setpoint_callback(self, msg):\n"
            f"    (x1, y1, z1) = pm.geodetic2enu(msg.latitude, msg.longitude, msg.altitude, self.home.latitude, self.home.longitude, self.home.altitude)\n"
            f"    (x2, y2, z2) = pm.geodetic2enu(self.f{i}_gact.latitude, self.f{i}_gact.longitude, self.f{i}_gact.rel_altitude, self.home.latitude, self.home.longitude, self.home.altitude)\n"
            f"    if (x1<500 and x2<500 and y1<500 and y2<500 and z1<500 and z2<500):\n"
            f"        self.f{i}_sgposition.append(Vector3(x1, y1, z1))\n"
            f"        self.f{i}_agposition.append(Vector3(x2, y2, z2))\n"
            f"    self.f{i}_sgvel.append(msg.velocity)\n"
            f"    self.f{i}_sgacc.append(msg.acceleration_or_force)\n"
            f"    self.f{i}_time.append(msg.header.stamp.to_sec() - self.start_time)\n"
            f"    self.f{i}_agvel.append(self.f{i}_gact.linear_velocity)\n"
            f"    self.f{i}_agacc.append(self.f{i}_gact.linear_acceleration)\n"
            "\n"
            f"def f{i}_loc_setpoint_callback(self, msg):\n"
            f"    self.f{i}_slposition.append(msg.position)\n"
            f"    self.f{i}_slvel.append(msg.velocity)\n"
            f"    self.f{i}_slacc.append(msg.acceleration_or_force)\n"
            f"    self.f{i}_time.append(msg.header.stamp.to_sec() - self.start_time)\n"
            "\n"
            f"    self.f{i}_alposition.append(self.f{i}_lact.position)\n"
            f"    self.f{i}_alvel.append(self.f{i}_lact.linear_velocity)\n"
            f"    self.f{i}_alacc.append(self.f{i}_lact.linear_acceleration)\n"
            "\n"
            f"def f{i}_glob_actual_callback(self, msg):\n"
            f"    self.f{i}_gact = msg\n"
            "\n"
            f"def f{i}_loc_actual_callback(self, msg):\n"
            f"    self.f{i}_lact = msg\n"
            "\n"
        )

    def vl_callback(self,msg):
        self.vl_position.append(msg.position)
        self.vl_vel.append(msg.linear_velocity)
        self.vl_acc.append(msg.linear_acceleration)
        self.vl_time.append(msg.header.stamp.to_sec()-self.start_time)
        self.vlf1.append(self.f1_lact.position)
        self.vlf2.append(self.f2_lact.position)
        self.vlf3.append(self.f3_lact.position)
        self.vlf4.append(self.f4_lact.position)
        self.vlf5.append(self.f5_lact.position)
    
    def plot_all(self):
        while True:
            user_input = input("Enter 'yes' to plot: ")
            if user_input.lower() == "yes":
                code = '''
if (self.f{i}_sgposition or self.f{i}_sgvel or self.f{i}_sgacc):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    if (self.f{i}_sgposition):
        xs1 = [pos.x for pos in self.f{i}_sgposition]
        ys1 = [pos.y for pos in self.f{i}_sgposition]
        zs1 = [pos.z for pos in self.f{i}_sgposition]
        ax.plot(xs1, ys1, zs1, label='f{i} set position')
    xs2 = [pos.x for pos in self.f{i}_agposition]
    ys2 = [pos.y for pos in self.f{i}_agposition]
    zs2 = [pos.z for pos in self.f{i}_agposition]
    ax.plot(xs2, ys2, zs2, label='f{i} actual position')
    ax.legend()

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("f{i} Position")

    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True)
    if(self.f{i}_sgvel):
        ax1.plot(self.f{i}_time, [vel.x for vel in self.f{i}_sgvel], label='f{i} Vx control input')
    ax1.plot(self.f{i}_time, [vel.x for vel in self.f{i}_agvel], label='f{i} Vx response')
    ax1.set_ylabel("Vx")
    ax1.set_title("Velocity Components")
    ax1.legend()

    if(self.f{i}_sgvel):
        ax2.plot(self.f{i}_time, [vel.y for vel in self.f{i}_sgvel], label='f{i} Vy control input')
    ax2.plot(self.f{i}_time, [vel.y for vel in self.f{i}_agvel], label='f{i} Vy response')
    ax2.set_ylabel("Vy")
    ax2.legend()

    if(self.f{i}_sgvel):
        ax3.plot(self.f{i}_time, [vel.z for vel in self.f{i}_sgvel], label='f{i} Vz control input')
    ax3.plot(self.f{i}_time, [vel.z for vel in self.f{i}_agvel], label='f{i} Vz response')
    ax3.set_xlabel("Time")
    ax3.set_ylabel("Vz")
    ax3.legend()


    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True)
    if(self.f{i}_sgacc):
        ax1.plot(self.f{i}_time, [acc.x for acc in self.f{i}_sgacc], label='f{i} Ax control input')
    ax1.plot(self.f{i}_time, [acc.x for acc in self.f{i}_agacc], label='f{i} Ax response')
    ax1.set_ylabel("Ax")
    ax1.set_title("Acceleration Components")
    ax1.legend()

    if(self.f{i}_sgacc):
        ax2.plot(self.f{i}_time, [acc.y for acc in self.f{i}_sgacc], label='f{i} Ay control input')
    ax2.plot(self.f{i}_time, [acc.y for acc in self.f{i}_agacc], label='f{i} Ay response')
    ax2.set_ylabel("Ay")
    ax2.legend()

    if(self.f{i}_sgacc):
        ax3.plot(self.f{i}_time, [acc.z for acc in self.f{i}_sgacc], label='f{i} Az control input')
    ax3.plot(self.f{i}_time, [acc.z for acc in self.f{i}_agacc], label='f{i} Az response')
    ax3.set_xlabel("Time")
    ax3.set_ylabel("Az")
    ax3.legend()


elif (self.f{i}_slposition or self.f{i}_slvel or self.f{i}_slacc):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    if (self.f{i}_slposition):
        xs1 = [pos.x for pos in self.f{i}_slposition]
        ys1 = [pos.y for pos in self.f{i}_slposition]
        zs1 = [pos.z for pos in self.f{i}_slposition]
        ax.plot(xs1, ys1, zs1, label='f{i} set position')
    xs2 = [pos.x for pos in self.f{i}_alposition]
    ys2 = [pos.y for pos in self.f{i}_alposition]
    zs2 = [pos.z for pos in self.f{i}_alposition]
    ax.plot(xs2, ys2, zs2, label='f{i} actual position')
    ax.legend()

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("f{i} Position")

    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True)
    if(self.f{i}_slvel):
        ax1.plot(self.f{i}_time, [vel.x for vel in self.f{i}_slvel], label='f{i} Vx control input')
    ax1.plot(self.f{i}_time, [vel.x for vel in self.f{i}_alvel], label='f{i} Vx response')
    ax1.set_ylabel("Vx")
    ax1.set_title("Velocity Components")
    ax1.legend()

    if(self.f{i}_slvel):
        ax2.plot(self.f{i}_time, [vel.y for vel in self.f{i}_slvel], label='f{i} Vy control input')
    ax2.plot(self.f{i}_time, [vel.y for vel in self.f{i}_alvel], label='f{i} Vy response')
    ax2.set_ylabel("Vy")
    ax2.legend()

    if(self.f{i}_slvel):
        ax3.plot(self.f{i}_time, [vel.z for vel in self.f{i}_slvel], label='f{i} Vz control input')
    ax3.plot(self.f{i}_time, [vel.z for vel in self.f{i}_alvel], label='f{i} Vz response')
    ax3.set_xlabel("Time")
    ax3.set_ylabel("Vz")
    ax3.legend()


    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True)
    if(self.f{i}_slacc):
        ax1.plot(self.f{i}_time, [acc.x for acc in self.f{i}_slacc], label='f{i} Ax control input')
    ax1.plot(self.f{i}_time, [acc.x for acc in self.f{i}_alacc], label='f{i} Ax response')
    ax1.set_ylabel("Ax")
    ax1.set_title("Acceleration Components")
    ax1.legend()

    if(self.f{i}_slacc):
        ax2.plot(self.f{i}_time, [acc.y for acc in self.f{i}_slacc], label='f{i} Ay control input')
    ax2.plot(self.f{i}_time, [acc.y for acc in self.f{i}_alacc], label='f{i} Ay response')
    ax2.set_ylabel("Ay")
    ax2.legend()

    if(self.f{i}_slacc):
        ax3.plot(self.f{i}_time, [acc.z for acc in self.f{i}_slacc], label='f{i} Az control input')
    ax3.plot(self.f{i}_time, [acc.z for acc in self.f{i}_alacc], label='f{i} Az response')
    ax3.set_xlabel("Time")
    ax3.set_ylabel("Az")
    ax3.legend()
                '''
                    
                for i in range(1, 6):
                    exec(code.format(i=i))

                if(self.vl_position):
                    fig = plt.figure()
                    ax = fig.add_subplot(111, projection='3d')

                    xs = [pos.x for pos in self.vl_position]
                    ys = [pos.y for pos in self.vl_position]
                    zs = [pos.z for pos in self.vl_position]
                    ax.plot(xs, ys, zs, label="vl position")

                    xs1 = [pos.x for pos in self.vlf1]
                    ys1 = [pos.y for pos in self.vlf1]
                    zs1 = [pos.z for pos in self.vlf1]
                    ax.plot(xs1, ys1, zs1, label="f1")

                    xs2 = [pos.x for pos in self.vlf2]
                    ys2 = [pos.y for pos in self.vlf2]
                    zs2 = [pos.z for pos in self.vlf2]
                    ax.plot(xs2, ys2, zs2, label="f2")

                    xs3 = [pos.x for pos in self.vlf3]
                    ys3 = [pos.y for pos in self.vlf3]
                    zs3 = [pos.z for pos in self.vlf3]
                    ax.plot(xs3, ys3, zs3, label="f3")

                    xs4 = [pos.x for pos in self.vlf4]
                    ys4 = [pos.y for pos in self.vlf4]
                    zs4 = [pos.z for pos in self.vlf4]
                    ax.plot(xs4, ys4, zs4, label="f4")

                    xs5 = [pos.x for pos in self.vlf5]
                    ys5 = [pos.y for pos in self.vlf5]
                    zs5 = [pos.z for pos in self.vlf5]
                    ax.plot(xs5, ys5, zs5, label="f5")
                    
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