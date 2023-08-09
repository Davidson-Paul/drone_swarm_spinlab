#!/usr/bin/env python3

#Script for publishing Virtual Leader/ the actual Virtual Centre (VL/VC) states

from iq_gnc.msg import VL_local
import numpy as np
import rospy
from geometry_msgs.msg import Vector3
import math
from time_paths import *
import time

class VL_states_pub:
    def __init__(self):
        rospy.init_node("VL_pub")

        #Publishing to VL topic
        self.topic_pub = rospy.Publisher('/VL_states',VL_local,queue_size=10)

        #Running the code at specific timestamps with 5Hz
        rate = rospy.Rate(1000)
        self.dt = 0.2

        self.topic = VL_local()

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

        amp=20
        fc=1/60
        t=0

        while not rospy.is_shutdown():
            #Getting paths for VL
            if(t<60):
                xr,vr,ar,yr,wr = generate_sine_path(t,amp,fc,6,1)
            elif(t<120):
                xr,vr,ar,yr,wr = generate_straight_path((120-t),6,-1)
            else:
                xr,vr,ar,yr,wr = give_setpoint(0,0,6)

            # if(t<30):
            #     xr,vr,ar,yr,wr = generate_straight_path(-t,6,-1)
            # elif(t<60):
            #     xr,vr,ar,yr,wr = generate_straight_path(-(60-t),6,1)
            # else:
                # xr,vr,ar,yr,wr = give_setpoint(0,0,6)

            # xr,vr,ar,yr,wr = give_setpoint(0,0,6)
            # xr,vr,ar,yr,wr = generate_circle_path(t,r,vel,h)

            self.topic.header.stamp = rospy.Time.now()
            self.topic.position = Vector3(xr[0],xr[1],xr[2])
            self.topic.linear_velocity = Vector3(vr[0],vr[1],vr[2])
            self.topic.linear_acceleration = Vector3(ar[0],ar[1],ar[2])
            self.topic.yaw = yr
            self.topic.yaw_rate = wr

            current_time = rospy.Time.now()  # Get current time
            # Synchronize publishing with a specific time interval
            # Timing for 5 Hz rate - blocks runs at timestamp: .0, .2, .4, .6, .8
            if (current_time.to_sec()*5) % 1.0 < 0.05:

                #Publishing the topic after some time buffer
                time.sleep(0.09)
                self.topic_pub.publish(self.topic)
                t+=self.dt
                time.sleep(0.09)
            rate.sleep()
            


if __name__ == '__main__':
    try:
        vl = VL_states_pub()
    except rospy.ROSInterruptException:
        pass
