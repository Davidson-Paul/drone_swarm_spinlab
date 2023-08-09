#! /usr/bin/env python3

#Script to takeoff drone from any point on land

import rospy
from iq_gnc.py_gnc_functions_2 import *
import math

def main():
    rospy.init_node("fury_controller", anonymous=True)
    
    fury.wait4connect()
    fury.set_mode('GUIDED')
    rospy.loginfo("Set to GUIDED mode")

    #get the drone's initial position inorder to give setpoints to the drnoe
    fury.initialize_local_frame()

    fury.takeoff(6)
    rospy.loginfo("Taking off")


if __name__ == '__main__':
    try:
        fury=gnc_api()
        main()
    except rospy.ROSInterruptException:
        pass
