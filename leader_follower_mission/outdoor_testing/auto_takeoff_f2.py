#!/usr/bin/env python
import rospy
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import CommandTOL
from mavros_msgs.msg import State, HomePosition
from geometry_msgs.msg import Vector3
from mavros_msgs.msg import GlobalPositionTarget
from iq_gnc.msg import DroneStates
import math
import pymap3d as pm
import time


def send_takeoff_command(drone_id, altitude):
    rospy.wait_for_service(f'{drone_id}/mavros/cmd/takeoff')
    try:
        takeoff_service = rospy.ServiceProxy(f'{drone_id}/mavros/cmd/takeoff', CommandTOL)
        resp = takeoff_service(0, 0, 0, 0, altitude)
        if resp.success:
            print(f"{drone_id} Takeoff command sent successfully.")
            return True
        else:
            print(f"Failed to send {drone_id} takeoff command. Check system status.")
            return False
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")
        return False
        print(drone_id)

def arm_drone(drone_id):
    rospy.wait_for_service(f'{drone_id}/mavros/cmd/arming')
    try:
        arming_service = rospy.ServiceProxy(f'{drone_id}/mavros/cmd/arming', CommandBool)
        resp = arming_service(True)
        if resp.success:
            print(f"{drone_id} Drone armed successfully.")
            return True
        else:
            print(f"Failed to arm {drone_id} drone.")
            return False
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")
        return False
    
class DroneController:
    def __init__(self):
        # Initialize ROS node
        # rospy.init_node('drone_controller')

        self.current_state_g = State()
        self.leader_states = DroneStates()

        rospy.Subscriber('ditto_1/drone_states',DroneStates,self.leader_states_callback)
        self.glob_setpoint_pub = rospy.Publisher('/ditto_2/mavros/setpoint_raw/global',GlobalPositionTarget,queue_size=10)

        # Subscribe to the home position of the leader drone
        rospy.Subscriber('/ditto_1/mavros/home_position/home', HomePosition, self.leader_home_position_callback)
        rospy.Subscriber('/ditto_2/mavros/home_position/home', HomePosition, self.follower_home_position_callback)

        # Wait for topics to become available
        rospy.wait_for_message('/ditto_1/mavros/home_position/home', HomePosition)
        rospy.wait_for_message('/ditto_2/mavros/home_position/home', HomePosition)

        state_sub_1 = rospy.Subscriber('/ditto_2/mavros/state', State, self.state_callback_1)

        # Set the loop rate to 10 Hz       
        self.rate = rospy.Rate(5)

    def state_callback_1(self,msg):
        self.current_state_g = msg

    def wait4connect(self):
        """Wait for connect is a function that will hold the program until communication with the FCU is established.

        Returns:
                0 (int): Connected to FCU.
                -1 (int): Failed to connect to FCU.
        """
        rospy.loginfo("Waiting for FCU connection")
        while not rospy.is_shutdown() and not self.current_state_g.connected:
            rospy.sleep(0.01)
        else:
            if self.current_state_g.connected:
                rospy.loginfo("FCU connected")
                return 0
            else:
                rospy.logerr("Error connecting to drone's FCU")
                return -1
    def wait4start(self):
        """This function will hold the program until the user signals the FCU to mode enter GUIDED mode. This is typically done from a switch on the safety pilot's remote or from the Ground Control Station.

        Returns:
                0 (int): Mission started successfully.
                -1 (int): Failed to start mission.
        """
        rospy.loginfo(
                      "Waiting for user to set mode to GUIDED")
        while not rospy.is_shutdown() and self.current_state_g.mode != "GUIDED":
            rospy.sleep(0.01)
        else:
            if self.current_state_g.mode == "GUIDED":
                rospy.loginfo(
                     "Mode set to GUIDED. Starting Mission..." )
                return 0
            else:
                rospy.logerr("Error startting mission")
                return -1           

    def control_loop(self):
        # Start the main loop
        while not rospy.is_shutdown():
            follow_distance = 21.6  # 1 meter behind
            left_distance = 12.5 # 1 meter to the left
            alt_distance = 6

            dt = 0.10  # time interval in seconds 0.1 
            dx = self.leader_states.linear_velocity.x * dt + 0.5 * self.leader_states.linear_acceleration.x * dt ** 2  # calculate distance traveled in x direction based on velocity and acceleration
            dy = self.leader_states.linear_velocity.y * dt + 0.5 * self.leader_states.linear_acceleration.y * dt ** 2  # calculate distance traveled in y direction based on velocity and acceleration
            distance = math.sqrt(dx**2 + dy**2)  # calculate total distance traveled

            yaw = math.radians(90.0-self.leader_states.heading)

            ox = - follow_distance * math.cos(yaw) + distance * math.cos(yaw) - left_distance * math.sin(yaw)  # predict future x position of leader
            oy = - follow_distance * math.sin(yaw) + distance * math.sin(yaw) + left_distance * math.cos(yaw) # predict future y position of leader
            oz = 0 #+ self.leader_states.linear_velocity.z * dt + 0.5 * self.leader_states.linear_acceleration.z * dt ** 2  # predict future z position of leader

            #converting the offsets wrt to leader global frame
            tlat,tlong,talt=pm.enu2geodetic(ox,oy,oz,self.leader_states.latitude,self.leader_states.longitude,self.leader_states.rel_altitude)


            # Extrapolate the leader's velocity based on its accelerationS
            e = 0.00 #error compensation 0.05
            vx = self.leader_states.linear_velocity.x + self.leader_states.linear_acceleration.x * dt + e
            vy = self.leader_states.linear_velocity.y + self.leader_states.linear_acceleration.y * dt + e
            vz = self.leader_states.linear_velocity.z #+ self.leader_states.linear_acceleration.z * dt
            target_vel = Vector3(vx, vy, vz)

            setpoint_msg = GlobalPositionTarget()
            setpoint_msg.header.stamp = rospy.Time.now()
            setpoint_msg.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_REL_ALT #height set from ground 
            setpoint_msg.type_mask = GlobalPositionTarget.IGNORE_AFX + GlobalPositionTarget.IGNORE_AFY + GlobalPositionTarget.IGNORE_AFZ + GlobalPositionTarget.IGNORE_YAW_RATE 
            setpoint_msg.header.frame_id = 'map'
            setpoint_msg.latitude = tlat
            setpoint_msg.longitude = tlong
            setpoint_msg.altitude = alt_distance + self.leader_states.rel_altitude
            setpoint_msg.velocity =  target_vel
            # setpoint_msg.acceleration_or_force = self.leader_states.linear_acceleration
            setpoint_msg.yaw = yaw

            self.glob_setpoint_pub.publish(setpoint_msg)


            leader_pos=Vector3(self.leader_states.latitude,self.leader_states.longitude,self.leader_states.rel_altitude)
            print("Leader_pos:",leader_pos)
            print("leader_vel:",math.sqrt(self.leader_states.linear_velocity.x**2+self.leader_states.linear_velocity.y**2+self.leader_states.linear_velocity.z**2))

            target_pos=Vector3(setpoint_msg.latitude,setpoint_msg.longitude,setpoint_msg.altitude)
            print("Target_pos: ", target_pos)
            print("Target_vel:",math.sqrt(target_vel.x**2+target_vel.y**2+target_vel.z**2))
            print("yaw:",yaw)

            self.rate.sleep()

    def leader_states_callback(self, msg):
        # Get all leader's info
        self.leader_states = msg

    def leader_home_position_callback(self, msg):
        # Get the local position of the leader drone
        self.leader_home_pos = msg.geo

    def follower_home_position_callback(self, msg):
        # Get the local position of the follower drone
        self.follower_home_pos = msg.geo

    def auto_takeoff(self):
        self.wait4connect()
        self.wait4start()
        

        if self.current_state_g.mode == "GUIDED":
            print("Flight mode set to guided successfully.")

            while not rospy.is_shutdown():
                # ox,oy,oz=pm.geodetic2enu(self.follower_home_pos.latitude,self.follower_home_pos.longitude,0,self.leader_home_pos.latitude,self.leader_home_pos.longitude,0)
                # if(ox<0 and oy>0) and (ox<0 and oy<0):
                #     pos_vec_angle=math.pi+math.atan2(oy,ox)
                # else:
                #     pos_vec_angle=math.atan2(oy,ox)

                # leader_yaw_new=math.radians(90.0-self.leader_states.heading)

                # pos_vec_angle=rad_to_360(pos_vec_angle)
                # leader_yaw_new=rad_to_360(leader_yaw_new)

                # print(pos_vec_angle)
                # print(leader_yaw_new)

                if self.current_state_g.connected:
                    # if (math.sqrt(ox**2+oy**2) > 24 and math.sqrt(ox**2+oy**2)<26) and ( ((pos_vec_angle-leader_yaw_new)>145 and (pos_vec_angle-leader_yaw_new)<155) or ((pos_vec_angle-leader_yaw_new)>-215 and (pos_vec_angle-leader_yaw_new)<-205) ):
                    if not self.current_state_g.armed:
                        if not arm_drone('ditto_2'):
                            break
                        else:
                            altitude = 10
                            if send_takeoff_command('ditto_2', altitude):
                                print("Took off")
                                break
                            else:
                                break
                self.rate.sleep()
            time.sleep(10)

def rad_to_360(angle):
    if angle<0:
        return (math.degrees(2*math.pi+angle))
    else:
        return (math.degrees(angle))

if __name__ == '__main__':
    try:
        rospy.init_node('autotakeoff_node2')
        controller = DroneController()
        controller.auto_takeoff()
        controller.control_loop()
        
    except rospy.ROSInterruptException:
        pass
