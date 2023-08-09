#!/bin/bash


gnome-terminal \
 --tab -e "sim_vehicle.py --mavproxy-args='--streamrate=20' --sysid=1 -v ArduCopter -f gazebo-drone1 --console --map -L iitk -I 0" \
 --tab -e "sim_vehicle.py --mavproxy-args='--streamrate=20' --sysid=2 -v ArduCopter -f gazebo-drone2 --console --map -L iitk -I 1" \
 --tab -e "sim_vehicle.py --mavproxy-args='--streamrate=20' --sysid=3 -v ArduCopter -f gazebo-drone3 --console --map -L iitk -I 2" \
 --tab -e "sim_vehicle.py --mavproxy-args='--streamrate=20' --sysid=4 -v ArduCopter -f gazebo-drone4 --console --map -L iitk -I 3" \
 --tab -e "sim_vehicle.py --mavproxy-args='--streamrate=20' --sysid=5 -v ArduCopter -f gazebo-drone5 --console --map -L iitk -I 4" \


sleep 20s

gnome-terminal \
 --tab -- bash -c "roslaunch mavros apmsitl.launch; exec bash"

sleep 1s
gnome-terminal \
 --tab -- bash -c "roslaunch mavros apmsitl_i1.launch; exec bash"
 
sleep 1s
gnome-terminal \
 --tab -- bash -c "roslaunch mavros apmsitl_i2.launch; exec bash"

sleep 1s
gnome-terminal \
 --tab -- bash -c "roslaunch mavros apmsitl_i3.launch; exec bash"

sleep 1s
gnome-terminal \
 --tab -- bash -c "roslaunch mavros apmsitl_i4.launch; exec bash"

