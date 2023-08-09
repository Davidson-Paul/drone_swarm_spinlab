
# DRONE SWARM
## Evaluating Feasibility of Mission Paths Considering Hardware Constraints for Multi-UAV Control and Coordination

The overall project focuses on the implementation of multi-UAV control and coordination, targeting the
practical applications of UAV swarms. My part is to implement and test formation-control algorithms for the swarm. Both centralised and distributed algorithms are considered and simulated. The simulations are conducted using the iris-drone model in Gazebo,
specifically for a V-shaped formation. The focus is on analysing the performance and effectiveness of the
control algorithms.



### Structure

**common_scripts** contains scripts for plotting various parameter measurements to be analysed.

**consensus** contains scripts and results related to distributed consensus tracking algorithms

**leader_follower_mission** contains scripts and results on centralised leader-follwer scheme and outdoor testing

**setup_files** contains files needed to setup simulation in ArduPilot-SITL and Gazebo


