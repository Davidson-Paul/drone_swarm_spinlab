
# DRONE SWARM
## Path Creation

### About
Find scripts for creating equally-spaced path position-setpoints, considering yaw-rate constraint.

- Yaw-rate constraint on the leader drone arises due to the velocity constraint of 10m/s in the drones, because of which the leader has to make heading change limitedly so that follower don't move beyond 10m/s.
- yaw_rate_bound = max_vel/max_formation_spacing
- For max_vel=10m/s and max_formation_spacing=50m, yaw_rate_bound=0.2 rad/s

### Further Development
- Add velocity, acceleration, yaw, yaw-rate setpoints along with position
- Need to make this max yaw-rate bounded path creation automatic.