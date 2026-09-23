Trajectory controller based on the approach presented in [Chen et al. (2019)](https://doi.org/10.1016/j.ast.2019.02.034) with the assumption that the virtual target is not moving independent of the UAV. 

The controller uses a terminal non-singular sliding mode controller with lateral acceleration as the output. 

PX4 flight mode is created for quadcopters since fixed wing versions do not accept acceleration setpoints. A MAVLink library is created to send waypoints. It starts by initiating the trajectory with the message TRAJECTORY_SETPOINT_INITIATE which specifies how many waypoints the trajectory contains and TRAJECTORY_SETPOINT_UPLOAD is used to send each waypoit.

Each waypoint contains: 
- index
- x in NED
- y in NED
- vx in NED
- vy in NED
- magnitude of laeral acceleration in NED
- maginutude of jerk in NED
- timestamp 

The architecture:
Trajectory manager is a PX4 module running with the flight mode to receieve incoming trajectory setpoints and determine the virtual target point based on the position of the UAV. This is implemented as a separate module to reduce the load on the flight mode itself which performs tasks related to the sliding mode controller.

![Architecture](docs/img/architecture.png)




## Setting up
Run the PX4 docker container and inside the container run PX4 on Gazebo x500 SITL. Perform a takeoff and set the flight mode to Trajectory and send the waypoints.

## References

Chen, Q., Wang, X., Yang, J., & Wang, Z. (2019). Trajectory-following guidance based on a virtual target and an angle constraint. Aerospace Science and Technology, 87, 448–458. https://doi.org/10.1016/j.ast.2019.02.034
