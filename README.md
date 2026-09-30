Trajectory controller based on the approach presented in [Chen et al. (2019)](https://doi.org/10.1016/j.ast.2019.02.034) with the assumption that the virtual target is not moving independent of the UAV. 


![Overview](docs/img/overview1.gif)
![Overview](docs/img/overview2.gif)

The implementation uses a terminal non-singular sliding mode controller with lateral acceleration as the output. 

PX4 flight mode is created for quadcopters since fixed wing versions do not accept acceleration setpoints. A MAVLink library is created to send waypoints. It starts by initiating the trajectory with the message TRAJECTORY_SETPOINT_INITIATE which specifies how many waypoints the trajectory contains and TRAJECTORY_SETPOINT_UPLOAD is used to send each waypoit.

Each waypoint contains: 
- index
- x in NED
- y in NED
- vx in NED
- vy in NED
- magnitude of laeral acceleration
- magnitude of jerk
- timestamp 

The architecture:
Trajectory manager is a PX4 module running with the trajectory flight mode to receive incoming trajectory setpoints and determine the virtual target point based on the position of the UAV. This is implemented as a separate module to reduce the load on the flight mode itself which performs tasks related to the sliding mode controller.

![Architecture](docs/img/architecture.png)




## Setting up
Build the PX4 docker container and inside the container:
docker compose -f docker-compose-px4.yaml up -d --build
docker exec -it px4_gazebo_container bash


Run PX4 on Gazebo x500 SITL. Perform a takeoff and set the flight mode to Trajectory and send the waypoints.

To watch the UAV's position live in 2D or 3D, run the web ground station. It connects to PX4 through a C++ MAVLink↔MQTT bridge. See [ground_station/README.md](ground_station/README.md).

For a VTOL in fixed-wing flight, the same guidance runs as a ROS 2 external mode, with no PX4 changes. See [ros2/README.md](ros2/README.md).

## Quick start: SITL + ground station in one command

```bash
./run_sim.sh                # multicopter (Gazebo x500, headless) + web GCS
./run_sim.sh --vtol         # standard VTOL (Gazebo, headless) + web GCS + ROS 2 fixed-wing Trajectory mode
./run_sim.sh --vtol --sih   # the same with PX4's lightweight built-in simulator instead of Gazebo
./run_sim.sh stop           # stop PX4 SITL, the GCS and (if running) the ROS 2 mode; containers are kept
```

Everything runs **detached**: the containers with `docker compose up -d`, and PX4 SITL as a daemon in the background. The script returns once PX4 reports ready for takeoff and prints the web GCS address (`http://localhost:<WEB_PORT>`, 8080 or the port set in `.env`). While it runs:

```bash
./run_sim.sh logs                     # follow PX4's output
./run_sim.sh px4 commander status     # any PX4 shell command (commander, listener, param, ...)
```

Gazebo runs headless by default; add `--gui` to see its window. `--build` rebuilds the images after code changes.

## References

Chen, Q., Wang, X., Yang, J., & Wang, Z. (2019). Trajectory-following guidance based on a virtual target and an angle constraint. Aerospace Science and Technology, 87, 448–458. https://doi.org/10.1016/j.ast.2019.02.034
