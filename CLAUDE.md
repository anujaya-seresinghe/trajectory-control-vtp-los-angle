# CLAUDE.md

Trajectory following for PX4 UAVs with virtual-target (VTP) sliding-mode guidance (Chen et al. 2019). A planned 2D
path is uploaded point by point over MAVLink; the guidance outputs a lateral acceleration `a_m` and a longitudinal
acceleration `a_long`.

## Layout

| Path | What |
|---|---|
| `PX4/` | **Overlay** on PX4-Autopilot `ec1c6a4901`: only the modified/added files. `DockerfilePX4` clones that commit and copies this folder over it. |
| `PX4/src/modules/flight_mode_manager/tasks/Traj/` | Internal PX4 mode *Trajectory* (AUTO main 4 / sub 9), multicopter only: `FlightTaskTraj` (SMC law) + `trajectory_manager/TrajectoryManager` (virtual target selection). Params in `flight_task_traj_params.yaml` (`TRAJ_*`). |
| `PX4/msg/` | `ContinuousTrajectory*.msg` uORB topics, modified versioned `VehicleStatus.msg` |
| `libraries/mavlink/` | MAVLink v2 C headers; the `common` dialect contains the custom `TRAJECTORY_SETPOINT_INITIATE` (602) and `TRAJECTORY_SETPOINT_UPLOAD` (601) |
| `ground_station/bridge/` | C++ MAVLink (UDP) ↔ MQTT bridge (libmosquitto). `fake_vehicle.cpp` is a fake PX4 for tests. |
| `ground_station/web/` | Vite web GCS (three.js 3D, canvas 2D, mqtt.js): telemetry, trajectory planner, takeoff/fixed-wing/go-here/hold, parameter panel |
| `ros2/src/trajectory_mode_fw/` | ROS 2 Jazzy external PX4 mode *Trajectory* for VTOLs in fixed-wing flight (px4-ros2-interface-lib 2.2.0). Same guidance ported ROS-free in `vtp_guidance.cpp`; `a_m` → FW lateral acceleration, `a_long` → speed reference → EAS. **No PX4 changes.** Talks MAVLink as compid 191 on its own PX4 MAVLink instance (PX4 forwards the GCS's upload and `TRAJ_*` params to it); guidance state as `NAMED_VALUE_FLOAT`. |
| `configs/mqtt/mosquitto.conf` | Broker config |
| `run_sim.sh` | PX4 SITL + GCS (+ ROS 2 mode with `--vtol`) in one command |
| `tests/waypoint_sender/`, `scripts/` | Test/utility scripts |

## Running

```bash
./run_sim.sh                # Gazebo x500 (headless), GCS
./run_sim.sh --vtol         # Gazebo standard_vtol + ROS 2 fixed-wing mode; --sih for SIH, --gui for Gazebo window, --build to rebuild
./run_sim.sh stop           # stop SITL, GCS, ROS 2 (containers are kept)
./run_sim.sh logs           # PX4 output;  ./run_sim.sh px4 <cmd>  runs a PX4 shell command
```

- Web GCS: http://localhost:8088 (ports in `.env`: `MQTT_PORT=11883`, `MQTT_WS_PORT=19001`, `WEB_PORT=8088`).
- Compose files: `docker-compose-px4.yaml` (`px4_gazebo_container`, PX4 runs as daemon `px4 -d`, log `/tmp/px4_sitl.log`),
  `docker-compose-gcs.yaml` (`gcs_mqtt_broker`, `gcs_bridge`, `gcs_web`), `docker-compose-ros2.yaml` (`trajectory_mode_fw` + XRCE-DDS agent). All host networking except the broker.
- Web only: `npm --prefix ground_station/web run build` (or `dev`, see `.claude/launch.json`); redeploy with
  `docker compose -f docker-compose-gcs.yaml up -d --build web`.
- ROS 2 node: the image build compiles it and runs its gtests (`docker compose -f docker-compose-ros2.yaml build`).

## Conventions and gotchas

- **The user makes PX4 changes themselves** unless they explicitly ask. The multicopter path (internal PX4 mode, PX4 `TRAJ_*` params over MAVLink) must stay as it is; fixed-wing work goes into the ROS 2 node.
- **Never `docker compose down` the PX4 compose**: removing `px4_gazebo_container` deletes SITL's `parameters.bson`. Use `stop`.
- **Check running containers before testing** (`docker ps`) and ask before restarting anything the user may be flying with; restarting `trajectory_mode_fw` mid-flight drops the external mode. Rebuilding `gcs_web` alone is harmless.
- Use `127.0.0.1`, not `localhost`, for MQTT (libmosquitto tries IPv6 `::1` first; the broker is IPv4).
- The `common` dialect here lacks the `MAV_CMD` enum: use numeric command IDs (DO_SET_MODE 176, ARM 400, NAV_TAKEOFF 22, DO_REPOSITION 192, REQUEST_MESSAGE 512, SET_MESSAGE_INTERVAL 511, DO_VTOL_TRANSITION 3000).
- Coordinates are NED. The planner resamples every 0.5 m at a constant 16 m/s; `at = V²κ` is **signed** (flight-path y, positive = right turn), `jt = V³ dκ/ds`, `t = s/V`.
- The ROS 2 mode registers only once the VTOL is in fixed-wing flight (PX4 rejects FW setpoint types otherwise). The web app finds it through `AVAILABLE_MODES`; old QGC versions show it as "Unknown".
- Fixed-wing tracking needs the plan speed within `FW_AIRSPD_MIN..MAX`, and the vehicle must pass within r* of point 0 to lock on (with `a_m ≤ 2 m/s²` at 16 m/s the turn radius is ~128 m).
- The bridge's upload pacing (5 ms) is limited by PX4 forwarding (one forwarded message per MAVLink loop, 2-message buffer); 2 ms drops points for the ROS 2 mode.
- ROS 2 parameter changes are written back to `ros2/src/trajectory_mode_fw/config/params.yaml` (mounted into the container).
- Match the surrounding style: PX4-style C++ (tabs) in `PX4/` and `ros2/`, ES modules without a framework in the web app.
