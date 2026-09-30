# Ground station

A web app that shows the UAV's position in 2D or 3D, plus a C++ bridge that carries MAVLink between PX4 and the web app over MQTT.

```
PX4 SITL  <--MAVLink/UDP-->  mavlink_mqtt_bridge (C++)  <--MQTT-->  mosquitto  <--MQTT/WebSocket-->  web app
          14580 <-> 14540                              1883                    9001
```

## Components

| Path | What it does |
|---|---|
| `bridge/` | The C++ MAVLink ↔ MQTT bridge. It uses `libraries/mavlink` and libmosquitto. |
| `bridge/src/fake_vehicle.cpp` | A stand-in for PX4 that flies a figure eight, so you can test without SITL. |
| `web/` | The web app (Vite, three.js, mqtt.js). |
| `../configs/mqtt/mosquitto.conf` | The broker config: TCP on 1883, WebSockets on 9001. |
| `../docker-compose-gcs.yaml` | Runs the broker, the bridge and the web app in Docker. |
| `bridge/Dockerfile`, `web/Dockerfile` | Images for the bridge (built from the repo root) and the web app (nginx). |

## MQTT topics

| Topic | Direction | Payload |
|---|---|---|
| `uav/<sysid>/local_position_ned` | vehicle → web | JSON `{time_boot_ms, x, y, z, vx, vy, vz}` (NED, m, m/s) |
| `uav/<sysid>/attitude` | vehicle → web | JSON `{time_boot_ms, roll, pitch, yaw, rollspeed, pitchspeed, yawspeed}` (rad) |
| `uav/<sysid>/heartbeat` | vehicle → web | JSON `{type, autopilot, base_mode, custom_mode, system_status, armed}` |
| `mavlink/rx` | vehicle → MQTT | Raw MAVLink v2 frame bytes, one message per MQTT message |
| `mavlink/tx` | MQTT → vehicle | Raw MAVLink frame bytes. The bridge checks the CRC, then forwards the frame over UDP. |
| `uav/<sysid>/cmd/trajectory` | web → vehicle | JSON `{"id": 0-255, "points": [[x, y, vx, vy, at, jt, t], ...]}`. The bridge sends `TRAJECTORY_SETPOINT_INITIATE` (id, number of points), then one `TRAJECTORY_SETPOINT_UPLOAD` per point with index 0…N-1 in array order. |
| `uav/<sysid>/cmd/set_mode` | web → vehicle | JSON `{"main_mode", "sub_mode"}` (PX4 custom mode). The bridge sends `COMMAND_LONG` `MAV_CMD_DO_SET_MODE` (param1 = custom mode enabled). |
| `uav/<sysid>/command_ack` | vehicle → web | JSON `{command, result}` from `COMMAND_ACK` addressed to the bridge |
| `uav/<sysid>/cmd/command_long` | web → vehicle | JSON `{"command": n, "params": [p1, ..., p7]}`, where `null` is NaN. The bridge sends `COMMAND_LONG`, and the ACK arrives on `command_ack`. |
| `uav/<sysid>/cmd/command_int` | web → vehicle | JSON `{"command", "frame", "params": [p1..p4], "x", "y", "z"}`. `x`/`y` are integers (degE7 in global frames), and `null` means "not set": `INT32_MAX` for x/y, NaN otherwise. The bridge sends `COMMAND_INT`. |
| `uav/<sysid>/global_position` | vehicle → web | JSON `{time_boot_ms, lat, lon, alt, relative_alt}` (deg, m AMSL, m) from `GLOBAL_POSITION_INT` |
| `uav/<sysid>/extended_sys_state` | vehicle → web | JSON `{vtol_state, landed_state}` from `EXTENDED_SYS_STATE` |
| `uav/<sysid>/available_mode` | vehicle → web | JSON `{index, number_modes, custom_mode, standard_mode, properties, name}` from `AVAILABLE_MODES`, one per mode. The web app uses it to find the ROS 2 mode. |
| `uav/<sysid>/available_modes_monitor` | vehicle → web | JSON `{seq}`. It changes when modes are added or removed, for example when the ROS 2 mode registers. |
| `uav/<sysid>/statustext` | vehicle → web | JSON `{severity, text, component}` from `STATUSTEXT`, for example the reason arming was denied |
| `uav/<sysid>/cmd/param_get` | web → vehicle | JSON `{"names": ["TRAJ_R_STAR", ...], "component"?: n}`. The bridge sends one `PARAM_REQUEST_READ` per name, to the autopilot or to `component` (191 = the ROS 2 fixed-wing mode). |
| `uav/<sysid>/cmd/param_set` | web → vehicle | JSON `{"name": "TRAJ_R_STAR", "value": 30, "component"?: n}`. The bridge sends `PARAM_SET` (float parameters). |
| `uav/<sysid>/param` | vehicle → web | JSON `{name, value, type, index, count, component}` from every `PARAM_VALUE` |
| `uav/<sysid>/named_value` | vehicle → web | JSON `{name, value, time_boot_ms, component}` from `NAMED_VALUE_FLOAT`, e.g. the ROS 2 mode's `TRAJ_STATE`, `TRAJ_R`, `TRAJ_IDX`, `TRAJ_N`, `TRAJ_A_M`, `TRAJ_ALONG` |
| `uav/<sysid>/trajectory_status` | bridge → web | JSON `{id, state: "uploading"\|"done"\|"error", sent, total, error?}` |
| `bridge/status` | bridge → web | JSON `{online}`. It is retained, and the MQTT last will sets it to offline. |

Trajectory uploads are spaced 5 ms apart (`--upload-interval MS`). PX4's `TrajectoryManager` reads the `continuous_trajectory_setpoint` uORB topic, which has queue depth 1, so a burst could drop points. For a VTOL, PX4 also forwards the upload to the ROS 2 mode; PX4 forwards at most one message per MAVLink loop with a 2-message buffer, and 2 ms lost points in SITL while 3 and 5 ms did not.

The ROS 2 fixed-wing mode is a MAVLink component (sysid 1, compid 191) on its own PX4 MAVLink instance, so everything reaches it through PX4 like the internal mode: the same trajectory upload, `PARAM_*` addressed to compid 191, and its `STATUSTEXT`/`NAMED_VALUE_FLOAT` back. See [ros2/README.md](../ros2/README.md).

When the bridge first sees a vehicle's heartbeat, it asks PX4 for `LOCAL_POSITION_NED` and `ATTITUDE` at 30 Hz with `MAV_CMD_SET_MESSAGE_INTERVAL`. Every second it sends its own GCS heartbeat (sysid 255).

## Setup

### Docker (recommended)

`docker-compose-gcs.yaml` runs three containers:
- the broker (`configs/mqtt/mosquitto.conf`);
- the bridge, on host networking so it can reach PX4 SITL over UDP;
- the web app, served by nginx.

```bash
docker compose -f docker-compose-gcs.yaml up -d --build
```

Then open http://localhost:8080.

Ports are set with environment variables, or with a `.env` file next to the compose file:

| Variable | Default | Meaning |
|---|---|---|
| `MQTT_PORT` | 1883 | Broker TCP port on the host (the bridge connects here) |
| `MQTT_WS_PORT` | 9001 | Broker WebSocket port on the host (built into the web app as its default) |
| `WEB_PORT` | 8080 | Web app port |
| `MAVLINK_UDP_LISTEN` | 14540 | UDP port the bridge listens on |
| `MAVLINK_UDP_TARGET` | 127.0.0.1:14580 | Where the bridge sends until it has heard from the vehicle |

If a system mosquitto already uses 1883, or another service uses 8080:

```bash
MQTT_PORT=11883 MQTT_WS_PORT=19001 WEB_PORT=8088 docker compose -f docker-compose-gcs.yaml up -d --build
```

Pass the same variables to `down`, `logs` and later `up` commands; a `.env` file saves repeating them. `MQTT_WS_PORT` is compiled into the web app, so rebuild (`--build`) after changing it.

To test without PX4, the bridge image includes the fake vehicle:

```bash
docker run --rm --network host --entrypoint fake_vehicle trajectory-control-vtp-los-angle-bridge
```

### Native

Dependencies (Ubuntu): `sudo apt install cmake g++ libmosquitto-dev`, plus Node.js 18 or newer.

```bash
# 1. MQTT broker (only this container)
docker compose -f docker-compose-gcs.yaml up -d mosquitto

# 2. Bridge
cmake -S ground_station/bridge -B ground_station/bridge/build
cmake --build ground_station/bridge/build -j
ground_station/bridge/build/mavlink_mqtt_bridge          # --help for options

# 3. Web app → http://localhost:5173
cd ground_station/web && npm install && npm run dev
```

To test without PX4, run `ground_station/bridge/build/fake_vehicle` in place of SITL. With non-default broker ports, run the bridge with `--mqtt-port`, and open the app with `?broker=ws://localhost:<ws port>` or type the URL in the top bar, where it is remembered.

### PX4

Start PX4 SITL (see the main README). The bridge's default ports match the PX4 SITL API link: PX4 sends from 14580 to 14540. The bridge replies to the address the vehicle's packets come from, so a different instance or host also works: point `MAVLINK_UDP_LISTEN` / `--udp-listen` at the port PX4 sends to.

## Web app

- **2D**: a top-down map with North up and East right. Drag to pan (this turns Follow off) and scroll to zoom. It shows the grid, the NED origin, the trail, the heading arrow (from `ATTITUDE.yaw`), the 1-second velocity vector and the altitude label.
- **3D**: an orbit camera with the quad model posed from roll/pitch/yaw, the trail, the trail's ground projection and an altitude stalk. The model is drawn at 4× real size so it stays visible. three.js loads only when the 3D view is first opened.
- The side panel shows the flight mode (decoded from PX4 `custom_mode`, including *Trajectory*), armed state, position rate, NED position and velocity, altitude, ground speed, course and attitude.
- **Takeoff**: set the altitude above the current position (default 2.5 m) and press **Takeoff**. If PX4's `MIS_TAKEOFF_ALT` differs, the new value is written to that parameter first. The app then arms (`MAV_CMD_COMPONENT_ARM_DISARM`) if needed, and sends `MAV_CMD_NAV_TAKEOFF` with no position or altitude, so PX4 climbs `MIS_TAKEOFF_ALT` above where the UAV is. If PX4 rejects a step, the status line shows the result and PX4's last status text (for example, a failed preflight check). The button refuses if the UAV is already armed and more than 1 m up.
- **VTOLs** (heartbeat type VTOL, or a defined VTOL state) get VTOL-specific behaviour:
  - The vehicle panel shows the VTOL state: Multicopter, Fixed-wing or Transition.
  - a **Fixed-wing** button appears, and the takeoff altitude defaults to 40 m. Take off with **Takeoff** first; once the climb has finished (PX4 ignores a transition during Takeoff), **Fixed-wing** sends `MAV_CMD_DO_VTOL_TRANSITION` and waits for fixed-wing flight.
  - **Plan trajectory** and **Trajectory Flight Mode** activate the **ROS 2 "Trajectory" mode** ([ros2/](../ros2/README.md)) instead of PX4's internal multicopter mode. The app finds that mode's number in PX4's `AVAILABLE_MODES`. It registers only in fixed-wing flight, so before the transition the buttons explain this instead. While you plan, the VTOL flies straight on its course. **Finish** requires the ROS 2 mode to be active, and the status line confirms when the mode has loaded the trajectory. The plan speed (16 m/s) must be within the airframe's `FW_AIRSPD_MIN`…`FW_AIRSPD_MAX`.
- **Go here** (2D map, outside planning): right-click a point and press **Go here**. The app converts the point to lat/lon from the UAV's current `GLOBAL_POSITION_INT` and local position (flat earth, PX4's Earth radius), then sends `MAV_CMD_DO_REPOSITION` as `COMMAND_INT` with the change-mode flag and altitude "keep current". PX4 switches to **Hold** and flies there. A crosshair marks the target until the UAV is within 1.5 m or leaves Hold. The UAV must be armed.
- **Exit to Hold** switches the UAV to PX4 Hold (AUTO / LOITER) with `MAV_CMD_DO_SET_MODE`, for example to leave Trajectory mode.
- **Trajectory Flight Mode** does what `tests/waypoint_sender/set_on_traj_flight_mode.py` does: it sends `MAV_CMD_DO_SET_MODE` with main mode AUTO (4) and sub mode TRAJ (9) to the selected vehicle, then shows the `COMMAND_ACK` result, or "No ACK received" after 3 s. The Mode row shows the change once the next heartbeat arrives.
- **Controller parameters**: shows `TRAJ_R_STAR`, `TRAJ_SMC_ALPHA`, `TRAJ_SMC_BETA`, `TRAJ_SMC_EPS`, `TRAJ_K_LONG` and the acceleration limits `TRAJ_A_M_MAX` / `TRAJ_A_LONG_MAX`, read from the vehicle when it connects. Edit values, which turn amber until sent, then press **Apply**. Each change is confirmed by the value PX4 reports back, and **Refresh** re-reads everything. PX4 applies new values immediately, with no rebuild.
- **Plan trajectory** (2D only): first switches the UAV to Trajectory mode, exactly like the button above, and opens the planner only once the vehicle accepts. PX4's trajectory manager only exists in that mode, so it can receive the upload. Until a trajectory arrives, PX4 brakes to a hover ("Failsafe: stop and wait"). **Finish** refuses to upload if the UAV has left Trajectory mode meanwhile. Enter an **ID** (0–255) and click the map to add points. The path is a curvature-continuous cubic spline through them.
  - Drag a point to move it. Drag its square handles to change the curvature, and the opposite handle mirrors the move. A handle you have set turns pink. Right-click a handle to make it automatic again, or right-click a point to delete it. Ctrl+Z removes the last point.
  - The side panel shows the length, duration, number of trajectory points, and the peak lateral acceleration and jerk.
  - **Finish** samples the path every 0.5 m (plus the end point) at a constant 16 m/s and sends it to the selected vehicle through the bridge. The uploaded path stays on the map as a dashed line.
  - Each point carries `x, y` and `vx, vy` in NED (16 m/s along the path's direction) and `t` = s / V. `at` and `jt` are the **y components of the acceleration and jerk in the flight-path frame** (x along the velocity, y to the right, z down). With heading γ = atan2(vE, vN) and signed curvature κ = dγ/ds:
    - `at` = V·dγ/dt = V²·κ, positive when turning right and negative when turning left;
    - `jt` = d(at)/dt = V³·dκ/ds.
  - With automatic handles the curvature, and so `at`, is continuous. A handle you drag fixes the tangent at that point, so `at` can step there. `jt` steps at every control point, because a cubic spline's curvature derivative is piecewise.
- More than one vehicle (by MAVLink sysid) is supported. Choose which one to follow in the side panel.

Axis mapping in 3D: three.js `(x, y, z) = (E, −D, −N)`.
