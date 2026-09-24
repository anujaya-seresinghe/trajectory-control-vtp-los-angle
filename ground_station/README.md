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
| `uav/<sysid>/trajectory_status` | bridge → web | JSON `{id, state: "uploading"\|"done"\|"error", sent, total, error?}` |
| `bridge/status` | bridge → web | JSON `{online}`. It is retained, and the MQTT last will sets it to offline. |

Trajectory uploads are spaced 2 ms apart (`--upload-interval MS`). PX4's `TrajectoryManager` reads the `continuous_trajectory_setpoint` uORB topic, which has queue depth 1, so a burst could drop points.

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
- **Trajectory Flight Mode** does what `tests/waypoint_sender/set_on_traj_flight_mode.py` does: it sends `MAV_CMD_DO_SET_MODE` with main mode AUTO (4) and sub mode TRAJ (9) to the selected vehicle, then shows the `COMMAND_ACK` result, or "No ACK received" after 3 s. The Mode row shows the change once the next heartbeat arrives.
- **Plan trajectory** (2D only): enter an **ID** (0–255) and click the map to add points. The path is a curvature-continuous cubic spline through them.
  - Drag a point to move it. Drag its square handles to change the curvature, and the opposite handle mirrors the move. A handle you have set turns pink. Right-click a handle to make it automatic again, or right-click a point to delete it. Ctrl+Z removes the last point.
  - The side panel shows the length, duration, number of trajectory points, and the peak lateral acceleration and jerk.
  - **Finish** samples the path every 0.5 m (plus the end point) at a constant 16 m/s and sends it to the selected vehicle through the bridge. The uploaded path stays on the map as a dashed line.
  - Each point carries `x, y` and `vx, vy` in NED (16 m/s along the path's direction) and `t` = s / V. `at` and `jt` are the **y components of the acceleration and jerk in the flight-path frame** (x along the velocity, y to the right, z down). With heading γ = atan2(vE, vN) and signed curvature κ = dγ/ds:
    - `at` = V·dγ/dt = V²·κ, positive when turning right and negative when turning left;
    - `jt` = d(at)/dt = V³·dκ/ds.
  - With automatic handles the curvature, and so `at`, is continuous. A handle you drag fixes the tangent at that point, so `at` can step there. `jt` steps at every control point, because a cubic spline's curvature derivative is piecewise.
- More than one vehicle (by MAVLink sysid) is supported. Choose which one to follow in the side panel.

Axis mapping in 3D: three.js `(x, y, z) = (E, −D, −N)`.
