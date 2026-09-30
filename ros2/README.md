# Trajectory mode for fixed-wing flight (ROS 2 external mode)

`trajectory_mode_fw` is an external PX4 flight mode named **"Trajectory"** for a VTOL in fixed-wing flight (or a plain fixed-wing). It runs the same virtual-target sliding-mode guidance as the PX4-internal Trajectory mode, and needs **no PX4 changes**. It is built on [px4-ros2-interface-lib](https://github.com/Auterion/px4-ros2-interface-lib) and uXRCE-DDS.

```
web app ─MQTT─ bridge ─MAVLink─► PX4 ─forwarding (MAVLink instance with -f)─► trajectory_mode_fw (compid 191)
                                  ▲                                               │
                                  └──────────────── uXRCE-DDS (setpoints) ◄───────┘
                                                    TrajectoryManager + FlightTaskTraj port → fw_lateral_longitudinal_control
```

## What it does

| Internal PX4 mode (multicopter) | This mode (fixed-wing) |
|---|---|
| `TrajectoryManager` picks the virtual target | Same algorithm, ported to C++ without ROS (`vtp_guidance.cpp`): the r\* rule, a search window of 100, and the same fallback to index 0 |
| `FlightTaskTraj` sliding-mode law → `a_m`, `a_long` | Same equations and clamps, in `float`. Unit tests check them against an independent port. |
| `a_m` and `a_long` → NED acceleration setpoint | Fixed-wing controllers don't use NED acceleration, so they are converted: |
| — lateral `a_m` (flight-path y, positive right) | → `FixedWingLateralSetpoint.lateral_acceleration` (FRD y, the same axis in coordinated flight). PX4 limits it to `tan(FW_R_LIM)·g` and commands roll = atan(a/g). |
| — longitudinal `a_long = clamp(k_long (v_t − v), ±a_long_max)` | → drives a speed reference, which becomes an **equivalent airspeed** setpoint through the wind triangle and PX4's EAS/TAS ratio. TECS tracks it. |
| — altitude | → held at the AMSL altitude it had when the mode was activated (`FixedWingLongitudinalSetpoint.altitude`) |

Where it has to differ:
- **Everything to and from the ground station is MAVLink, like the internal mode.** PX4 doesn't export the `continuous_trajectory_*` uORB topics over DDS, so the node is its own MAVLink component (sysid 1, compid 191, "onboard computer") and PX4 forwards the ground station's messages to it:
  - the same broadcast `TRAJECTORY_SETPOINT_INITIATE` + `TRAJECTORY_SETPOINT_UPLOAD` the internal mode uses; complete when all indices have arrived, confirmed with the STATUSTEXT `Trajectory <id> received (<n> points)`;
  - the MAVLink parameter protocol for `TRAJ_*` (`PARAM_REQUEST_READ/LIST`, `PARAM_SET` → `PARAM_VALUE`, a rejected set is reported with a STATUSTEXT first);
  - the guidance state as `NAMED_VALUE_FLOAT` at 5 Hz: `TRAJ_STATE` (0 inactive, 1 holding course, 2 tracking), `TRAJ_R`, `TRAJ_IDX`, `TRAJ_N`, `TRAJ_A_M`, `TRAJ_ALONG`. The web app shows them in the **Virtual target** block. QGC's MAVLink Inspector shows them too.

  It needs **its own PX4 MAVLink instance with forwarding**, because PX4 never forwards back to the instance a message came from and the bridge uses the SITL onboard link. `run_sim.sh --vtol` starts it (runtime only, no PX4 change): `mavlink start -u 14591 -o 14590 -r 4000000 -m minimal -f`. On hardware, configure a `MAV_n_CONFIG` instance for the companion computer with `MAV_n_FORWARD = 1`.
- A ROS topic also accepts trajectories: `~/trajectory_json` (`std_msgs/String`, same JSON).
- **Nothing to fly yet**, no valid position, or no target in range (for example after the end of the path): a fixed-wing can't stop and wait, so it flies straight on its last course at the held altitude. The internal mode would publish a non-finite setpoint here.
- **Registration waits for fixed-wing flight.** PX4 only accepts fixed-wing setpoints while `vehicle_type == FIXED_WING`, and the library checks this once at registration, so the mode appears in PX4 after the first transition to fixed-wing. It is also unavailable (arming/run check) whenever the vehicle is not in fixed-wing flight.
- As in the internal mode, the target restarts at the first point on every activation. A trajectory that arrives while the mode is inactive is used at the next activation.

## Run

```bash
docker compose -f docker-compose-ros2.yaml up -d --build
```

The image contains:
- the Micro XRCE-DDS Agent (UDP 8888, which PX4 SITL's `uxrce_dds_client` connects to);
- `px4_msgs` generated from **this project's** PX4 messages (`ec1c6a4901` plus `PX4/msg`, which changes the versioned `VehicleStatus.msg`);
- px4-ros2-interface-lib **2.2.0**, the last release before that PX4 commit;
- this package. Its unit tests run during the build.

It uses host networking (XRCE-DDS on UDP 8888, MAVLink on UDP 14590 ↔ PX4 14591). If you already run an agent, set `START_XRCE_AGENT=0`. Logs: `docker logs -f trajectory_mode_fw`.

In flight: take off, **transition to fixed-wing**, and wait for `Mode "Trajectory" registered` in the log. Send the trajectory, then select the mode. It is the first external mode, so `commander mode ext1` works in the PX4 shell, and QGC lists it by name.

## Parameters

These are ROS 2 parameters, separate from PX4's `TRAJ_*` parameters, which only the internal multicopter mode uses. All of them can be changed while flying:
- **Web app:** for a VTOL, the **Controller parameters** panel is labelled "(ROS 2 fixed-wing mode)". It reads and sets them with MAVLink parameter messages addressed to compid 191, through the bridge (`cmd/param_get`/`cmd/param_set` with `"component": 191` → `param`m`). For a multicopter it still uses PX4's `TRAJ_*` over MAVLink, as before.
- **Command line:**
  ```bash
  docker exec trajectory_mode_fw bash -c 'source /ws/install/setup.bash && ros2 param set /trajectory_mode_fw traj_r_star 30.0'
  ```

Values are checked against the same ranges as the PX4 parameters, and an out-of-range value is rejected with the reason. **Every accepted change is saved** to `ros2/src/trajectory_mode_fw/config/params.yaml`: only that value is rewritten, and comments stay. `docker-compose-ros2.yaml` mounts that file into the container, so the node loads the saved values at the next start and after a rebuild.

| Parameter | Default | Same as |
|---|---|---|
| `traj_smc_alpha`, `traj_smc_beta`, `traj_smc_eps` | 1.4, 0.12, 0.05 | `TRAJ_SMC_ALPHA/BETA/EPS` |
| `traj_r_star` | 15 m | `TRAJ_R_STAR` |
| `traj_k_long`, `traj_a_long_max` | 0.5, 0.3 m/s² | `TRAJ_K_LONG`, `TRAJ_A_LONG_MAX` |
| `traj_a_m_max` | 2 m/s² | `TRAJ_A_M_MAX` |
| `traj_search_window` | 100 | search window in `TrajectoryManager` |
| `sysid`, `mavlink_compid` | 1, 191 | the MAVLink identity of the mode |
| `mavlink_local_port`, `mavlink_remote_host`, `mavlink_remote_port` | 14590, 127.0.0.1, 14591 | its PX4 MAVLink instance |

Current values (and defaults for new setups) are in `src/trajectory_mode_fw/config/params.yaml`.

## Tested

In PX4 SIH, standard VTOL (`10043`) at `ec1c6a4901`, the flight went: take off → transition → deferred registration → a 1505 m trajectory from the web planner, with a 300 m straight and a 300 m-radius U-turn. The mode tracked the whole path within **0.2–8.6 m** and held altitude within 3 m.

**The planned speed must be reachable.** The planner uses 16 m/s. The SIH VTOL's default `FW_AIRSPD_MAX = 12 m/s` makes the virtual target outrun the vehicle, and the guidance then diverges in turns. That happens with the same equations in the internal mode too, whenever the vehicle is slower than v_t. Keep the plan speed between `FW_AIRSPD_MIN` and `FW_AIRSPD_MAX`.
