#!/bin/bash
# Starts the Micro XRCE-DDS Agent (unless START_XRCE_AGENT=0) and the "Trajectory" fixed-wing mode.
set -e
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash

if [ "${START_XRCE_AGENT:-1}" = "1" ]; then
  MicroXRCEAgent udp4 -p "${XRCE_AGENT_PORT:-8888}" &
fi

# docker-compose-ros2.yaml mounts the repository's config/params.yaml here, so parameter changes made at runtime
# (web app parameter panel, ros2 param set) are written back to it and survive restarts and rebuilds
PARAMS_FILE=/ws/install/trajectory_mode_fw/share/trajectory_mode_fw/config/params.yaml

exec ros2 run trajectory_mode_fw trajectory_mode_fw --ros-args \
  --params-file "$PARAMS_FILE" -p params_file:="$PARAMS_FILE" \
  -p sysid:="${MAV_SYSID:-1}" "$@"
