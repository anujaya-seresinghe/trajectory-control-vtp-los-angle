#!/usr/bin/env bash
# Start PX4 SITL and the web GCS (broker + MAVLink bridge + web app) with one command.
#
#   ./run_sim.sh              multicopter (Gazebo x500, headless)
#   ./run_sim.sh --vtol       standard VTOL (Gazebo standard_vtol, headless) + the ROS 2 fixed-wing Trajectory mode
#   ./run_sim.sh stop         stop PX4 SITL, the GCS and (if running) the ROS 2 mode
#   ./run_sim.sh logs         follow PX4's output
#   ./run_sim.sh px4 CMD...   run a PX4 shell command, e.g. ./run_sim.sh px4 commander status
#
# Options:
#   --vtol       VTOL instead of multicopter; also starts docker-compose-ros2.yaml (trajectory_mode_fw)
#   --sih        PX4's built-in simulator (SIH) instead of Gazebo: much lighter
#   --gui        show the Gazebo window (Gazebo runs headless by default)
#   --build      rebuild the Docker images (after code changes)
#   -h, --help   this help
#
# Everything runs detached (docker -d, PX4 as a daemon); the script returns once PX4 is ready.
set -euo pipefail

cd "$(dirname "$(readlink -f "$0")")"

# The three compose files share this directory's project name; don't warn about each other's containers
export COMPOSE_IGNORE_ORPHANS=1

PX4_COMPOSE=(docker compose -f docker-compose-px4.yaml)
GCS_COMPOSE=(docker compose -f docker-compose-gcs.yaml)
ROS_COMPOSE=(docker compose -f docker-compose-ros2.yaml)
PX4_CONTAINER=px4_gazebo_container
PX4_BUILD_DIR=/app/PX4-Autopilot/build/px4_sitl_default
PX4_LOG=/tmp/px4_sitl.log   # inside the PX4 container

case "${1:-}" in
  logs)
    exec docker exec "$PX4_CONTAINER" tail -n 200 -f "$PX4_LOG" ;;
  px4)
    shift
    if [ $# -eq 0 ]; then
      echo "usage: ./run_sim.sh px4 <command> [args], e.g. ./run_sim.sh px4 commander status" >&2
      exit 1
    fi
    cmd=$1
    shift
    exec docker exec "$PX4_CONTAINER" "$PX4_BUILD_DIR/bin/px4-$cmd" "$@" ;;
esac

VTOL=0 SIH=0 HEADLESS=1 BUILD=() STOP=0
for arg in "$@"; do
  case "$arg" in
    stop|--down) STOP=1 ;;
    --vtol) VTOL=1 ;;
    --sih) SIH=1 ;;
    --gui) HEADLESS=0 ;;
    --headless) HEADLESS=1 ;;   # default; kept for compatibility
    --build) BUILD=(--build) ;;
    -h|--help) sed -n '2,17p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "Unknown option: $arg (see --help)" >&2; exit 1 ;;
  esac
done

stop_sitl() {
  # Stop a previous PX4/Gazebo instance inside the container (the container itself keeps running)
  docker exec "$PX4_CONTAINER" bash -c 'pkill -x px4 || true; pkill -f "gz sim" || true' >/dev/null 2>&1 || true
}

running() { [ "$(docker inspect -f '{{.State.Running}}' "$1" 2>/dev/null)" = true ]; }

if [ "$STOP" = 1 ]; then
  # stop, not down: removing the PX4 container would also delete SITL's saved parameters (parameters.bson)
  echo "==> Stopping PX4 SITL"
  stop_sitl
  if running trajectory_mode_fw; then
    echo "==> Stopping the ROS 2 fixed-wing Trajectory mode"
    "${ROS_COMPOSE[@]}" stop
  fi
  echo "==> Stopping the ground station"
  "${GCS_COMPOSE[@]}" stop
  echo "==> Stopping the PX4 container"
  "${PX4_COMPOSE[@]}" stop
  exit 0
fi

command -v docker >/dev/null || { echo "docker is not installed" >&2; exit 1; }

# Vehicle / simulator
if [ "$SIH" = 1 ]; then
  TARGET=$([ "$VTOL" = 1 ] && echo sihsim_standard_vtol || echo sihsim_quadx)
  SIM_ENV=(PX4_SIMULATOR=sihsim)
else
  TARGET=$([ "$VTOL" = 1 ] && echo gz_standard_vtol || echo gz_x500)
  SIM_ENV=(GZ_IP=127.0.0.1)
fi

WEB_PORT=$(grep -E '^WEB_PORT=' .env 2>/dev/null | cut -d= -f2 || true)
WEB_PORT=${WEB_PORT:-8080}

echo "==> Ground station (broker, bridge, web app)"
"${GCS_COMPOSE[@]}" up -d "${BUILD[@]}"

if [ "$VTOL" = 1 ]; then
  echo "==> ROS 2 fixed-wing Trajectory mode + XRCE-DDS agent"
  "${ROS_COMPOSE[@]}" up -d "${BUILD[@]}"
else
  # The fixed-wing mode is only for VTOLs; don't leave it running for a multicopter
  "${ROS_COMPOSE[@]}" stop >/dev/null 2>&1 || true
fi

echo "==> PX4 container"
"${PX4_COMPOSE[@]}" up -d "${BUILD[@]}"
stop_sitl

if [ "$SIH" = 0 ] && [ "$HEADLESS" = 0 ] && [ -n "${DISPLAY:-}" ] && command -v xhost >/dev/null; then
  xhost +local:docker >/dev/null 2>&1 || true   # let Gazebo open its window
fi

ENVS=()
[ "$HEADLESS" = 1 ] && ENVS+=(-e HEADLESS=1)

# PX4 as a daemon (px4 -d, no interactive shell) in a detached exec; same environment as `make px4_sitl <target>`
echo "==> PX4 SITL (${TARGET}), detached"
docker exec -d "${ENVS[@]}" "$PX4_CONTAINER" bash -c \
  "cd ${PX4_BUILD_DIR} && env PX4_SIM_MODEL=${TARGET} ${SIM_ENV[*]} ./bin/px4 -d > ${PX4_LOG} 2>&1"

printf 'Waiting for PX4'
ready=0
for _ in $(seq 1 90); do
  if docker exec "$PX4_CONTAINER" grep -q "Ready for takeoff" "$PX4_LOG" 2>/dev/null; then
    ready=1
    break
  fi
  if ! docker exec "$PX4_CONTAINER" pgrep -x px4 >/dev/null 2>&1; then
    echo
    echo "PX4 exited. Last lines of its log:" >&2
    docker exec "$PX4_CONTAINER" tail -n 20 "$PX4_LOG" >&2 || true
    exit 1
  fi
  printf '.'
  sleep 2
done
if [ "$ready" = 1 ]; then echo " ready for takeoff"; else echo " still starting (see ./run_sim.sh logs)"; fi

HEADLESS_NOTE=""
[ "$SIH" = 0 ] && [ "$HEADLESS" = 1 ] && HEADLESS_NOTE=" (headless)"

echo
echo "  Web GCS:    http://localhost:${WEB_PORT}"
echo "  Vehicle:    ${TARGET}${HEADLESS_NOTE}"
echo "  PX4 log:    ./run_sim.sh logs"
echo "  PX4 shell:  ./run_sim.sh px4 commander status   (any PX4 command)"
[ "$VTOL" = 1 ] && echo "  ROS 2 log:  docker logs -f trajectory_mode_fw"
echo "  Stop all:   ./run_sim.sh stop"
echo
