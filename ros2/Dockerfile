# ROS 2 external PX4 flight mode "Trajectory" (fixed-wing) + Micro XRCE-DDS Agent.
# Build context: repository root (needs ros2/src and PX4/msg).
FROM ros:jazzy-ros-base

# Must match the PX4 commit in DockerfilePX4
ARG PX4_COMMIT=ec1c6a4901
# Last px4-ros2-interface-lib release before that PX4 commit
ARG PX4_ROS2_LIB_VERSION=2.2.0
ARG XRCE_AGENT_VERSION=v2.4.3

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
        git build-essential cmake pkg-config libmosquitto-dev libeigen3-dev \
        ros-jazzy-eigen3-cmake-module ros-jazzy-ament-cmake-gtest \
    && rm -rf /var/lib/apt/lists/*

# Micro XRCE-DDS Agent (PX4's uxrce_dds_client connects to it on UDP 8888)
RUN git clone --depth 1 -b ${XRCE_AGENT_VERSION} https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/agent \
    && cmake -S /tmp/agent -B /tmp/agent/build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build /tmp/agent/build -j"$(nproc)" \
    && cmake --install /tmp/agent/build && ldconfig && rm -rf /tmp/agent

# px4_msgs generated from this project's PX4: upstream msg/ + srv/ at PX4_COMMIT, then the PX4/msg overlay
# (the overlay changes the versioned VehicleStatus.msg, so stock px4_msgs would fail the message check)
WORKDIR /ws/src
RUN git clone --depth 1 https://github.com/PX4/px4_msgs.git \
    && rm -f px4_msgs/msg/*.msg px4_msgs/srv/*.srv \
    && git clone --filter=blob:none --no-checkout https://github.com/PX4/PX4-Autopilot.git /tmp/px4 \
    && cd /tmp/px4 && git sparse-checkout set msg srv && git checkout ${PX4_COMMIT} \
    && cp msg/*.msg msg/versioned/*.msg /ws/src/px4_msgs/msg/ && cp srv/*.srv /ws/src/px4_msgs/srv/ \
    && rm -rf /tmp/px4
COPY PX4/msg /tmp/px4_msg_overlay
RUN cp /tmp/px4_msg_overlay/*.msg /ws/src/px4_msgs/msg/ \
    && if ls /tmp/px4_msg_overlay/versioned/*.msg >/dev/null 2>&1; then cp /tmp/px4_msg_overlay/versioned/*.msg /ws/src/px4_msgs/msg/; fi \
    && rm -rf /tmp/px4_msg_overlay

RUN git clone --depth 1 -b ${PX4_ROS2_LIB_VERSION} https://github.com/Auterion/px4-ros2-interface-lib.git

# Dependencies first (slow, cached), then the mode itself
WORKDIR /ws
RUN source /opt/ros/jazzy/setup.bash \
    && colcon build --packages-up-to px4_ros2_cpp --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

COPY ros2/src/trajectory_mode_fw /ws/src/trajectory_mode_fw
RUN source /opt/ros/jazzy/setup.bash \
    && colcon build --packages-select trajectory_mode_fw --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && colcon test --packages-select trajectory_mode_fw --ctest-args -R test_vtp_guidance \
    && colcon test-result --verbose

COPY ros2/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
