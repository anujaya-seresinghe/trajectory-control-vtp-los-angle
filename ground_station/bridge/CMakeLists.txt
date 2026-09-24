cmake_minimum_required(VERSION 3.16)
project(mavlink_mqtt_bridge CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(PkgConfig REQUIRED)
pkg_check_modules(MOSQUITTO REQUIRED IMPORTED_TARGET libmosquitto)
find_package(Threads REQUIRED)

# Directory that contains mavlink/common/mavlink.h
set(MAVLINK_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../libraries)

add_executable(mavlink_mqtt_bridge src/main.cpp src/bridge.cpp src/trajectory.cpp)
target_include_directories(mavlink_mqtt_bridge PRIVATE ${MAVLINK_INCLUDE_DIR})
target_link_libraries(mavlink_mqtt_bridge PRIVATE PkgConfig::MOSQUITTO Threads::Threads)
# The generated MAVLink headers take the address of packed struct members.
target_compile_options(mavlink_mqtt_bridge PRIVATE -Wall -Wextra -Wno-address-of-packed-member)

# Fake vehicle that streams LOCAL_POSITION_NED / ATTITUDE / HEARTBEAT over UDP,
# useful for testing the bridge and web app without PX4 SITL.
add_executable(fake_vehicle src/fake_vehicle.cpp)
target_include_directories(fake_vehicle PRIVATE ${MAVLINK_INCLUDE_DIR})
target_compile_options(fake_vehicle PRIVATE -Wall -Wextra -Wno-address-of-packed-member)
