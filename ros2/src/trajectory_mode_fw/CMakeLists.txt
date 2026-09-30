cmake_minimum_required(VERSION 3.8)
project(trajectory_mode_fw)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_ros2_cpp REQUIRED)

# Directory that contains mavlink/common/mavlink.h (this repository's libraries/, with the TRAJECTORY_SETPOINT_* messages)
set(MAVLINK_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../libraries CACHE PATH "MAVLink C headers")
if(NOT EXISTS ${MAVLINK_INCLUDE_DIR}/mavlink/common/mavlink.h)
  message(FATAL_ERROR "mavlink/common/mavlink.h not found in MAVLINK_INCLUDE_DIR=${MAVLINK_INCLUDE_DIR}")
endif()

# ROS-free guidance core (a port of the PX4-internal Trajectory mode), also used by the unit test
add_library(${PROJECT_NAME}_core STATIC src/vtp_guidance.cpp src/acquisition.cpp src/fw_controller.cpp
  src/trajectory_json.cpp src/yaml_params.cpp)
target_include_directories(${PROJECT_NAME}_core PUBLIC include)
set_target_properties(${PROJECT_NAME}_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(${PROJECT_NAME} src/main.cpp src/mode.cpp src/mavlink_link.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE include ${EIGEN3_INCLUDE_DIR})
target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${MAVLINK_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_core px4_ros2_cpp::px4_ros2_cpp rclcpp::rclcpp
  ${std_msgs_TARGETS} ${px4_msgs_TARGETS})

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_vtp_guidance test/test_vtp_guidance.cpp)
  target_include_directories(test_vtp_guidance PRIVATE test)
  target_link_libraries(test_vtp_guidance ${PROJECT_NAME}_core)
endif()

ament_package()
