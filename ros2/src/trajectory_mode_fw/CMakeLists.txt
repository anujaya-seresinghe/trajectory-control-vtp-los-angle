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
find_package(PkgConfig REQUIRED)
pkg_check_modules(MOSQUITTO REQUIRED IMPORTED_TARGET libmosquitto)

# ROS-free guidance core (a port of the PX4-internal Trajectory mode), also used by the unit test
add_library(${PROJECT_NAME}_core STATIC src/vtp_guidance.cpp src/trajectory_json.cpp src/yaml_params.cpp)
target_include_directories(${PROJECT_NAME}_core PUBLIC include)
set_target_properties(${PROJECT_NAME}_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(${PROJECT_NAME} src/main.cpp src/mode.cpp src/mqtt_trajectory_source.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE include ${EIGEN3_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_core px4_ros2_cpp::px4_ros2_cpp rclcpp::rclcpp
  ${std_msgs_TARGETS} ${px4_msgs_TARGETS} PkgConfig::MOSQUITTO)

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_vtp_guidance test/test_vtp_guidance.cpp)
  target_link_libraries(test_vtp_guidance ${PROJECT_NAME}_core)
endif()

ament_package()
