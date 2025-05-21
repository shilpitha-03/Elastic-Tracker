cmake_minimum_required(VERSION 3.5)
project(planning)

# Use C++14 standard
add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

# Find necessary packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nodelet2 REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(traj_opt REQUIRED)
find_package(mapping REQUIRED)
find_package(decomp_ros_utils REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${DECOMP_UTIL_INCLUDE_DIRS}
)

# Planning node library
add_library(${PROJECT_NAME}_nodelet SHARED
  src/${PROJECT_NAME}_nodelet.cpp
)

ament_target_dependencies(${PROJECT_NAME}_nodelet
  rclcpp
  quadrotor_msgs
  traj_opt
  mapping
  decomp_ros_utils
  Eigen3
)

# Executable nodes
add_executable(traj_server src/traj_server.cpp)
ament_target_dependencies(traj_server
  rclcpp
  quadrotor_msgs
  traj_opt
  mapping
  decomp_ros_utils
)

add_executable(play_bag_node src/play_bag_node.cpp)
ament_target_dependencies(play_bag_node
  rclcpp
  quadrotor_msgs
  traj_opt
  mapping
  decomp_ros_utils
)

add_executable(test_node src/test_node.cpp)
ament_target_dependencies(test_node
  rclcpp
  quadrotor_msgs
  traj_opt
  mapping
  decomp_ros_utils
)

# Install targets
install(TARGETS
  ${PROJECT_NAME}_nodelet
  traj_server
  play_bag_node
  test_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
