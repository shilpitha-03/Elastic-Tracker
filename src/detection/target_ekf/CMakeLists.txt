cmake_minimum_required(VERSION 3.5)
project(target_ekf)

# use C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# ament build system
find_package(ament_cmake REQUIRED)

# ROS 2 equivalents
find_package(rclcpp REQUIRED)
find_package(object_detection_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(message_filters REQUIRED)

# Eigen3 via the ROS2 eigen3_cmake_module
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# include paths
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# ————————————————
# target_ekf_node
# ————————————————
add_executable(target_ekf_node src/target_ekf_node.cpp)
ament_target_dependencies(target_ekf_node
  rclcpp
  object_detection_msgs
  nav_msgs
  message_filters
  Eigen3
)

install(TARGETS target_ekf_node
  DESTINATION lib/${PROJECT_NAME}
)

# ————————————————
# target_ekf_sim_node
# ————————————————
add_executable(target_ekf_sim_node src/target_ekf_sim_node.cpp)
ament_target_dependencies(target_ekf_sim_node
  rclcpp
  object_detection_msgs
  nav_msgs
  message_filters
  Eigen3
)

install(TARGETS target_ekf_sim_node
  DESTINATION lib/${PROJECT_NAME}
)

# enable C++ tests and linters if you add any
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
