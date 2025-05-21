cmake_minimum_required(VERSION 3.5)
project(mapping)

# Use C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_BUILD_TYPE Release)

# ament
find_package(ament_cmake REQUIRED)

# ROS2 dependencies
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(object_detection_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pluginlib REQUIRED)

# message packages for publishers/subscribers
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Group dependencies
set(DEPENDENCIES
  rclcpp
  rclcpp_components
  quadrotor_msgs
  object_detection_msgs
  tf2_ros
  cv_bridge
  PCL
  pcl_conversions
  Eigen3
  pluginlib
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
)

# Core mapping library
add_library(${PROJECT_NAME}_core src/mapping.cc)
ament_target_dependencies(${PROJECT_NAME}_core
  PCL pcl_conversions Eigen3
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# Mapping component (nodelet → component)
add_library(${PROJECT_NAME}_nodelet SHARED src/mapping_nodelet.cpp)
ament_target_dependencies(${PROJECT_NAME}_nodelet ${DEPENDENCIES})
target_include_directories(${PROJECT_NAME}_nodelet PUBLIC
  include
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_nodelet ${PROJECT_NAME}_core)

# Register component
rclcpp_components_register_nodes(${PROJECT_NAME}_nodelet
  "mapping::MappingNodelet"
)

# Visualization node
add_executable(${PROJECT_NAME}_vis_node src/mapping_vis_node.cpp)
ament_target_dependencies(${PROJECT_NAME}_vis_node ${DEPENDENCIES})
target_link_libraries(${PROJECT_NAME}_vis_node ${PROJECT_NAME}_core)

# History path tool
add_executable(visualize_history_path src/visualize_history_path.cpp)
ament_target_dependencies(visualize_history_path ${DEPENDENCIES})
target_link_libraries(visualize_history_path ${PROJECT_NAME}_core)

# Install targets
install(
  TARGETS
    ${PROJECT_NAME}_core
    ${PROJECT_NAME}_nodelet
    ${PROJECT_NAME}_vis_node
    visualize_history_path
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install config, launch, resource
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Export
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_package()
