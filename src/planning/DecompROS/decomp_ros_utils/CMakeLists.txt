cmake_minimum_required(VERSION 3.5)
project(decomp_ros_utils)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(decomp_ros_msgs REQUIRED)
find_package(decomp_basis REQUIRED)
find_package(decomp_geometry REQUIRED)

add_library(decomp_rviz_plugins SHARED
  src/bound_visual.cpp
  src/mesh_visual.cpp
  src/vector_visual.cpp
  src/ellipsoid_array_visual.cpp
  src/ellipsoid_array_display.cpp
  src/polyhedron_array_display.cpp
  src/objects/mesh_shape.cpp          # vendored MeshShape
)

# expose all of include/, which now contains:
#   – decomp_rviz_plugins/*
#   – rviz_rendering/objects/mesh_shape.hpp
#   – rviz_common/frame_manager.hpp
target_include_directories(decomp_rviz_plugins
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(decomp_rviz_plugins
  rclcpp
  pluginlib
  rviz_common
  rviz_rendering
  Eigen3
  decomp_ros_msgs
  decomp_basis
  decomp_geometry
)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS decomp_rviz_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(FILES plugin_description.xml DESTINATION share/${PROJECT_NAME})

ament_package()
