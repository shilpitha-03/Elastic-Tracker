cmake_minimum_required(VERSION 3.0.2)
project(local_sensing_node)

## Compile flags
set(CMAKE_BUILD_TYPE Release)         # Release, RelWithDebInfo, etc.
add_compile_options(-std=c++14)

## Option for CUDA
option(ENABLE_CUDA "Enable CUDA support" ON)

if(ENABLE_CUDA)
  # Point CMake at your CUDA installation if not already set
  if(NOT DEFINED CUDA_TOOLKIT_ROOT_DIR)
    set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda)
  endif()

  find_package(CUDA REQUIRED)
  # NVCC flags
  set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -O3 -use_fast_math")
  list(APPEND CUDA_NVCC_FLAGS
    "-gencode arch=compute_61,code=sm_61"
    # add more -gencode lines here if you target additional architectures
  )
  set(CUDA_PROPAGATE_HOST_FLAGS OFF)
endif()

## ROS and 3rd-party deps
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  cmake_modules
  cv_bridge
  image_transport
  pcl_ros
  sensor_msgs
  geometry_msgs
  nav_msgs
  dynamic_reconfigure
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)

## Dynamic reconfigure
generate_dynamic_reconfigure_options(
  cfg/local_sensing_node.cfg
)

## Catkin package export
catkin_package(
  DEPENDS OpenCV Eigen3 Boost
  CATKIN_DEPENDS
    roscpp
    roslib
    cv_bridge
    image_transport
    pcl_ros
    sensor_msgs
    geometry_msgs
    nav_msgs
    dynamic_reconfigure
  LIBRARIES depth_render_cuda
)

## Include paths
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
)

if(ENABLE_CUDA)
  ###############
  # CUDA branch #
  ###############

  ## Build CUDA helper library
  cuda_add_library(depth_render_cuda
    src/depth_render.cu
    OPTIONS ${CUDA_NVCC_FLAGS}
  )

  ## Main node
  add_executable(pcl_render_node
    src/pcl_render_node.cpp
  )
  target_link_libraries(pcl_render_node
    depth_render_cuda
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
  )
  add_dependencies(pcl_render_node
    ${catkin_EXPORTED_TARGETS}
  )

else()
  ####################
  # CPU-only fallback #
  ####################

  add_executable(pcl_render_node
    src/pointcloud_render_node.cpp
  )
  target_link_libraries(pcl_render_node
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
  )
  add_dependencies(pcl_render_node
    ${catkin_EXPORTED_TARGETS}
  )
endif()
