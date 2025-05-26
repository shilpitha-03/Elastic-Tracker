cmake_minimum_required(VERSION 3.0.2)
project(odom_visualization)

## Find dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  visualization_msgs
  quadrotor_msgs
  tf
  pose_utils
)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(Armadillo REQUIRED)

## Catkin package
catkin_package(
  INCLUDE_DIRS   ${OpenCV_INCLUDE_DIRS}
                 ${EIGEN3_INCLUDE_DIRS}
                 ${Boost_INCLUDE_DIRS}
                 ${ARMADILLO_INCLUDE_DIRS}
  LIBRARIES odom_visualization ${ARMADILLO_LIBRARIES}
  CATKIN_DEPENDS roscpp sensor_msgs nav_msgs visualization_msgs quadrotor_msgs tf pose_utils
  DEPENDS OpenCV Eigen3 Boost Armadillo
)

## Include dirs
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${ARMADILLO_INCLUDE_DIRS}
)

## Executable: odom_visualization
add_executable(odom_visualization src/odom_visualization.cpp)
target_link_libraries(odom_visualization
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
  ${ARMADILLO_LIBRARIES}
  pose_utils
)
add_dependencies(odom_visualization ${catkin_EXPORTED_TARGETS})

## Executable: odom_visualization_car
add_executable(odom_visualization_car src/odom_visualization_car.cpp)
target_link_libraries(odom_visualization_car
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
  ${ARMADILLO_LIBRARIES}
  pose_utils
)
add_dependencies(odom_visualization_car ${catkin_EXPORTED_TARGETS})
