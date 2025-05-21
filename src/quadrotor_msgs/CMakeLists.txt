cmake_minimum_required(VERSION 3.5)
project(quadrotor_msgs)

# use C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# message files
set(MSG_FILES
  msg/OccMap3d.msg
  msg/PolyTraj.msg
  msg/ReplanState.msg
  msg/PositionCommand.msg
  msg/SO3Command.msg
  msg/AuxCommand.msg
  msg/Px4ctrlDebug.msg
  msg/TakeoffLand.msg
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  DEPENDENCIES
    geometry_msgs
    nav_msgs
    builtin_interfaces
    std_msgs
)

# export dependencies so downstream packages can use these messages
ament_export_dependencies(rosidl_default_runtime geometry_msgs nav_msgs)

# install message definitions
install(
  DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
