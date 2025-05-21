cmake_minimum_required(VERSION 3.5)
project(object_detection_msgs)

# If you ever add C++ code, compile as C++11
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

# ament build system and interface generators
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# —————————————————————————————
# 1 Declare your message files
# —————————————————————————————
set(MSG_FILES
  msg/BoundingBox.msg
  msg/BoundingBoxes.msg
  msg/ObjectCount.msg
  msg/CarPosition.msg
  msg/MotorAngle.msg
)

# —————————————————————————————
# 2 Generate ROS 2 interfaces
# —————————————————————————————
rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  DEPENDENCIES std_msgs
)

# —————————————————————————————
# 3 (Optional) Services & Actions stubs
# —————————————————————————————
# If you later add srv/*.srv or action/*.action, uncomment & mirror above:
# set(SRV_FILES srv/Service1.srv srv/Service2.srv)
# set(ACTION_FILES action/Action1.action action/Action2.action)
# rosidl_generate_interfaces(${PROJECT_NAME}
#   ${MSG_FILES} ${SRV_FILES} ${ACTION_FILES}
#   DEPENDENCIES std_msgs
# )

# —————————————————————————————
# 4 Export for downstream
# —————————————————————————————
ament_export_dependencies(rosidl_default_runtime std_msgs)

# —————————————————————————————
# 5 Install message definitions
# —————————————————————————————
install(
  DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}
)

# —————————————————————————————
# 6 Testing
# —————————————————————————————
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # automatically add lint and style tests (ament_lint)
  ament_lint_auto_find_test_dependencies()

  # Example C++ test (if you add test/test_*.cpp):
  # find_package(ament_cmake_gtest REQUIRED)
  # ament_add_gtest(test_object_detection_msgs test/test_object_detection_msgs.cpp)
endif()

ament_package()
