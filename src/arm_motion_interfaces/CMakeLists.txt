cmake_minimum_required(VERSION 3.8)
project(arm_motion_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JointPosition.msg"
  "msg/MotionStep.msg"
  "msg/MotionSummary.msg"
  "srv/SaveMotion.srv"
  "srv/LoadMotion.srv"
  "srv/ListMotions.srv"
  "srv/DeleteMotion.srv"
  "srv/JogJoint.srv"
  "action/PlayMotion.action"
  DEPENDENCIES action_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
