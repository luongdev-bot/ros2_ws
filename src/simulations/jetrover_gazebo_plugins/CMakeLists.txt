cmake_minimum_required(VERSION 3.8)
project(jetrover_gazebo_plugins)

find_package(ament_cmake REQUIRED)
find_package(ignition-gazebo6 REQUIRED)

add_library(jetrover-grasp-velocity-system SHARED
  src/grasp_velocity_system.cc
)
target_compile_features(jetrover-grasp-velocity-system PUBLIC cxx_std_17)
target_include_directories(jetrover-grasp-velocity-system PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(jetrover-grasp-velocity-system
  ignition-gazebo6::core
)

install(DIRECTORY include/ DESTINATION include)

install(TARGETS jetrover-grasp-velocity-system
  LIBRARY DESTINATION lib
)

ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/ign_gazebo_system_plugin_path.dsv.in"
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mimic_joint_targets
    test/test_mimic_joint_targets.cc
  )
  target_include_directories(test_mimic_joint_targets PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
  )
endif()

ament_package()
