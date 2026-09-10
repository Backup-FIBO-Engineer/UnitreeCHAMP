cmake_minimum_required(VERSION 3.8)
project(champ_base_gait_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-std=c++17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(champ_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(urdf REQUIRED)
find_package(champ REQUIRED)
find_package(message_filters REQUIRED)

set(dependencies
  rclcpp
  champ_msgs
  visualization_msgs
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  nav_msgs
  tf2
  urdf
  message_filters
  champ
)

include_directories(include)
include_directories(SYSTEM
  ${champ_INCLUDE_DIRS}
  ${champ_INCLUDE_DIRS}/champ/
)

add_library(quadruped_controller SHARED src/quadruped_controller.cpp)
ament_target_dependencies(quadruped_controller ${dependencies})
add_executable(quadruped_controller_node src/quadruped_controller_node.cpp)
target_link_libraries(quadruped_controller_node quadruped_controller)
ament_target_dependencies(quadruped_controller_node ${dependencies})

add_library(state_estimation SHARED src/state_estimation.cpp)
ament_target_dependencies(state_estimation ${dependencies})
add_executable(state_estimation_node src/state_estimation_node.cpp)
target_link_libraries(state_estimation_node state_estimation)
ament_target_dependencies(state_estimation_node ${dependencies})

find_package(Threads REQUIRED)
find_package(unitree_sdk2 QUIET)
if(NOT unitree_sdk2_FOUND)
  find_path(UNITREE_SDK2_INCLUDE_DIR
    NAMES unitree/robot/channel/channel_publisher.hpp
    PATHS /usr/local/include /opt/unitree_robotics/include
          $ENV{HOME}/unitree_sdk2/include)
  find_library(UNITREE_SDK2_LIB
    NAMES unitree_sdk2
    PATHS /usr/local/lib /opt/unitree_robotics/lib
          $ENV{HOME}/unitree_sdk2/lib)
  if(UNITREE_SDK2_INCLUDE_DIR AND UNITREE_SDK2_LIB)
    set(unitree_sdk2_FOUND TRUE)
  endif()
endif()

set(UNITREE_DDS_BRIDGE_BUILT FALSE)
if(unitree_sdk2_FOUND)
  add_executable(unitree_dds_bridge
    src/unitree_dds_bridge.cpp
    src/motor_crc.cpp
  )
  ament_target_dependencies(unitree_dds_bridge rclcpp sensor_msgs champ_msgs)
  if(TARGET unitree_sdk2)
    target_link_libraries(unitree_dds_bridge unitree_sdk2 Threads::Threads)
  else()
    target_include_directories(unitree_dds_bridge PRIVATE ${UNITREE_SDK2_INCLUDE_DIR})
    target_link_libraries(unitree_dds_bridge ${UNITREE_SDK2_LIB} Threads::Threads)
  endif()
  set(UNITREE_DDS_BRIDGE_BUILT TRUE)
  message(STATUS "unitree_sdk2 found: building unitree_dds_bridge (Go2/B2 rt/lowcmd Sim2Real)")
else()
  message(WARNING
    "unitree_sdk2 not found; skip unitree_dds_bridge. Install unitree_sdk2 to run "
    "Go2/B2 hardware (ros2 launch champ_base_gait_planning b2_sim2real.launch.py).")
endif()

install(TARGETS
  quadruped_controller
  state_estimation
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

set(EXEC_TARGETS
  quadruped_controller_node
  state_estimation_node
)
if(UNITREE_DDS_BRIDGE_BUILT)
  list(APPEND EXEC_TARGETS unitree_dds_bridge)
endif()
install(TARGETS ${EXEC_TARGETS}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  meshes
  urdf
  rviz
  mujoco
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  enable_testing()
  add_executable(verify_champ_xgo tools/verify_champ_xgo.cpp)
  target_include_directories(verify_champ_xgo PRIVATE
    ${champ_INCLUDE_DIRS}
    ${champ_INCLUDE_DIRS}/champ
  )
  add_test(NAME verify_champ_xgo COMMAND verify_champ_xgo)
  add_test(
    NAME verify_urdf_champ_contract
    COMMAND python3
      ${CMAKE_CURRENT_SOURCE_DIR}/tools/verify_urdf_champ_contract.py
      ${CMAKE_CURRENT_SOURCE_DIR}/urdf/xgo_rviz.xacro
      ${CMAKE_CURRENT_SOURCE_DIR}/config/xgo_links.yaml
      ${CMAKE_CURRENT_SOURCE_DIR}/config/xgo_joints.yaml
  )
  add_test(
    NAME verify_mujoco_physics
    COMMAND python3
      ${CMAKE_CURRENT_SOURCE_DIR}/tools/verify_mujoco_physics.py
      ${CMAKE_CURRENT_SOURCE_DIR}/mujoco/xgo.xml
  )
  add_test(
    NAME validate_mujoco_against_urdf
    COMMAND python3
      ${CMAKE_CURRENT_SOURCE_DIR}/tools/validate_mujoco_against_urdf.py
      ${CMAKE_CURRENT_SOURCE_DIR}/urdf/xgo_rviz.xacro
      ${CMAKE_CURRENT_SOURCE_DIR}/mujoco/xgo.xml
  )
  add_executable(verify_champ_go2 tools/verify_champ_go2.cpp)
  target_include_directories(verify_champ_go2 PRIVATE
    ${champ_INCLUDE_DIRS}
    ${champ_INCLUDE_DIRS}/champ
  )
  add_test(NAME verify_champ_go2 COMMAND verify_champ_go2)
  add_executable(verify_unitree_lowcmd tools/verify_unitree_lowcmd.cpp src/motor_crc.cpp)
  target_include_directories(verify_unitree_lowcmd PRIVATE include)
  add_test(NAME verify_unitree_lowcmd COMMAND verify_unitree_lowcmd)
  add_test(
    NAME verify_urdf_champ_contract_go2
    COMMAND python3
      ${CMAKE_CURRENT_SOURCE_DIR}/tools/verify_urdf_champ_contract.py
      ${CMAKE_CURRENT_SOURCE_DIR}/urdf/go2.urdf
      ${CMAKE_CURRENT_SOURCE_DIR}/config/go2_links.yaml
      ${CMAKE_CURRENT_SOURCE_DIR}/config/go2_joints.yaml
      --preset go2
  )
  add_test(
    NAME verify_mujoco_physics_go2
    COMMAND python3
      ${CMAKE_CURRENT_SOURCE_DIR}/tools/verify_mujoco_physics.py
      ${CMAKE_CURRENT_SOURCE_DIR}/mujoco/go2.xml
  )
  add_test(
    NAME validate_mujoco_against_urdf_go2
    COMMAND python3
      ${CMAKE_CURRENT_SOURCE_DIR}/tools/validate_mujoco_against_urdf.py
      ${CMAKE_CURRENT_SOURCE_DIR}/urdf/go2.urdf
      ${CMAKE_CURRENT_SOURCE_DIR}/mujoco/go2.xml
      --robot go2
  )
  add_test(
    NAME verify_go2_gait_yaml
    COMMAND python3
      ${CMAKE_CURRENT_SOURCE_DIR}/tools/verify_gait_yaml.py go2
  )
  add_test(
    NAME verify_mujoco_walk_go2
    COMMAND python3
      ${CMAKE_CURRENT_SOURCE_DIR}/tools/verify_mujoco_walk.py
      --robot go2 0.35
  )

  add_executable(verify_champ_b2 tools/verify_champ_b2.cpp)
  target_include_directories(verify_champ_b2 PRIVATE
    ${champ_INCLUDE_DIRS}
    ${champ_INCLUDE_DIRS}/champ
  )
  add_test(NAME verify_champ_b2 COMMAND verify_champ_b2)
  add_test(
    NAME verify_urdf_champ_contract_b2
    COMMAND python3
      ${CMAKE_CURRENT_SOURCE_DIR}/tools/verify_urdf_champ_contract.py
      ${CMAKE_CURRENT_SOURCE_DIR}/urdf/b2.urdf
      ${CMAKE_CURRENT_SOURCE_DIR}/config/b2_links.yaml
      ${CMAKE_CURRENT_SOURCE_DIR}/config/b2_joints.yaml
      --preset b2
  )
  add_test(
    NAME verify_mujoco_physics_b2
    COMMAND python3
      ${CMAKE_CURRENT_SOURCE_DIR}/tools/verify_mujoco_physics.py
      ${CMAKE_CURRENT_SOURCE_DIR}/mujoco/b2.xml
  )
  add_test(
    NAME validate_mujoco_against_urdf_b2
    COMMAND python3
      ${CMAKE_CURRENT_SOURCE_DIR}/tools/validate_mujoco_against_urdf.py
      ${CMAKE_CURRENT_SOURCE_DIR}/urdf/b2.urdf
      ${CMAKE_CURRENT_SOURCE_DIR}/mujoco/b2.xml
      --robot b2
  )
  add_test(
    NAME verify_b2_gait_yaml
    COMMAND python3
      ${CMAKE_CURRENT_SOURCE_DIR}/tools/verify_gait_yaml.py b2
  )
  add_test(
    NAME verify_mujoco_walk_b2
    COMMAND python3
      ${CMAKE_CURRENT_SOURCE_DIR}/tools/verify_mujoco_walk.py
      --robot b2 0.35
  )
endif()

install(PROGRAMS
  mujoco/mujoco_sim.py
  tools/verify_urdf_champ_contract.py
  tools/verify_mujoco_physics.py
  tools/validate_mujoco_against_urdf.py
  tools/generate_unitree_mjcf.py
  tools/verify_gait_yaml.py
  tools/verify_mujoco_walk.py
  tools/run_offline_checks.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()

