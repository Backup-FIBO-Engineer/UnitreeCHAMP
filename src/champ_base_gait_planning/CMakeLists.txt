cmake_minimum_required(VERSION 3.8)
project(champ_base_gait_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-std=c++17)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
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

# IMU-closed body roll/pitch between the user's body_pose and CHAMP
# (include/body_orientation_controller.h is the ROS-free loop, also verified
# offline by tools/verify_body_pose_controller.cpp).
add_library(body_pose_controller SHARED src/body_pose_controller.cpp)
ament_target_dependencies(body_pose_controller rclcpp geometry_msgs sensor_msgs urdf)
add_executable(body_pose_controller_node src/body_pose_controller_node.cpp)
target_link_libraries(body_pose_controller_node body_pose_controller)
ament_target_dependencies(body_pose_controller_node rclcpp geometry_msgs sensor_msgs urdf)

# ---------------------------------------------------------------------------
# unitree_ros2_bridge: any Unitree quadruped speaking unitree_go LowCmd/LowState
# on /lowcmd + /lowstate through unitree_ros2 (same ROS 2 / CycloneDDS graph as
# CHAMP). Needs the unitree_go and unitree_api message packages from
# https://github.com/unitreerobotics/unitree_ros2 (cyclonedds_ws/src/unitree),
# either built in this workspace or sourced from unitree_ros2/cyclonedds_ws.
# ---------------------------------------------------------------------------
find_package(Threads REQUIRED)
find_package(unitree_go QUIET)
find_package(unitree_api QUIET)

set(UNITREE_ROS2_BRIDGE_BUILT FALSE)
if(unitree_go_FOUND AND unitree_api_FOUND)
  add_executable(unitree_ros2_bridge
    src/unitree_ros2_bridge.cpp
    src/motor_crc.cpp
  )
  ament_target_dependencies(unitree_ros2_bridge
    rclcpp sensor_msgs champ_msgs urdf unitree_go unitree_api)
  target_link_libraries(unitree_ros2_bridge Threads::Threads)
  set(UNITREE_ROS2_BRIDGE_BUILT TRUE)
  message(STATUS "unitree_go + unitree_api found: building unitree_ros2_bridge (LowCmd Sim2Real)")
else()
  message(WARNING
    "unitree_go/unitree_api (unitree_ros2) not found; skip unitree_ros2_bridge. Build or source "
    "unitree_ros2/cyclonedds_ws to run Unitree hardware "
    "(ros2 launch champ_base_gait_planning unitree_sim2real.launch.py robot:=<robot>).")
endif()

install(TARGETS
  quadruped_controller
  state_estimation
  body_pose_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

set(EXEC_TARGETS
  quadruped_controller_node
  state_estimation_node
  body_pose_controller_node
)
if(UNITREE_ROS2_BRIDGE_BUILT)
  list(APPEND EXEC_TARGETS unitree_ros2_bridge)
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

# Robot file resolution shared by the launch files and the offline tools.
ament_python_install_module(tools/champ_robot_files.py)
# URDF visual mesh (dae/stl) -> MuJoCo OBJ conversion used by generate_mjcf.py.
# Installed next to champ_robot_files.py with a plain install(): the
# ament_cmake_python shipped with Jazzy rejects a second top-level
# ament_python_install_module() call (it keys duplicates on the destination
# directory), and one call already registers the PYTHONPATH hook.
install(FILES tools/champ_mesh_assets.py DESTINATION ${PYTHON_INSTALL_DIR})

# ---------------------------------------------------------------------------
# Tests: one set per robot found in urdf/ (<robot>.urdf or <robot>.xacro).
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
  enable_testing()
  find_package(yaml-cpp REQUIRED)
  find_package(tinyxml2_vendor REQUIRED)
  find_package(TinyXML2 REQUIRED)
  find_package(urdfdom REQUIRED)
  if(TARGET yaml-cpp::yaml-cpp)
    set(CHAMP_YAML_CPP_TARGET yaml-cpp::yaml-cpp)
  else()
    set(CHAMP_YAML_CPP_TARGET yaml-cpp)
  endif()

  set(CHAMP_TOOL_INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}/tools
    ${champ_INCLUDE_DIRS}
    ${champ_INCLUDE_DIRS}/champ
  )
  add_executable(verify_champ_robot tools/verify_champ_robot.cpp)
  target_include_directories(verify_champ_robot PRIVATE ${CHAMP_TOOL_INCLUDES})
  target_link_libraries(verify_champ_robot ${CHAMP_YAML_CPP_TARGET} tinyxml2::tinyxml2)

  add_executable(dump_champ_gait tools/dump_champ_gait.cpp)
  target_include_directories(dump_champ_gait PRIVATE ${CHAMP_TOOL_INCLUDES})
  target_link_libraries(dump_champ_gait ${CHAMP_YAML_CPP_TARGET} tinyxml2::tinyxml2)

  add_executable(verify_unitree_lowcmd tools/verify_unitree_lowcmd.cpp src/motor_crc.cpp)
  target_include_directories(verify_unitree_lowcmd PRIVATE include)
  target_link_libraries(verify_unitree_lowcmd ${CHAMP_YAML_CPP_TARGET})
  ament_target_dependencies(verify_unitree_lowcmd urdfdom)

  add_executable(verify_body_pose_controller tools/verify_body_pose_controller.cpp)
  target_include_directories(verify_body_pose_controller PRIVATE include ${CHAMP_TOOL_INCLUDES})
  target_link_libraries(verify_body_pose_controller ${CHAMP_YAML_CPP_TARGET} tinyxml2::tinyxml2)

  add_executable(measure_gait_continuity tools/measure_gait_continuity.cpp)
  target_include_directories(measure_gait_continuity PRIVATE include ${CHAMP_TOOL_INCLUDES})
  target_link_libraries(measure_gait_continuity ${CHAMP_YAML_CPP_TARGET} tinyxml2::tinyxml2)

  set(TOOLS ${CMAKE_CURRENT_SOURCE_DIR}/tools)
  file(GLOB ROBOT_DESCRIPTIONS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/urdf
    ${CMAKE_CURRENT_SOURCE_DIR}/urdf/*.urdf
    ${CMAKE_CURRENT_SOURCE_DIR}/urdf/*.xacro)
  set(EXPANDED_URDFS)
  foreach(description ${ROBOT_DESCRIPTIONS})
    get_filename_component(robot ${description} NAME_WE)
    get_filename_component(extension ${description} EXT)
    if(extension STREQUAL ".xacro")
      # The C++ tools read plain URDF; expand the xacro at build time.
      set(robot_urdf ${CMAKE_CURRENT_BINARY_DIR}/urdf/${robot}.urdf)
      add_custom_command(
        OUTPUT ${robot_urdf}
        COMMAND python3 ${TOOLS}/champ_robot_files.py expand
          ${CMAKE_CURRENT_SOURCE_DIR}/urdf/${description} ${robot_urdf}
        DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/urdf/${description} ${TOOLS}/champ_robot_files.py
        COMMENT "Expanding ${description} for the offline checks")
      list(APPEND EXPANDED_URDFS ${robot_urdf})
    else()
      set(robot_urdf ${CMAKE_CURRENT_SOURCE_DIR}/urdf/${description})
    endif()
    set(config ${CMAKE_CURRENT_SOURCE_DIR}/config)

    add_test(NAME verify_urdf_champ_contract_${robot}
      COMMAND python3 ${TOOLS}/verify_urdf_champ_contract.py --robot ${robot})
    add_test(NAME verify_champ_robot_${robot}
      COMMAND verify_champ_robot ${robot_urdf}
        ${config}/${robot}_gait.yaml ${config}/${robot}_joints.yaml ${config}/${robot}_links.yaml)
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/mujoco/${robot}.xml)
      add_test(NAME validate_mujoco_against_urdf_${robot}
        COMMAND python3 ${TOOLS}/validate_mujoco_against_urdf.py --robot ${robot})
      add_test(NAME verify_mujoco_physics_${robot}
        COMMAND python3 ${TOOLS}/verify_mujoco_physics.py --robot ${robot})
      add_test(NAME verify_mujoco_walk_${robot}
        COMMAND python3 ${TOOLS}/verify_mujoco_walk.py --robot ${robot})
    endif()
    if(EXISTS ${config}/${robot}_lowcmd.yaml)
      add_test(NAME verify_unitree_lowcmd_${robot}
        COMMAND verify_unitree_lowcmd ${robot_urdf} ${config}/${robot}_joints.yaml)
    endif()
    if(EXISTS ${config}/${robot}_body_pose.yaml)
      add_test(NAME verify_body_pose_controller_${robot}
        COMMAND verify_body_pose_controller ${robot_urdf}
          ${config}/${robot}_gait.yaml ${config}/${robot}_joints.yaml ${config}/${robot}_links.yaml
          ${config}/${robot}_body_pose.yaml)
    endif()
    add_test(NAME measure_gait_continuity_${robot}
      COMMAND measure_gait_continuity ${robot_urdf}
        ${config}/${robot}_gait.yaml ${config}/${robot}_joints.yaml ${config}/${robot}_links.yaml)
  endforeach()
  if(EXPANDED_URDFS)
    add_custom_target(expand_robot_urdfs ALL DEPENDS ${EXPANDED_URDFS})
  endif()
endif()

install(PROGRAMS
  mujoco/mujoco_sim.py
  tools/generate_mjcf.py
  tools/verify_urdf_champ_contract.py
  tools/verify_mujoco_physics.py
  tools/validate_mujoco_against_urdf.py
  tools/verify_mujoco_walk.py
  tools/run_offline_checks.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()
