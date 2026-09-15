cmake_minimum_required(VERSION 3.8)
project(unitree_rl_deploy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
add_compile_options(-std=c++17)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(Threads REQUIRED)
find_package(unitree_go QUIET)
find_package(unitree_api QUIET)

ament_python_install_package(${PROJECT_NAME})

set(UNITREE_ROS2_BRIDGE_BUILT FALSE)
if(unitree_go_FOUND AND unitree_api_FOUND)
  add_executable(unitree_ros2_bridge
    src/unitree_ros2_bridge.cpp
    src/motor_crc.cpp
  )
  target_include_directories(unitree_ros2_bridge PRIVATE include)
  ament_target_dependencies(unitree_ros2_bridge
    rclcpp sensor_msgs urdf unitree_go unitree_api)
  target_link_libraries(unitree_ros2_bridge Threads::Threads)
  set(UNITREE_ROS2_BRIDGE_BUILT TRUE)
  message(STATUS "unitree_go + unitree_api found: building unitree_ros2_bridge")
else()
  message(WARNING
    "unitree_go/unitree_api not found; skip unitree_ros2_bridge. Source "
    "unitree_ros2/cyclonedds_ws to run hardware "
    "(ros2 launch unitree_rl_deploy unitree_rl_sim2real.launch.py robot:=<robot>).")
endif()

set(EXEC_TARGETS)
if(UNITREE_ROS2_BRIDGE_BUILT)
  list(APPEND EXEC_TARGETS unitree_ros2_bridge)
endif()
if(EXEC_TARGETS)
  install(TARGETS ${EXEC_TARGETS} DESTINATION lib/${PROJECT_NAME})
endif()

install(PROGRAMS
  mujoco/mujoco_sim.py
  scripts/policy_runner
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  meshes
  mujoco
  policies
  urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest QUIET)
  if(ament_cmake_pytest_FOUND)
    ament_add_pytest_test(unitree_rl_deploy_pytest test)
  endif()
endif()

ament_package()
