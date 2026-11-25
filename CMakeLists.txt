cmake_minimum_required(VERSION 3.8)
project(drone_swarm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

add_executable(waypoint_follower src/waypoint_follower.cpp)
ament_target_dependencies(waypoint_follower rclcpp px4_msgs)

add_executable(boids src/boids.cpp)
ament_target_dependencies(boids rclcpp px4_msgs std_msgs geometry_msgs)

add_executable(formation_control src/formation_control.cpp)
ament_target_dependencies(formation_control rclcpp px4_msgs std_msgs geometry_msgs)

add_executable(consensus src/consensus.cpp)
ament_target_dependencies(consensus rclcpp px4_msgs std_msgs geometry_msgs)

add_executable(waypoint_mission src/waypoint_mission_sender.cpp)
ament_target_dependencies(waypoint_mission rclcpp px4_msgs std_msgs geometry_msgs)

add_executable(potential_fields src/potential_fields.cpp)
ament_target_dependencies(potential_fields rclcpp px4_msgs std_msgs geometry_msgs)

add_executable(pso_controller src/pso_controller.cpp)
ament_target_dependencies(pso_controller rclcpp px4_msgs std_msgs geometry_msgs)

add_executable(aco_controller src/aco_controller.cpp)
ament_target_dependencies(aco_controller rclcpp px4_msgs std_msgs geometry_msgs)

add_executable(abc_controller src/abc_controller.cpp)
ament_target_dependencies(abc_controller rclcpp px4_msgs std_msgs geometry_msgs)

add_executable(swarm_logger src/swarm_logger.cpp)
ament_target_dependencies(swarm_logger rclcpp px4_msgs std_msgs geometry_msgs)

add_executable(mission_pub src/scaling_swarm/mission_pub.cpp)
ament_target_dependencies(mission_pub rclcpp std_msgs geometry_msgs)

add_executable(mission_logger src/scaling_swarm/mission_logger.cpp)
ament_target_dependencies(mission_logger rclcpp px4_msgs std_msgs geometry_msgs)

add_executable(mission_reset_bridge src/scaling_swarm/mission_reset_bridge.cpp)
ament_target_dependencies(mission_reset_bridge rclcpp std_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY include
    DESTINATION include
)

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
    waypoint_follower
    boids
    formation_control
    consensus
    waypoint_mission
    potential_fields
    pso_controller
    aco_controller
    abc_controller
    swarm_logger
    mission_pub
    mission_logger
    mission_reset_bridge
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
