cmake_minimum_required(VERSION 3.8)
project(moveit_minimal_planner)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_planners_ompl REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(minimal_planner src/minimal_planner.cpp)
ament_target_dependencies(minimal_planner
  rclcpp
  moveit_ros_planning_interface
)

add_executable(planner src/planner.cpp)
ament_target_dependencies(planner
  rclcpp
  moveit_ros_planning_interface
)

add_executable(mtc_node src/mtc_node.cpp)
ament_target_dependencies(mtc_node
  rclcpp
  moveit_ros_planning_interface
  moveit_core
  moveit_ros_planning
  moveit_planners_ompl
  moveit_task_constructor_core
  tf2_geometry_msgs
)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

install(TARGETS   
  minimal_planner
  planner
  mtc_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
