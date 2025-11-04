cmake_minimum_required(VERSION 3.8)
project(moveit_tutorial)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_planners_ompl REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)



add_executable(move_group_interface_tutorial src/move_group_interface_tutorial.cpp)
ament_target_dependencies(move_group_interface_tutorial
  rclcpp
  moveit_ros_planning_interface
  moveit_core
  moveit_ros_planning
  moveit_planners_ompl
  moveit_task_constructor_core
  moveit_visual_tools
  tf2_geometry_msgs
)

install(TARGETS   
  move_group_interface_tutorial
  DESTINATION lib/${PROJECT_NAME})

  install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
