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
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(rviz_visual_tools REQUIRED)



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

add_executable(robot_model_and_robot_state_tutorial src/robot_model_and_robot_state_tutorial.cpp)
ament_target_dependencies(robot_model_and_robot_state_tutorial
  rclcpp
  moveit_ros_planning_interface
  moveit_core
  moveit_ros_planning
  moveit_planners_ompl
  moveit_task_constructor_core
  moveit_visual_tools
  tf2_geometry_msgs
)

add_executable(planning_scene_tutorial src/planning_scene_tutorial.cpp)
ament_target_dependencies(planning_scene_tutorial
  rclcpp
  moveit_ros_planning_interface
  moveit_core
  moveit_ros_planning
  moveit_planners_ompl
  moveit_task_constructor_core
  moveit_visual_tools
  tf2_geometry_msgs
)
add_executable(planning_scene_ros_api_tutorial src/planning_scene_ros_api_tutorial.cpp)
ament_target_dependencies(planning_scene_ros_api_tutorial
  rclcpp
  moveit_ros_planning_interface
  moveit_core
  moveit_ros_planning
  moveit_planners_ompl
  moveit_task_constructor_core
  moveit_visual_tools
  tf2_geometry_msgs
  moveit_msgs
  geometry_msgs
  shape_msgs
  rviz_visual_tools
)


install(TARGETS   
  move_group_interface_tutorial
  robot_model_and_robot_state_tutorial
  planning_scene_tutorial
  planning_scene_ros_api_tutorial
  DESTINATION lib/${PROJECT_NAME})

  install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
