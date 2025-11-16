cmake_minimum_required(VERSION 3.8)
project(moveit_demo)

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
find_package(pluginlib REQUIRED)
find_package(Eigen3 REQUIRED)


include_directories(
  include
)

add_library(custom_ik_solver
  src/custom_ik_solver.cpp
)

ament_target_dependencies(custom_ik_solver
  rclcpp
  moveit_core
  geometry_msgs
  Eigen3
)

add_executable(moveit_demo src/moveit_demo.cpp)
ament_target_dependencies(moveit_demo
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
target_link_libraries(moveit_demo
  custom_ik_solver
)


add_executable(moveit_demo_v2 src/moveit_demo_v2.cpp)
ament_target_dependencies(moveit_demo_v2
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
  moveit_demo
  moveit_demo_v2
  custom_ik_solver
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
