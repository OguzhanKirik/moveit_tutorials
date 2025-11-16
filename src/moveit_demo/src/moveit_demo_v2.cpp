#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <chrono>
#include <thread>

int main(int argc, char* argv[])
{
  // Step 1: Initialize ROS 2
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("panda_movegroup_demo", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  static const std::string PLANNING_GROUP = "panda_arm";

  // Step 2: Create MoveGroupInterface (loads robot model, connects to move_group, etc.)
  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

  // Optional: choose planner (must match ompl_planning.yaml configs)
  // move_group.setPlannerId("RRTConnectkConfigDefault");  // example

  // Step 3: Create PlanningSceneInterface for collision objects
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Step 4: Add table as a collision object
  moveit_msgs::msg::CollisionObject table;
  table.header.frame_id = "panda_link0";
  table.id = "table";
  table.operation = table.ADD;

  table.primitives.resize(1);
  table.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  table.primitives[0].dimensions = {0.2, 0.2, 0.2};  // x, y, z

  table.primitive_poses.resize(1);
  table.primitive_poses[0].position.x = 0.9;
  table.primitive_poses[0].position.y = 0.4;
  table.primitive_poses[0].position.z = 0.2;
  table.primitive_poses[0].orientation.w = 1.0;

  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.push_back(table);

  RCLCPP_INFO(node->get_logger(), "Adding collision object (table) to planning scene...");
  planning_scene_interface.addCollisionObjects(collision_objects);

  // Give RViz / planning scene some time to receive objects
  std::this_thread::sleep_for(std::chrono::seconds(1));

  // Step 5: Define the target pose for the end-effector
  geometry_msgs::msg::Pose target_pose;
  target_pose.orientation.w = 1.0;
  target_pose.position.x = 0.5;
  target_pose.position.y = 0.2;
  target_pose.position.z = 0.6;  // above the table

  move_group.setPoseTarget(target_pose);

  // Optional: adjust planning time / attempts
  move_group.setPlanningTime(10.0);
  move_group.setNumPlanningAttempts(3);

  // Step 6: Plan with MoveGroupInterface
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  RCLCPP_INFO(node->get_logger(), "Planning with MoveGroupInterface...");
  auto plan_result = move_group.plan(plan);

  if (plan_result != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Planning failed with error code: %d", plan_result.val);
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Planning succeeded. Executing...");

  // Step 7: Execute the plan (MoveGroupInterface talks to controller for you)
  auto exec_result = move_group.execute(plan);
  if (exec_result == moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_INFO(node->get_logger(), "Execution completed successfully!");
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Execution failed with error code: %d", exec_result.val);
  }

  // Step 8: Shutdown
  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  RCLCPP_INFO(node->get_logger(), "Demo finished.");
  return 0;
}
