#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <thread>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("minimal_planner");

  // Spin in the background so MoveIt/ROS callbacks get serviced.
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  std::thread spinner([&exec]() { exec.spin(); });

  // ------------ Parameters ------------
  const std::string planning_group =
      node->declare_parameter<std::string>("planning_group", "panda_arm");

  const bool use_named_goal =
      node->declare_parameter<bool>("use_named_goal", true);

  const std::string named_goal =
      node->declare_parameter<std::string>("named_goal", "ready"); // UR often "home"

  const double target_x = node->declare_parameter<double>("target_x", 0.30);
  const double target_y = node->declare_parameter<double>("target_y", 0.40);
  const double target_z = node->declare_parameter<double>("target_z", 0.30);

  const double planning_time = node->declare_parameter<double>("planning_time", 10.0);
  const int planning_attempts = node->declare_parameter<int>("planning_attempts", 5);
  const double vscale = node->declare_parameter<double>("max_vel_scale", 0.2);
  const double ascale = node->declare_parameter<double>("max_acc_scale", 0.2);

  const std::string pipeline_id =
      node->declare_parameter<std::string>("planning_pipeline", "ompl");
  const std::string planner_id =
      node->declare_parameter<std::string>("planner_id", "RRTConnectkConfigDefault");

  // ------------ MoveIt setup ------------
  moveit::planning_interface::MoveGroupInterface mgi(node, planning_group);
  moveit::planning_interface::PlanningSceneInterface psi;

  mgi.setPlanningPipelineId(pipeline_id);
  mgi.setPlannerId(planner_id);
  mgi.setPlanningTime(planning_time);
  mgi.setNumPlanningAttempts(planning_attempts);
  mgi.setMaxVelocityScalingFactor(vscale);
  mgi.setMaxAccelerationScalingFactor(ascale);
  mgi.setStartStateToCurrentState();

  RCLCPP_INFO(node->get_logger(), "Planning group: %s", planning_group.c_str());
  RCLCPP_INFO(node->get_logger(), "Planning frame: %s", mgi.getPlanningFrame().c_str());
  RCLCPP_INFO(node->get_logger(), "End effector link: %s", mgi.getEndEffectorLink().c_str());

  // Small settle to ensure parameters/scene are ready (esp. when launched together)
  rclcpp::sleep_for(std::chrono::milliseconds(200));

  // ------------ Optional warm-up to a named state ------------
  if (use_named_goal)
  {
    RCLCPP_INFO(node->get_logger(), "Trying named target: %s", named_goal.c_str());
    if (mgi.setNamedTarget(named_goal))
    {
      moveit::planning_interface::MoveGroupInterface::Plan warmup;
      if (mgi.plan(warmup) == moveit::core::MoveItErrorCode::SUCCESS)
      {
        auto res = mgi.execute(warmup);
        if (res == moveit::core::MoveItErrorCode::SUCCESS)
          RCLCPP_INFO(node->get_logger(), "Reached named target: %s", named_goal.c_str());
        else
          RCLCPP_WARN(node->get_logger(), "Execution to named target failed.");
      }
      else
      {
        RCLCPP_WARN(node->get_logger(), "Planning to named target failed; continuing.");
      }
      mgi.setStartStateToCurrentState();
    }
    else
    {
      RCLCPP_WARN(node->get_logger(), "Named target '%s' not defined; continuing.", named_goal.c_str());
    }
  }

  // ------------ Pose goal ------------
  geometry_msgs::msg::PoseStamped target;
  target.header.frame_id = mgi.getPlanningFrame(); // typically base frame
  target.pose.orientation.w = 1.0;
  target.pose.position.x = target_x;
  target.pose.position.y = target_y;
  target.pose.position.z = target_z;

  mgi.setPoseTarget(target);

  // Plan with a couple of retries using the same seed (simple robustness)
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const int max_retries = 2;
  bool ok = false;
  for (int i = 0; i <= max_retries; ++i)
  {
    RCLCPP_INFO(node->get_logger(), "Planning attempt %d/%d ...", i+1, max_retries+1);
    if (mgi.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
    {
      ok = true;
      break;
    }
  }

  if (ok)
  {
    RCLCPP_INFO(node->get_logger(), "Planning succeeded. Executing trajectory with %.0f%% vel, %.0f%% acc.",
                vscale*100.0, ascale*100.0);
    auto exec_result = mgi.execute(plan);
    if (exec_result == moveit::core::MoveItErrorCode::SUCCESS)
      RCLCPP_INFO(node->get_logger(), "Execution done.");
    else
      RCLCPP_ERROR(node->get_logger(), "Execution failed.");
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Planning failed for pose [%.3f, %.3f, %.3f] in frame '%s'.",
                 target_x, target_y, target_z, target.header.frame_id.c_str());
  }

  // Clean shutdown
  rclcpp::shutdown();
  if (spinner.joinable()) spinner.join();
  return ok ? 0 : 1;
}
