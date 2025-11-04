#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/parameter_client.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <thread>
#include <sstream>

static void ensure_moveit_params_present(const rclcpp::Node::SharedPtr& node) {
  // If robot_description_semantic is missing, try to fetch from /move_group
  const std::string key_srdf = "robot_description_semantic";
  if (!node->has_parameter(key_srdf)) {
    RCLCPP_WARN(node->get_logger(), "MoveIt params not on this node; fetching from /move_group ...");
    auto client = std::make_shared<rclcpp::SyncParametersClient>(node, "/move_group");
    // Wait for parameter services
    while (!client->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_INFO(node->get_logger(), "Waiting for /move_group parameter service...");
    }

    std::vector<std::string> keys = {
      "robot_description",
      "robot_description_semantic",
      "robot_description_planning",
      "robot_description_kinematics",
      "planning_pipelines",
      "moveit_controller_manager",
      "moveit_manage_controllers",
      "joint_limits",
      "ompl"
    };

    // Get and set them locally
    auto params = client->get_parameters(keys);
    std::vector<rclcpp::Parameter> to_set;
    to_set.reserve(params.size());
    for (size_t i = 0; i < keys.size(); ++i) {
      // Declare first if not declared
      if (!node->has_parameter(keys[i])) {
        // Declare with empty string as default value
        node->declare_parameter(keys[i], std::string(""));
      }
      to_set.emplace_back(params[i]);
    }
    node->set_parameters(to_set);
    RCLCPP_INFO(node->get_logger(), "MoveIt parameters imported from /move_group.");
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("planner");

  // -------- Params (still usable) --------
  const std::string planning_group =
      node->declare_parameter<std::string>("planning_group", "panda_arm");
  const bool use_named_goal =
      node->declare_parameter<bool>("use_named_goal", true);
  const std::string named_goal =
      node->declare_parameter<std::string>("named_goal", "ready");
  const double planning_time     = node->declare_parameter<double>("planning_time", 10.0);
  const int    planning_attempts = node->declare_parameter<int>("planning_attempts", 5);
  const double vscale            = node->declare_parameter<double>("max_vel_scale", 0.2);
  const double ascale            = node->declare_parameter<double>("max_acc_scale", 0.2);
  const std::string pipeline_id  =
      node->declare_parameter<std::string>("planning_pipeline", "ompl");
  const std::string planner_id   =
      node->declare_parameter<std::string>("planner_id", "RRTConnectkConfigDefault");

  // If we weren’t launched with MoveIt params, copy them from /move_group
  ensure_moveit_params_present(node);

  // -------- MoveIt setup --------
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

  rclcpp::sleep_for(std::chrono::milliseconds(200));

  // Optional warm-up
  if (use_named_goal) {
    if (mgi.setNamedTarget(named_goal)) {
      moveit::planning_interface::MoveGroupInterface::Plan warmup;
      if (mgi.plan(warmup) == moveit::core::MoveItErrorCode::SUCCESS) {
        (void)mgi.execute(warmup);
      }
      mgi.setStartStateToCurrentState();
    }
  }

  auto plan_and_execute = [&](double x, double y, double z) -> bool {
    geometry_msgs::msg::PoseStamped target;
    target.header.frame_id = mgi.getPlanningFrame();
    target.pose.orientation.w = 1.0;
    target.pose.position.x = x;
    target.pose.position.y = y;
    target.pose.position.z = z;

    mgi.setStartStateToCurrentState();
    mgi.setPoseTarget(target);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    RCLCPP_INFO(node->get_logger(), "Planning to [%.3f, %.3f, %.3f] ...", x, y, z);
    auto ok = (mgi.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!ok) {
      RCLCPP_ERROR(node->get_logger(), "Planning failed.");
      return false;
    }
    auto exec_result = mgi.execute(plan);
    bool exec_ok = (exec_result == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(node->get_logger(), exec_ok ? "Execution done." : "Execution failed.");
    return exec_ok;
  };

  // ---------- Interactive loop (stdin) ----------
  // NOTE: stdin works when running with `ros2 run`. When started via `ros2 launch`,
  // stdin is not connected to your process.
  while (rclcpp::ok()) {
    std::cout << "\nEnter x y z (meters), or 'q' to quit: " << std::flush;
    std::string line;
    if (!std::getline(std::cin, line)) break;      // EOF/ctrl-d ends
    if (line == "q" || line == "Q") break;

    std::istringstream iss(line);
    double x, y, z;
    if (!(iss >> x >> y >> z)) {
      std::cout << "Could not parse three numbers. Try again.\n";
      continue;
    }
    plan_and_execute(x, y, z);
  }

  rclcpp::shutdown();
  return 0;
}
