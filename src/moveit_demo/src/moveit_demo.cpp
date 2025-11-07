#include <rclcpp/rclcpp.hpp>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_scene/planning_scene.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>


int main(int argc, char*argv[]){
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    std::shared_ptr<rclcpp::Node> motion_demo =
        rclcpp::Node::make_shared("motion_demo", node_options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(motion_demo);
    std::thread spinner([&executor]() {executor.spin();});


    //LOAD MODEL
    robot_model_loader::RobotModelLoader robot_model_loader(motion_demo,"robot_description");
    const moveit::core::RobotModelPtr& robot_model = robot_model_loader.getModel();
    if(!robot_model){
        RCLCPP_FATAL(motion_demo->get_logger(), "Failed to load robot model (robot_description).");
        executor.cancel(); spinner.join(); rclcpp::shutdown(); return 1;
    }
    RCLCPP_INFO(motion_demo->get_logger(), "Loaded RobotModel: %s", robot_model->getName().c_str());

    // Robot State and Planning Scene
    moveit::core::RobotStatePtr robot_state = std::make_shared<moveit::core::RobotState>(robot_model);
    robot_state->setToDefaultValues();  // SRDF defaults if available

    const std::string PLANNING_GROUP = "panda_arm";
    const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP); 
    
    if (!joint_model_group) {
        RCLCPP_FATAL(motion_demo->get_logger(), "JointModelGroup '%s' not found.", PLANNING_GROUP.c_str());
        executor.cancel(); spinner.join(); rclcpp::shutdown(); return 1;
    }

    planning_scene::PlanningScenePtr planning_scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
    planning_scene->setCurrentState(*robot_state);

    // Create GOAL + PLan
    // 1. Goal Pose for the EE Link
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = "panda_link0";
    goal.pose.orientation.w = 1.0;
    goal.pose.position.x = 0.4;
    goal.pose.position.y = 0.0;
    goal.pose.position.z = 0.4;
    const std::string ee_link = "panda_link8";

    auto goal_constraints = kinematic_constraints::constructGoalConstraints(ee_link, goal,1e-3, 1e-3);
    
    
    // 2. MotionPlanRequest
    moveit_msgs::msg::MotionPlanRequest req;
    req.group_name = PLANNING_GROUP;
    req.allowed_planning_time = 5.0;
    req.num_planning_attempts = 1;
    req.goal_constraints = {goal_constraints};

    moveit::core::RobotState start_state = planning_scene->getCurrentState();
    moveit::core::robotStateToRobotStateMsg(start_state, req.start_state);

    // 3. Plan via PlanningPipeline(OMPL)
    planning_pipeline::PlanningPipeline pipeline(robot_model, motion_demo, "planning_pipeline");
    planning_interface::MotionPlanResponse res;

    if (!pipeline.generatePlan(planning_scene, req, res) ||
        res.error_code_.val != res.error_code_.SUCCESS) {
        RCLCPP_ERROR(motion_demo->get_logger(), "Planning failed: %d", res.error_code_.val);
        executor.cancel(); spinner.join(); rclcpp::shutdown(); return 1;
    }

    // Time parameterize
    robot_trajectory::RobotTrajectory robot_traj(robot_model, PLANNING_GROUP);
    // Get the trajectory message from the response
    moveit_msgs::msg::RobotTrajectory trajectory_msg;
    res.trajectory_->getRobotTrajectoryMsg(trajectory_msg);
    robot_traj.setRobotTrajectoryMsg(start_state, trajectory_msg);
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    iptp.computeTimeStamps(robot_traj, 0.5, 0.5);

    moveit_msgs::msg::RobotTrajectory traj_msg;
    robot_traj.getRobotTrajectoryMsg(traj_msg);


    // 5) Visualize in RViz (Planned Path display)
    auto disp_pub = motion_demo->create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path", 10);
    moveit_msgs::msg::DisplayTrajectory disp;
    disp.model_id = robot_model->getName();
    moveit::core::robotStateToRobotStateMsg(start_state, disp.trajectory_start);
    disp.trajectory.push_back(traj_msg);
    disp_pub->publish(disp);
    RCLCPP_INFO(motion_demo->get_logger(), "Published /display_planned_path");

    // 6) (Optional) Execute via ros2_control
    auto ctrl_pub = motion_demo->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/joint_trajectory_controller/joint_trajectory", 10);
    ctrl_pub->publish(traj_msg.joint_trajectory);
    RCLCPP_INFO(motion_demo->get_logger(), "Sent trajectory to controller.");

    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 0;
}