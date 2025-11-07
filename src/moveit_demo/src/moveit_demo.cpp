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
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <chrono>
#include <thread>

// --- [NEW] includes for constraints & collision objects/checks
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/robot_trajectory/robot_trajectory.h>


#include <moveit_visual_tools/moveit_visual_tools.h>


// Fix the visulaizaton problems, there is too many rvuz visual command
// implement a diffenet IK method, or control method
// implmenet a different planner
// implemetn all these with move group interface

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
    
    // Set initial joint values to avoid collision with table
    std::vector<double> initial_joint_values = {0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785};
    robot_state->setJointGroupPositions(PLANNING_GROUP, initial_joint_values);

    planning_scene::PlanningScenePtr planning_scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
    planning_scene->setCurrentState(*robot_state);



    // Setup MoveIt Visual Tools for step-by-step visualization
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools(motion_demo, "panda_link0", "moveit_demo_tutorial", robot_model);
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();
    
    // Text pose for status messages
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.5;
    
    // Step 1: Show initial setup
    visual_tools.publishText(text_pose, "MoveIt Demo: Step-by-Step Planning", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' to see initial robot state");




    // Step 2: Show initial robot state
    visual_tools.publishText(text_pose, "Step 1: Initial Robot State", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishRobotState(robot_state, rvt::GREEN);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' to add collision objects");

    // --- [NEW] 2a) Add collision objects to the local planning scene (e.g., a table)
    {
      moveit_msgs::msg::CollisionObject table;
      table.header.frame_id = "panda_link0";
      table.id = "table";
      table.operation = table.ADD;

      table.primitives.resize(1);
      table.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
      table.primitives[0].dimensions = {0.2, 0.2, 0.2}; // x,y,z (m) - made table shorter

      table.primitive_poses.resize(1);
      table.primitive_poses[0].position.x = 0.9;  // moved table further away
      table.primitive_poses[0].position.y = 0.4;
      table.primitive_poses[0].position.z = 0.2;  // lowered table
      table.primitive_poses[0].orientation.w = 1.0;

      planning_scene->processCollisionObjectMsg(table);
      RCLCPP_INFO(motion_demo->get_logger(), "Added collision object '%s'", table.id.c_str());
      
      // Step 3: Show collision object
      visual_tools.publishText(text_pose, "Step 2: Added Collision Object (Table)", rvt::WHITE, rvt::XLARGE);
      // Visualize collision object using its pose and dimensions
      geometry_msgs::msg::Pose table_pose = table.primitive_poses[0];
      visual_tools.publishCollisionBlock(table_pose, "table", 
                                         table.primitives[0].dimensions[0], rvt::RED);
      visual_tools.trigger();
      visual_tools.prompt("Press 'next' to show goal pose");
    }




    // Create GOAL + Plan
    // 1. Goal Pose for the EE Link
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = "panda_link0";
    goal.pose.orientation.w = 1.0;
    goal.pose.position.x = 0.5;  // Adjusted to be more reachable
    goal.pose.position.y = 0.2;
    goal.pose.position.z = 0.6;  // Higher to avoid table collision
    const std::string ee_link = "panda_link8";

    auto goal_constraints = kinematic_constraints::constructGoalConstraints(ee_link, goal, 0.01, 0.01);
    
    // Step 4: Show goal pose
    visual_tools.publishText(text_pose, "Step 3: Goal Pose", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishAxisLabeled(goal.pose, "goal_pose", rvt::LARGE);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' to start motion planning");
    

    // Remove the joint constraint as it's causing issues
    // --- [NEW] 1a) (Optional) Add a joint constraint to the goal
    // {
    //   moveit_msgs::msg::JointConstraint jc;
    //   jc.joint_name = "panda_joint4";
    //   jc.position = 0.0;
    //   jc.tolerance_above = 1.57; // +/- 90°
    //   jc.tolerance_below = 1.57;
    //   jc.weight = 0.5;
    //   goal_constraints.joint_constraints.push_back(jc);
    // }
    
    // 2. MotionPlanRequest
    moveit_msgs::msg::MotionPlanRequest req;
    req.group_name = PLANNING_GROUP;
    req.allowed_planning_time = 10.0;  // Increased planning time
    req.num_planning_attempts = 3;     // More attempts
    req.goal_constraints = {goal_constraints};


    // Remove path constraints to simplify the planning problem
    // --- [NEW] 1b) (Optional) Add a path constraint (keep tool upright)
    // {
    //   moveit_msgs::msg::OrientationConstraint ocon;
    //   ocon.link_name = ee_link;
    //   ocon.header.frame_id = "panda_link0";
    //   ocon.orientation.w = 1.0;                 // world Z up
    //   ocon.absolute_x_axis_tolerance = 0.05;     // rad
    //   ocon.absolute_y_axis_tolerance = 0.05;
    //   ocon.absolute_z_axis_tolerance = 3.14;     // allow yaw
    //   ocon.weight = 1.0;

    //   moveit_msgs::msg::Constraints path_cons;
    //   path_cons.orientation_constraints.push_back(ocon);
    //   req.path_constraints = path_cons;
    // }

    moveit::core::RobotState start_state = planning_scene->getCurrentState();
    moveit::core::robotStateToRobotStateMsg(start_state, req.start_state);



    // --- [NEW] (Optional) Explicitly verify start_state collision
    {
      collision_detection::CollisionRequest creq;
      collision_detection::CollisionResult  cres;
      planning_scene->checkCollision(creq, cres, start_state);
      if (cres.collision) {
        RCLCPP_WARN(motion_demo->get_logger(), "Start state is in collision.");
      }
    }


    // 3. Plan via PlanningPipeline(OMPL)
    visual_tools.publishText(text_pose, "Step 4: Motion Planning in Progress...", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();
    
    planning_pipeline::PlanningPipeline pipeline(robot_model, motion_demo, "ompl");
    planning_interface::MotionPlanResponse res;

    if (!pipeline.generatePlan(planning_scene, req, res) ||
        res.error_code_.val != res.error_code_.SUCCESS) {
        visual_tools.publishText(text_pose, "Planning FAILED!", rvt::RED, rvt::XLARGE);
        visual_tools.trigger();
        RCLCPP_ERROR(motion_demo->get_logger(), "Planning failed: %d", res.error_code_.val);
        executor.cancel(); spinner.join(); rclcpp::shutdown(); return 1;
    }
    
    // Step 5: Show successful planning
    visual_tools.publishText(text_pose, "Step 5: Planning Successful!", rvt::GREEN, rvt::XLARGE);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' to see the planned trajectory");

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

    // Step 6: Visualize the planned trajectory
    visual_tools.publishText(text_pose, "Step 6: Planned Trajectory", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(res.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' to see trajectory waypoints step-by-step");

    // Step 7: Animate trajectory execution step by step
    visual_tools.publishText(text_pose, "Step 7: Trajectory Animation", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();
    
    // Create a new robot trajectory for waypoint animation
    robot_trajectory::RobotTrajectory anim_traj(robot_model, PLANNING_GROUP);
    anim_traj.setRobotTrajectoryMsg(start_state, trajectory_msg);
    
    const size_t num_waypoints = anim_traj.getWayPointCount();
    RCLCPP_INFO(motion_demo->get_logger(), "Trajectory has %zu waypoints", num_waypoints);
    
    // Show each waypoint
    moveit::core::RobotState waypoint_state(robot_model);
    for (size_t i = 0; i < num_waypoints; ++i) {
        waypoint_state = anim_traj.getWayPoint(i);
        
        // Clear previous robot state and show current waypoint
        visual_tools.publishRobotState(waypoint_state, rvt::ORANGE);
        visual_tools.publishText(text_pose, "Waypoint " + std::to_string(i + 1) + "/" + 
                                 std::to_string(num_waypoints), rvt::WHITE, rvt::XLARGE);
        visual_tools.trigger();
        
        // Brief pause to see the motion
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    
    // Final state
    visual_tools.publishText(text_pose, "Step 8: Motion Complete!", rvt::GREEN, rvt::XLARGE);
    visual_tools.trigger();
    visual_tools.prompt("Demo finished! Press 'next' to exit");

    RCLCPP_INFO(motion_demo->get_logger(), "Demo completed successfully!");




    // // --- [NEW] 3b) Validate each waypoint of the planned trajectory
    // {
    //   bool traj_ok = true;
    //   moveit::core::RobotState waypoint_state(robot_model);
    //   collision_detection::CollisionRequest creq;
    //   for (const auto& pt : traj_msg.joint_trajectory.points) {
    //     waypoint_state = start_state;
    //     waypoint_state.setVariablePositions(traj_msg.joint_trajectory.joint_names, pt.positions);
    //     waypoint_state.update();

    //     collision_detection::CollisionResult cres;
    //     planning_scene->checkCollision(creq, cres, waypoint_state);
    //     if (cres.collision) {
    //       traj_ok = false;
    //       RCLCPP_ERROR(motion_demo->get_logger(), "Planned trajectory has a colliding waypoint.");
    //       break;
    //     }
    //   }
    //   if (!traj_ok) {
    //     executor.cancel(); spinner.join(); rclcpp::shutdown(); return 1;
    //   }
    // }

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