#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
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

#include <chrono>
#include <thread>

// Constraints & collision
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

// Fix the visualization problems, there is too many rviz visual commands
// implement a different IK method, or control method
// implement a different planner
// implement all these with move group interface

int main(int argc, char* argv[])
{
    // Step 1: Initialize ROS 2 node and executor
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    std::shared_ptr<rclcpp::Node> motion_demo =
        rclcpp::Node::make_shared("motion_demo", node_options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(motion_demo);
    std::thread spinner([&executor]() { executor.spin(); });

    // Step 2: Load robot model (URDF from 'robot_description')
    robot_model_loader::RobotModelLoader robot_model_loader(motion_demo, "robot_description");
    const moveit::core::RobotModelPtr& robot_model = robot_model_loader.getModel();
    if (!robot_model)
    {
        RCLCPP_FATAL(motion_demo->get_logger(), "Failed to load robot model (robot_description).");
        executor.cancel();
        spinner.join();
        rclcpp::shutdown();
        return 1;
    }
    RCLCPP_INFO(motion_demo->get_logger(), "Loaded RobotModel: %s", robot_model->getName().c_str());

    // Step 3: Create RobotState and set to default configuration
    moveit::core::RobotStatePtr robot_state =
        std::make_shared<moveit::core::RobotState>(robot_model);
    robot_state->setToDefaultValues();  // SRDF defaults if available

    const std::string PLANNING_GROUP = "panda_arm";
    const moveit::core::JointModelGroup* joint_model_group =
        robot_state->getJointModelGroup(PLANNING_GROUP);

    if (!joint_model_group)
    {
    RCLCPP_FATAL(motion_demo->get_logger(), "JointModelGroup '%s' not found.",
                 PLANNING_GROUP.c_str());
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
    }

    // Step 4: Set an initial joint configuration (collision-free startup pose)
    std::vector<double> initial_joint_values = {0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785};
    robot_state->setJointGroupPositions(PLANNING_GROUP, initial_joint_values);

    // Step 5: Create planning scene and set the current state
    planning_scene::PlanningScenePtr planning_scene =
        std::make_shared<planning_scene::PlanningScene>(robot_model);
    planning_scene->setCurrentState(*robot_state);

    // Step 6: Add a collision object (table) to the planning scene
    {
        moveit_msgs::msg::CollisionObject table;
        table.header.frame_id = "panda_link0";
        table.id = "table";
        table.operation = table.ADD;

        table.primitives.resize(1);
        table.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
        table.primitives[0].dimensions = {0.2, 0.2, 0.2};  // x, y, z (m)

        table.primitive_poses.resize(1);
        table.primitive_poses[0].position.x = 0.9;  // moved further away
        table.primitive_poses[0].position.y = 0.4;
        table.primitive_poses[0].position.z = 0.2;  // lower
        table.primitive_poses[0].orientation.w = 1.0;

        planning_scene->processCollisionObjectMsg(table);
        RCLCPP_INFO(motion_demo->get_logger(), "Added collision object '%s'", table.id.c_str());
        
        // Publish collision object to RViz for visualization
        auto collision_pub = motion_demo->create_publisher<moveit_msgs::msg::CollisionObject>("/collision_object", 10);
        // Give time for publisher to be ready
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        collision_pub->publish(table);
        RCLCPP_INFO(motion_demo->get_logger(), "Published collision object to RViz");
        
        // Also publish to planning scene for RViz
        auto scene_pub = motion_demo->create_publisher<moveit_msgs::msg::PlanningScene>("/planning_scene", 10);
        moveit_msgs::msg::PlanningScene scene_msg;
        scene_msg.world.collision_objects.push_back(table);
        scene_msg.is_diff = true;  // This is a diff, not the full scene
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        scene_pub->publish(scene_msg);
        RCLCPP_INFO(motion_demo->get_logger(), "Published planning scene to RViz");
    }

    // Step 7: Define end-effector goal pose (in base frame)
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = "panda_link0";
    goal.pose.orientation.w = 1.0;
    goal.pose.position.x = 0.5;   // reachable x
    goal.pose.position.y = 0.2;
    goal.pose.position.z = 0.6;   // high enough to avoid table
    const std::string ee_link = "panda_link8";

    auto goal_constraints =
        kinematic_constraints::constructGoalConstraints(ee_link, goal, 0.01, 0.01); // set goal threshold in joint space

    // Step 8: Fill MotionPlanRequest with group, time and constraints
    moveit_msgs::msg::MotionPlanRequest req;
    req.group_name = PLANNING_GROUP;
    req.allowed_planning_time = 10.0;  // Increased planning time
    req.num_planning_attempts = 3;     // More attempts
    req.goal_constraints = {goal_constraints};
    // Set planner here
    req.planner_id = "RRTConnect";   // BiRRT // PRM // RRTstar // 

  // (Optional) Joint / path constraints are removed to simplify the problem

    // Step 9: Set start state of the motion plan from current planning scene state
    moveit::core::RobotState start_state = planning_scene->getCurrentState();
    moveit::core::robotStateToRobotStateMsg(start_state, req.start_state);

    // Step 10: Check if start state is collision-free
    {
        collision_detection::CollisionRequest creq;
        collision_detection::CollisionResult cres;
        planning_scene->checkCollision(creq, cres, start_state);
        if (cres.collision)
        {
        RCLCPP_WARN(motion_demo->get_logger(), "Start state is in collision.");
        }
        else
        {
        RCLCPP_INFO(motion_demo->get_logger(), "Start state is collision-free.");
        }
    }

    // Step 11: Plan motion with PlanningPipeline (OMPL)
    planning_pipeline::PlanningPipeline pipeline(robot_model, motion_demo, "ompl");
    planning_interface::MotionPlanResponse res;

    RCLCPP_INFO(motion_demo->get_logger(), "Planning motion...");
    if (!pipeline.generatePlan(planning_scene, req, res) ||
        res.error_code_.val != res.error_code_.SUCCESS)
    {
        RCLCPP_ERROR(motion_demo->get_logger(), "Planning failed: %d", res.error_code_.val);
        executor.cancel();
        spinner.join();
        rclcpp::shutdown();
        return 1;
    }
    RCLCPP_INFO(motion_demo->get_logger(), "Planning succeeded.");

    // Step 12: Copy trajectory from response into a RobotTrajectory object
    robot_trajectory::RobotTrajectory robot_traj(robot_model, PLANNING_GROUP); // moveit type trajectory
    moveit_msgs::msg::RobotTrajectory trajectory_msg;
    res.trajectory_->getRobotTrajectoryMsg(trajectory_msg); //ompl trajectory converted into moveit trajectory(for timing ,accel, velocity)
    robot_traj.setRobotTrajectoryMsg(start_state, trajectory_msg); // moveit has trajectroy created by ompl

    // Step 13: Time-parameterize trajectory (assign velocities and time stamps)
    trajectory_processing::IterativeParabolicTimeParameterization iptp; // iptp algo
    iptp.computeTimeStamps(robot_traj, 0.5, 0.5);  // max vel / acc scaling 50%

    moveit_msgs::msg::RobotTrajectory traj_msg;
    robot_traj.getRobotTrajectoryMsg(traj_msg);

    // Step 14: Log number of waypoints and (optionally) inspect them
    // robot_trajectory::RobotTrajectory anim_traj(robot_model, PLANNING_GROUP);
    // anim_traj.setRobotTrajectoryMsg(start_state, trajectory_msg);

    // const size_t num_waypoints = anim_traj.getWayPointCount();
    // RCLCPP_INFO(motion_demo->get_logger(), "Trajectory has %zu waypoints", num_waypoints);

    /*
    MoveGroupInterface is a high-level wrapper around:
    planning scene
    planning pipeline
    controllers
*/
    // Step 15: Publish trajectory to RViz (DisplayTrajectory) for visualization
    auto disp_pub =
        motion_demo->create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path", 10);
    moveit_msgs::msg::DisplayTrajectory disp;
    disp.model_id = robot_model->getName();
    moveit::core::robotStateToRobotStateMsg(start_state, disp.trajectory_start);
    disp.trajectory.push_back(traj_msg);
    disp_pub->publish(disp);
    RCLCPP_INFO(motion_demo->get_logger(), "Published /display_planned_path");
    




    // Step 16: Execute the manually planned trajectory via ros2_control
    auto ctrl_pub = motion_demo->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "panda_arm_controller/joint_trajectory", 10);

    // Give the publisher some time to connect to the controller
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    RCLCPP_INFO(motion_demo->get_logger(), "Sending manually planned trajectory to controller...");
    ctrl_pub->publish(traj_msg.joint_trajectory);
    RCLCPP_INFO(motion_demo->get_logger(), "Trajectory sent. Controller will execute it if configured.");














    // RCLCPP_INFO(motion_demo->get_logger(), "TRAJECTORY VISUALIZATION COMPLETE");
    // RCLCPP_INFO(motion_demo->get_logger(), "Note: Robot shows planned path but does NOT execute it.");
    // RCLCPP_INFO(motion_demo->get_logger(), "To execute: uncomment MoveGroupInterface section in code.");
    // RCLCPP_INFO(motion_demo->get_logger(), "Or run with: ros2 launch panda_moveit_config demo.launch.py");

    // // Step 16: Execute the trajectory using MoveGroupInterface
    // RCLCPP_INFO(motion_demo->get_logger(), "Executing trajectory...");
    // moveit::planning_interface::MoveGroupInterface move_group(motion_demo, PLANNING_GROUP);
    // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    
    // // Add collision object to planning scene interface
    // std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    // moveit_msgs::msg::CollisionObject table;
    // table.header.frame_id = "panda_link0";
    // table.id = "table";
    // table.operation = table.ADD;
    // table.primitives.resize(1);
    // table.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    // table.primitives[0].dimensions = {0.2, 0.2, 0.2};
    // table.primitive_poses.resize(1);
    // table.primitive_poses[0].position.x = 0.9;
    // table.primitive_poses[0].position.y = 0.4;
    // table.primitive_poses[0].position.z = 0.2;
    // table.primitive_poses[0].orientation.w = 1.0;
    // collision_objects.push_back(table);
    // planning_scene_interface.addCollisionObjects(collision_objects);
    
    // // Set goal pose and execute
    // geometry_msgs::msg::Pose target_pose;
    // target_pose.orientation.w = 1.0;
    // target_pose.position.x = 0.5;
    // target_pose.position.y = 0.2;
    // target_pose.position.z = 0.6;
    // move_group.setPoseTarget(target_pose);
    
    // moveit::planning_interface::MoveGroupInterface::Plan plan;
    // bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    // if (success) {
    //     RCLCPP_INFO(motion_demo->get_logger(), "Planning successful! Executing...");
    //     auto execute_result = move_group.execute(plan);
    //     if (execute_result == moveit::core::MoveItErrorCode::SUCCESS) {
    //         RCLCPP_INFO(motion_demo->get_logger(), "Execution completed successfully!");
    //     } else {
    //         RCLCPP_ERROR(motion_demo->get_logger(), "Execution failed!");
    //     }
    // } else {
    //     RCLCPP_ERROR(motion_demo->get_logger(), "Planning failed!");
    // }

    // Step 17: Clean shutdown
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    RCLCPP_INFO(motion_demo->get_logger(), "Demo completed successfully.");
    return 0;

}
