#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

//moveit
#include <moveit_msgs/msg/planning_scene.h>
#include <moveit_msgs/msg/attached_collision.h>
#include <moveit_msgs/msg/get_state_validity.h>
#include <moveit_msgs/msg/display_robot_state.h>
#include <moveit_msgs/msg/apply_planning_scene.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/robot_state/conversions.hpp>

#include <rviz_visual_tools/rviz_visual_tools.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("planning_scene_ros_api_tutorial");

int main(int argv, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("planning_scene_ros_api_tutorial", node_options);

    rclcpp::executors::SingleThreadedExecutor exeecutor;
    executor.add_node(node);
    std::thread([&executor]() {executor.spin();}).detach();

    //Visualization
    rviz_tutorial_tools::

}