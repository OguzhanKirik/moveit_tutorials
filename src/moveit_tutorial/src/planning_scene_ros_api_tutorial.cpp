#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

//moveit
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/srv/get_state_validity.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <rviz_visual_tools/rviz_visual_tools.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("planning_scene_ros_api_tutorial");

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("planning_scene_ros_api_tutorial", node_options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() {executor.spin();}).detach();

    //Visualization
    rviz_visual_tools::RvizVisualTools visual_tools("panda_link0", "planning_scene_ros_api_tutorial", node);
    visual_tools.loadRemoteControl();
    visual_tools.deleteAllMarkers();

    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_diff_publisher =
        node->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene",1);

    while(planning_scene_diff_publisher->get_subscription_count() < 1){
        rclcpp::sleep_for(std::chrono::milliseconds(500));
    }
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    // Define the attached object message
    moveit_msgs::msg::AttachedCollisionObject attached_object;
    attached_object.link_name = "panda_hand";
    attached_object.object.header.frame_id = "panda_hand";
    attached_object.object.id = "box";

    /* A default pose */
    geometry_msgs::msg::Pose pose;
    pose.position.z = 0.11;
    pose.orientation.w = 1.0;
    /* Define a box to be attached */
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.075;
    primitive.dimensions[1] = 0.075;
    primitive.dimensions[2] = 0.075;

    attached_object.object.primitives.push_back(primitive);
    attached_object.object.primitive_poses.push_back(pose);

    //since we're attachig the object ot the robot hand, collision checker must be
    // set to ignore
    // add an object into environment
    RCLCPP_INFO(LOGGER, "Adding the object into the world at the location of the hand");
    moveit_msgs::msg::PlanningScene planning_scene;
    planning_scene.world.collision_objects.push_back(attached_object.object);
    planning_scene.is_diff = true;
    planning_scene_diff_publisher->publish(planning_scene);
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");


    // Interlude: Synchronous vs Asynchronous updates
    // There are two separate mechanisms available to interact

    // * Send a diff via a rosservice call and block until
    //   the diff is applied (synchronous update)
    // * Send a diff via a topic, continue even though the diff
    //   might not be applied yet (asynchronous update)
    rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr planning_scene_diff_client =
    node->create_client<moveit_msgs::srv::ApplyPlanningScene>("apply_planning_scene");
    planning_scene_diff_client->wait_for_service();
    
    // and send the diffs to the planning scene via a service call:
    auto request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
    request->scene = planning_scene;
    auto response_future = planning_scene_diff_client->async_send_request(request);

    // wait for the service to respond
    std::chrono::seconds wait_time(1);
    std::future_status fs = response_future.wait_for(wait_time);
    if (fs == std::future_status::timeout){
        RCLCPP_ERROR(LOGGER, "Service timed out.");
    }else{
        auto planning_response = response_future.get();
        if (planning_response->success){
        RCLCPP_INFO(LOGGER, "Service successfully added object.");
        }else{
        RCLCPP_ERROR(LOGGER, "Service failed to add object.");
        }
    }

    // Attach an object to the robot
    // Attaching an object requires two operations
    //  * Removing the original object from the environment
    //  * Attaching the object to the robot
    
    /* First, define the REMOVE object message*/
    moveit_msgs::msg::CollisionObject remove_object;
    remove_object.id ="box";
    remove_object.header.frame_id = "panda_hand";
    remove_object.operation = remove_object.REMOVE;

     /* Carry out the REMOVE + ATTACH operation */
    RCLCPP_INFO(LOGGER, "Attaching the object to the hand and removing it from the world.");
    planning_scene.world.collision_objects.clear();
    planning_scene.world.collision_objects.push_back(remove_object);
    planning_scene.robot_state.attached_collision_objects.push_back(attached_object);
    planning_scene.robot_state.is_diff = true;
    planning_scene_diff_publisher->publish(planning_scene);
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");


      // Detach an object from the robot
    // Detaching an object from the robot requires two operations
    //  * Detaching the object from the robot
    //  * Re-introducing the object into the environmen

      /* First, define the DETACH object message*/
    moveit_msgs::msg::AttachedCollisionObject detach_object;
    detach_object.object.id = "box";
    detach_object.link_name = "panda_hand";
    detach_object.object.operation = attached_object.object.REMOVE;


      // Note how we make sure that the diff message contains no other
  // attached objects or collisions objects by clearing those fields
  // first.
  /* Carry out the DETACH + ADD operation */
  RCLCPP_INFO(LOGGER, "Detaching the object from the robot and returning it to the world.");
  planning_scene.robot_state.attached_collision_objects.clear();
  planning_scene.robot_state.attached_collision_objects.push_back(detach_object);
  planning_scene.robot_state.is_diff = true;
  planning_scene.world.collision_objects.clear();
  planning_scene.world.collision_objects.push_back(attached_object.object);
  planning_scene.is_diff = true;
  planning_scene_diff_publisher->publish(planning_scene);
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // Remove the object from the collision world
  // Removing the object from the collision world just requires
  // using the remove object message defined earlier.
  RCLCPP_INFO(LOGGER, "Removing the object from the world.");
  planning_scene.robot_state.attached_collision_objects.clear();
  planning_scene.world.collision_objects.clear();
  planning_scene.world.collision_objects.push_back(remove_object);
  planning_scene_diff_publisher->publish(planning_scene);
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to end the demo");

  rclcpp::shutdown();
}