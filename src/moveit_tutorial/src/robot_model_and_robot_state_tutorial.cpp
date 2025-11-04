// Moveit 

#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/erobot_state.hpp>

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    // This enables loading undeclared parameters
    // best practice would be to declare parameters in the corresponding classes
    // and provide descriptions about expected use
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("robot_model_and_state_tutorial", node_options);
    conts auto& LOGGER = node->get_logger();


    // Setting up to start using the Robotmodel class is very easy. In general, you iwll find
    // foind that most hgier-level componenets will return a shared pointer to the RobotModel
    // You should always use that when possible. 

    // Instantiate a RobotModelLoader
      // object, which will look up
    // the robot description on the ROS parameter server and construct a
    // :moveit_codedir:`RobotModel<moveit_core/robot_model/include/moveit/robot_model/robot_model.hpp>` for us to use.
    //
    // .. _RobotModelLoader:
    //   
    //     https://github.com/moveit/moveit2/blob/main/moveit_ros/planning/robot_model_loader/include/moveit/robot_model_loader/robot_model_loader.hpp
    robot_model_loader::RobotModelLoader robot_model_loader(node);
    const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
    RCLCPP_INFO(LOGGER, "Model frame: %s", kinematic_model->getModelFrame().c_str());

    moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(kinematic_model));
    robot_state->setToDefaultValues();
    const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("panda_arm");
    



    return 0;
}
