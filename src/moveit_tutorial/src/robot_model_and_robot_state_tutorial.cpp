// Moveit 

#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    // This enables loading undeclared parameters
    // best practice would be to declare parameters in the corresponding classes
    // and provide descriptions about expected use
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("robot_model_and_state_tutorial", node_options);
    const auto& LOGGER = node->get_logger();


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

    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

    // Get Joint Values
    //  Retrieve the currect set of joint vales stored in the sate for the Panda arm
    std::vector<double> joint_values;
    robot_state->copyJointGroupPositions(joint_model_group, joint_values);
    for(std::size_t i=0;i<joint_names.size();++i){
        RCLCPP_INFO(LOGGER, "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }

    // Joint Limits
    // setJointGroupPositions() does not enforce joint limits by itself, but a call to enforceBounds() will do it.
    /* Set one joint in the Panda arm outside its joint limit */
    joint_values[0] = 5.57;
    robot_state->setJointGroupPositions(joint_model_group, joint_values);

    // check whether any joint is outside its joint s limits
    RCLCPP_INFO_STREAM(LOGGER, "Current state is " << (robot_state->satisfiesBounds() ? "valid" : "not valid"));

    // Enforce the joint limits for this state and check again
    robot_state->enforceBounds();
    RCLCPP_INFO_STREAM(LOGGER, "Current state is " << (robot_state->satisfiesBounds() ? "valid" : "not valid"));


    // Forward Kinematics
    // calculate the pose of the panda_link8 from panda_arm
    robot_state->setToRandomPositions(joint_model_group);
    const Eigen::Isometry3d& end_effector_state = robot_state->getGlobalLinkTransform("panda_link8");

    /* Print end-effector pose. Remember that this is in the model frame */
    RCLCPP_INFO_STREAM(LOGGER, "Translation: \n" << end_effector_state.translation() << "\n");
    RCLCPP_INFO_STREAM(LOGGER, "Rotation: \n" << end_effector_state.rotation() << "\n");

    // Inverse Kinematics
    //Solve IK for the panda robot:
      //  * The desired pose of the end-effector (by default, this is the last link in the "panda_arm" chain):
    //    end_effector_state that we computed in the step above.
    //  * The timeout: 0.1 s

    double timeout = 0.1;
    bool found_ik = robot_state->setFromIK(joint_model_group, end_effector_state, timeout);
    // Now, we can print out the IK solution (if found):
    if (found_ik)
    {
        robot_state->copyJointGroupPositions(joint_model_group, joint_values);
        for (std::size_t i = 0; i < joint_names.size(); ++i)
        {
        RCLCPP_INFO(LOGGER, "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
        }
    }
    else
    {
        RCLCPP_INFO(LOGGER, "Did not find IK solution");
    }

    // Get the Jacobian
    // ^^^^^^^^^^^^^^^^
    // We can also get the Jacobian from the
    // :moveit_codedir:`RobotState<moveit_core/robot_state/include/moveit/robot_state/robot_state.hpp>`.
    Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
    Eigen::MatrixXd jacobian;
    robot_state->getJacobian(joint_model_group, robot_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                            reference_point_position, jacobian);
    RCLCPP_INFO_STREAM(LOGGER, "Jacobian: \n" << jacobian << "\n");
    // END_TUTORIAL



    return 0;
}
