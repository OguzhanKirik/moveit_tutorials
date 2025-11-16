#pragma once

#include <moveit/robot_model/robot_model.h>
#include <geometry_msgs/msg/pose.hpp>
#include <vector>
#include <iosfwd>  // for std::ostream


namespace panda_low_level_demo{

    bool computeCustomIK(const moveit::core::RobotModelPtr& robot_model,
                        const std::string& planning_group,
                        const std::string& ee_link,
                        const geometry_msgs::msg::Pose& target_pose,
                        std::vector<double>& joint_solution,
                        std::ostream* debug_stream=nullptr);


} // namespace panda_low_level_demo
