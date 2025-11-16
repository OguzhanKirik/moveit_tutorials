#include "moveit_demo/custom_ik_solver.hpp"

#include <moveit/robot_state/robot_state.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace panda_low_level_demo
{

bool computeCustomIK(const moveit::core::RobotModelPtr& robot_model,
                     const std::string& planning_group,
                     const std::string& ee_link,
                     const geometry_msgs::msg::Pose& target_pose,
                     std::vector<double>& joint_solution,
                     std::ostream* debug_stream)
{
  const moveit::core::JointModelGroup* jmg =
      robot_model->getJointModelGroup(planning_group);
  if (!jmg)
    return false;

  moveit::core::RobotState state(robot_model);
  state.setToDefaultValues();
  state.update();

  const std::size_t dof = jmg->getVariableCount();
  joint_solution.resize(dof);

  // Convert target pose to Eigen
  Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
  target.translation().x() = target_pose.position.x;
  target.translation().y() = target_pose.position.y;
  target.translation().z() = target_pose.position.z;

  Eigen::Quaterniond q_target(
      target_pose.orientation.w,
      target_pose.orientation.x,
      target_pose.orientation.y,
      target_pose.orientation.z);
  target.linear() = q_target.toRotationMatrix();

  const double position_tolerance = 1e-2;   // 1 mm
  const int max_iters = 200;
  const double step_size = 0.5;            // gradient step scaling

  for (int iter = 0; iter < max_iters; ++iter)
  {
    const Eigen::Isometry3d& current = state.getGlobalLinkTransform(ee_link);
    Eigen::Vector3d err_pos = target.translation() - current.translation();
    double err_norm = err_pos.norm();

    if (debug_stream && iter % 20 == 0)
      (*debug_stream) << "[IK] iter " << iter << ", pos error = " << err_norm << "\n";

    if (err_norm < position_tolerance)
    {
      state.copyJointGroupPositions(jmg, joint_solution);
      return true;
    }

    Eigen::MatrixXd J;
    state.getJacobian(jmg,
                      state.getLinkModel(ee_link),
                      Eigen::Vector3d::Zero(),
                      J);

    Eigen::MatrixXd J_pos = J.topRows<3>();
    Eigen::VectorXd dq = step_size * J_pos.transpose() * err_pos;

    std::vector<double> q;
    state.copyJointGroupPositions(jmg, q);
    for (std::size_t i = 0; i < dof && i < static_cast<std::size_t>(dq.size()); ++i)
      q[i] += dq[i];

    state.setJointGroupPositions(jmg, q);
    state.enforceBounds(jmg);
    state.update();
  }

  if (debug_stream)
    (*debug_stream) << "[IK] Did not converge within " << max_iters << " iterations\n";

  return false;
}

}  // namespace panda_low_level_demo
