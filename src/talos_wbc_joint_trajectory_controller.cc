// Pluginlib
#include <pluginlib/class_list_macros.hpp>

// Project
#include <trajectory_interface/quintic_spline_segment.h>
#include <joint_trajectory_controller/joint_trajectory_controller.h>
#include <talos_qp_legs_controller/talos_wbc_joint_trajectory_controller.hpp>

namespace whole_body_controllers
{
  using SPLINE = trajectory_interface::QuinticSplineSegment<double>;
  using HWINT = hardware_interface::EffortJointInterface;

  typedef joint_trajectory_controller::JointTrajectoryWholeBodyController<SPLINE, HWINT>
  JointTrajectoryWholeBodyController;
  
}

PLUGINLIB_EXPORT_CLASS(whole_body_controllers::JointTrajectoryWholeBodyController,
		       controller_interface::ControllerBase)
