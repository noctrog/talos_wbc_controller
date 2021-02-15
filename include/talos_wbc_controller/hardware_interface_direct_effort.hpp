#ifndef HARDWARE_INTERFACE_DIRECT_EFFORT_H
#define HARDWARE_INTERFACE_DIRECT_EFFORT_H

#include <cassert>
#include <string>
#include <vector>

#include <boost/shared_ptr.hpp>

#include <ros/node_handle.h>
#include <ros/time.h>

#include <hardware_interface/joint_command_interface.h>

/**
 * \brief Directly applies the desired torque calculated from the
 * JointTrajectoryWholeBodyController class.
 */
template <class State> class DirectEffortHardwareInterfaceAdapter {
public:
  DirectEffortHardwareInterfaceAdapter() : joint_handles_ptr_(0) {}

  bool init(std::vector<hardware_interface::JointHandle> &joint_handles,
	    ros::NodeHandle &controller_nh) {
    // Store pointer to joint handles
    joint_handles_ptr_ = &joint_handles;

    return true;
  }

  void starting(const ros::Time & /*time*/) {
    if (!joint_handles_ptr_) {
      return;
    }

    // Zero effort commands
    for (auto &handle : *joint_handles_ptr_) {
      handle.setCommand(0.0);
    }
  }

  void stopping(const ros::Time & /*time*/) {}

  void updateCommand(const ros::Time & /*time*/,
		     const ros::Duration & /*period*/,
		     const State &desired_state,
		     const State & /*state_error*/) {
    const unsigned int n_joints = joint_handles_ptr_->size();

    // Preconditions
    if (!joint_handles_ptr_) {
      return;
    }
    assert(n_joints == state_error.position.size());
    assert(n_joints == state_error.velocity.size());

    // Update efforts
    for (size_t i = 0; i < n_joints; ++i) {
      const double command = desired_state.effort[i];
      (*joint_handles_ptr_)[i].setCommand(command);
    }
  }

private:
  std::vector<hardware_interface::JointHandle> *joint_handles_ptr_;
};

#endif /* HARDWARE_INTERFACE_DIRECT_EFFORT_H */
