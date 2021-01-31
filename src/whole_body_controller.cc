#include <talos_qp_legs_controller/whole_body_controller.h>

namespace whole_body_controller_ns {
  bool WholeBodyController::init(hardware_interface::PositionJointInterface *hw, ros::NodeHandle &nh)
  {
    std::vector<std::string> joint_names;

    // If parameters are not loaded to the parameter server, throw an error. Otherwise load them
    if (!nh.getParam("joints", joint_names)) {
      std::string error_string = "Failed to load " + nh.getNamespace() + "/joints from parameter server";
      ROS_ERROR_STREAM(error_string);
      return false;
    }

    // Save joint names
    for(size_t i = 0; i < joint_names.size(); ++i) {
      joint_handles_.push_back(hw->getHandle(joint_names.at(i)));
    }

    return true;
  }

  void WholeBodyController::starting(const ros::Time &time)
  {
    init_pos_.clear();
    init_pos_.reserve(joint_handles_.size());

    for (size_t i = 0; i < joint_handles_.size(); i++) {
      init_pos_.push_back(joint_handles_[i].getPosition());
    }
  }

  void WholeBodyController::stopping(const ros::Time &time)
  {
    for (auto handle : joint_handles_) {
      handle.setCommand(0.0);
    }
  }

  void WholeBodyController::update(const ros::Time &time, const ros::Duration &period)
  {
    joint_handles_[3].setCommand(50 * (1.0 - joint_handles_[3].getPosition()));
  }
}

// Register the plugin
PLUGINLIB_EXPORT_CLASS(whole_body_controller_ns::WholeBodyController, controller_interface::ControllerBase);
