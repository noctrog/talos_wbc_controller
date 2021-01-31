#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

namespace whole_body_controller_ns {

class WholeBodyController : public controller_interface::Controller<
				hardware_interface::PositionJointInterface> {
  public:
    bool init(hardware_interface::PositionJointInterface *hw, ros::NodeHandle &n) override;
    void update(const ros::Time &time, const ros::Duration &period) override;
    void starting(const ros::Time &time) override;
    void stopping(const ros::Time &time) override;

  private:
    std::vector<hardware_interface::JointHandle> joint_handles_;
    std::vector<double> init_pos_;
};
} // namespace whole_body_controller_ns
