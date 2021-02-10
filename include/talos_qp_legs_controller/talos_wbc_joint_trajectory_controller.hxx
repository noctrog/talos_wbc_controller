#ifndef TALOS_WBC_JOINT_TRAJECTORY_CONTROLLER_HXX
#define TALOS_WBC_JOINT_TRAJECTORY_CONTROLLER_HXX

namespace joint_trajectory_controller {

template <class SegmentImpl, class HardwareInterface>
JointTrajectoryWholeBodyController<
    SegmentImpl, HardwareInterface>::JointTrajectoryWholeBodyController()
    : JointTrajectoryController<SegmentImpl, HardwareInterface>() {}

template <class SegmentImpl, class HardwareInterface>
bool JointTrajectoryWholeBodyController<SegmentImpl, HardwareInterface>::init(
    HardwareInterface *hw, ros::NodeHandle &root_nh,
    ros::NodeHandle &controller_nh) {

  return JointTrajectoryController<SegmentImpl, HardwareInterface>::init(
      hw, root_nh, controller_nh);
}

template <class SegmentImpl, class HardwareInterface>
void JointTrajectoryWholeBodyController<
    SegmentImpl, HardwareInterface>::starting(const ros::Time &time) {

  JointTrajectoryController<SegmentImpl, HardwareInterface>::starting(time);
}

template <class SegmentImpl, class HardwareInterface>
void JointTrajectoryWholeBodyController<
    SegmentImpl, HardwareInterface>::stopping(const ros::Time &time) {

  JointTrajectoryController<SegmentImpl, HardwareInterface>::stopping(time);
}

template <class SegmentImpl, class HardwareInterface>
void JointTrajectoryWholeBodyController<SegmentImpl, HardwareInterface>::update(
    const ros::Time &time, const ros::Duration &period) {

  JointTrajectoryController<SegmentImpl, HardwareInterface>::update(time,
								    period);
}

} // namespace joint_trajectory_controller

#endif /* TALOS_WBC_JOINT_TRAJECTORY_CONTROLLER_HXX */
