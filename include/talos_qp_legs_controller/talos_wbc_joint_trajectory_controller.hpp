#ifndef TALOS_WBC_JOINT_TRAJECTORY_CONTROLLER_H
#define TALOS_WBC_JOINT_TRAJECTORY_CONTROLLER_H

#include <joint_trajectory_controller/joint_trajectory_controller.h>

namespace joint_trajectory_controller {

  template <class SegmentImpl, class HardwareInterface>
  class JointTrajectoryWholeBodyController : public JointTrajectoryController<SegmentImpl, HardwareInterface>
  {
  public:

    JointTrajectoryWholeBodyController();

    /** \name Non Real-Time Safe Functions
    *\{*/
    bool
    init(HardwareInterface* hw,
	 ros::NodeHandle& root_nh,
	 ros::NodeHandle& controller_nh) override;
    /*\}*/

    /** \name Real-Time Safe Functions
    *\{*/
    /** \brief Holds the current position. */
    void
    starting(const ros::Time& time) override;

    /** \brief Cancels the active action goal, if any. */
    void
    stopping(const ros::Time& time) override;

    void
    update(const ros::Time& time,
	   const ros::Duration& period) override;
    /*\}*/
  };
}

#include <talos_qp_legs_controller/talos_wbc_joint_trajectory_controller.hxx>


#endif /* TALOS_WBC_JOINT_TRAJECTORY_CONTROLLER_H */

