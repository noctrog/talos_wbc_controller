#ifndef XPP_TALOS_CARTESIAN_JOINT_CONVERTER_H_
#define XPP_TALOS_CARTESIAN_JOINT_CONVERTER_H_

#include <string>

#include <ros/publisher.h>
#include <ros/subscriber.h>

#include <xpp_msgs/RobotStateCartesian.h>

#include <xpp_talos/inverse_kinematics_talos.h>

namespace xpp {

/**
 * @brief Converts a Cartesian robot representation to joint angles for Talos robot.
 *
 * This class subscribes to a Cartesian robot state message and publishes
 * the, through inverse kinematics converted, joint state message. This
 * can then be used to visualize URDFs in RVIZ.
 */
class TalosCartesianJointConverter {
public:
  /**
   * @brief Creates a converter initializing the subscriber and publisher.
   * @param  ik  The %InverseKinematics to use for conversion.
   * @param  cart_topic  The ROS topic containing the Cartesian robot state.
   * @param  joint_topic The ROS topic to publish for the URDF visualization.
   */
  TalosCartesianJointConverter (const InverseKinematicsTalos::Ptr& ik,
				const std::string& cart_topic,
				const std::string& joint_topic);
  virtual ~TalosCartesianJointConverter () = default;

private:
  void StateCallback(const xpp_msgs::RobotStateCartesian& msg);

  ros::Subscriber cart_state_sub_;
  ros::Publisher  joint_state_pub_;

  InverseKinematicsTalos::Ptr inverse_kinematics_;
};

} /* namespace xpp */

#endif /* XPP_VIS_CARTESIAN_JOINT_CONVERTER_H_ */
