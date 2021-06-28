#include <xpp_talos/talos_cartesian_joint_converter.h>

#include <ros/node_handle.h>

#include <xpp_msgs/RobotStateJoint.h>
#include <xpp_states/convert.h>

namespace xpp {

  TalosCartesianJointConverter::TalosCartesianJointConverter (const InverseKinematicsTalos::Ptr& ik,
							      const std::string& cart_topic,
							      const std::string& joint_topic)
  {
    inverse_kinematics_ = ik;

    ::ros::NodeHandle n;
    cart_state_sub_ = n.subscribe(cart_topic, 1, &TalosCartesianJointConverter::StateCallback, this);
    ROS_DEBUG("Subscribed to: %s", cart_state_sub_.getTopic().c_str());

    joint_state_pub_  = n.advertise<xpp_msgs::RobotStateJoint>(joint_topic, 1);
    ROS_DEBUG("Publishing to: %s", joint_state_pub_.getTopic().c_str());
  }

  void
  TalosCartesianJointConverter::StateCallback (const xpp_msgs::RobotStateCartesian& cart_msg)
  {
    auto cart = Convert::ToXpp(cart_msg);

    // transform feet from world -> base frame
    Eigen::Matrix3d B_R_W = cart.base_.ang.q.normalized().toRotationMatrix().inverse();
    // Eigen::Matrix3d B_R_W;
    // B_R_W << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    EndeffectorsPos ee_B(cart.ee_motion_.GetEECount());
    EndeffectorsRot ee_R(cart.ee_motion_.GetEECount());

    auto ee = ee_B.GetEEsOrdered();

    // Check if size is at least 2
    if (ee.size() < 2) return;

    int j = 0;
    for (auto ee_id : ee){
      if (j >= 2) break;

      ee_B.at(ee_id) = B_R_W * (cart.ee_motion_.at(ee_id).p_ - cart.base_.lin.p_);
      ee_R.at(ee_id) = B_R_W;

      ++j;
    }

    Eigen::VectorXd q =  inverse_kinematics_->GetAllJointAngles(ee_B, ee_R).ToVec();

    xpp_msgs::RobotStateJoint joint_msg;
    joint_msg.base            = cart_msg.base;
    joint_msg.ee_contact      = cart_msg.ee_contact;
    joint_msg.time_from_start = cart_msg.time_from_start;
    joint_msg.joint_state.position = std::vector<double>(q.data(), q.data()+q.size());
    // Attention: Not filling joint velocities or torques

    joint_state_pub_.publish(joint_msg);
  }

} /* namespace xpp */
