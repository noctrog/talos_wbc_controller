#include <pwd.h>
#include <sys/types.h>
#include <unistd.h>

#include <iostream>

#include <xpp_talos/inverse_kinematics_talos.h>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <xpp_msgs/RobotStateCartesian.h>
#include <xpp_states/convert.h>

#include <talos_wbc_controller/JointContactTrajectory.h>
#include <talos_wbc_controller/JointContactTrajectoryContacts.h>

typedef typename std::vector<bool> CurrentContacts;

/**
 * Takes a RobotStateCartesian and calculates the inverse kinematics
 * for the Talos robot
 */
void GetJointStates(const xpp_msgs::RobotStateCartesian::ConstPtr i,
		    const xpp::InverseKinematicsTalos &ik,
		    Eigen::VectorXd& q,
		    Eigen::VectorXd& qd,
		    Eigen::VectorXd& qdd) {
  auto cart = xpp::Convert::ToXpp(*i);

  // transform feet from world -> base frame
  Eigen::Matrix3d B_R_W =
      cart.base_.ang.q.normalized().toRotationMatrix().inverse();
  xpp::EndeffectorsPos ee_B_pos(cart.ee_motion_.GetEECount());
  xpp::EndeffectorsPos ee_B_vel(cart.ee_motion_.GetEECount());
  xpp::EndeffectorsPos ee_B_acc(cart.ee_motion_.GetEECount());
  xpp::EndeffectorsRot ee_R(cart.ee_motion_.GetEECount());
  for (auto ee : ee_B_pos.GetEEsOrdered()) {
    ee_B_pos.at(ee) = B_R_W * (cart.ee_motion_.at(ee).p_ - cart.base_.lin.p_);
    ee_B_vel.at(ee) = B_R_W * (cart.ee_motion_.at(ee).v_ - cart.base_.lin.v_);
    ee_B_acc.at(ee) = B_R_W * (cart.ee_motion_.at(ee).a_ - cart.base_.lin.a_); // TODO: Coriolis effect
    ee_R.at(ee) = B_R_W;
  }

  q   = ik.GetAllJointAngles(ee_B_pos, ee_R).ToVec();
  qd  = ik.GetAllJointVelocities(ee_B_vel, q).ToVec();
  qdd = ik.GetAllJointAccelerations(ee_B_acc, q, qd).ToVec();
}

void GetCenterOfMassState(const xpp_msgs::RobotStateCartesian::ConstPtr i,
			  const xpp::InverseKinematicsTalos& ik,
			  const Eigen::VectorXd& q_joints,
			  const Eigen::VectorXd& qd_joints,
			  Eigen::Vector3d& com_pos,
			  Eigen::Vector3d& com_vel)
{
  auto cart = xpp::Convert::ToXpp(*i);

  Eigen::VectorXd q(7 + q_joints.size());
  q.head(7) << cart.base_.lin.p_, cart.base_.ang.q.normalized().coeffs();
  q.tail(q_joints.size()) = q_joints;

  // Pinocchio free floating base velocity is with respect to the base robot frame
  Eigen::Quaterniond quat;
  quat.x() = q(3); quat.y() = q(4); quat.z() = q(5); quat.z() = q(6);
  Eigen::Matrix3d R = quat.normalized().toRotationMatrix().inverse();

  // Convert velocities
  Eigen::Vector3d twist_linear, twist_angular;
  twist_linear << cart.base_.lin.v_;
  twist_angular << cart.base_.ang.w;
  twist_linear = R * twist_linear;
  twist_angular = R * twist_angular;
  Eigen::VectorXd qd(6 + qd_joints.size());
  qd.head(6) << twist_linear, twist_angular;
  qd.tail(qd_joints.size()) = qd_joints;

  ik.GetCenterOfMassPositionAndVelocity(q, qd, com_pos, com_vel);
}

void GetCenterOfMassState(const xpp_msgs::RobotStateCartesian::ConstPtr i,
			  const xpp::InverseKinematicsTalos& ik,
			  const Eigen::VectorXd& q_joints,
			  const Eigen::VectorXd& qd_joints,
			  const Eigen::VectorXd& qdd_joints,
			  Eigen::Vector3d& com_pos,
			  Eigen::Vector3d& com_vel,
			  Eigen::Vector3d& com_acc)
{
  auto cart = xpp::Convert::ToXpp(*i);

  Eigen::VectorXd q(7 + q_joints.size());
  q.head(7) << cart.base_.lin.p_, cart.base_.ang.q.normalized().coeffs();
  q.tail(q_joints.size()) = q_joints;

  // Pinocchio free floating base velocity is with respect to the base robot frame
  Eigen::Quaterniond quat;
  quat.x() = q(3); quat.y() = q(4); quat.z() = q(5); quat.z() = q(6);
  Eigen::Matrix3d R = quat.normalized().toRotationMatrix().inverse();

  // Convert velocities
  Eigen::Vector3d twist_linear, twist_angular;
  twist_linear << cart.base_.lin.v_;
  twist_angular << cart.base_.ang.w;
  twist_linear = R * twist_linear;
  twist_angular = R * twist_angular;
  Eigen::VectorXd qd(6 + qd_joints.size());
  qd.head(6) << twist_linear, twist_angular;
  qd.tail(qd_joints.size()) = qd_joints;

  // Convert accelerations
  Eigen::Vector3d acc_linear, acc_angular;
  acc_linear << cart.base_.lin.a_;
  acc_angular << cart.base_.ang.wd;
  acc_linear = R * acc_linear;
  acc_angular = R * acc_angular;
  Eigen::VectorXd qdd(6 + qdd_joints.size());
  qdd.head(6) << acc_linear, acc_angular;
  qdd.tail(qdd_joints.size()) = qdd_joints;

  ik.GetCenterOfMassPositionVelocityAcceleration(q, qd, qdd, com_pos, com_vel, com_acc);
}

/** 
 * Takes a RobotStateCartesian and retrieves an std::vector<bool>
 * representing the current contacts of the end effectos.
 */
CurrentContacts GetCurrentContact(const xpp_msgs::RobotStateCartesian::ConstPtr i)
{
  const auto cart = xpp::Convert::ToXpp(*i);
  const auto n_ee = cart.ee_motion_.GetEECount();

  // One trajectory per end effector
  CurrentContacts contact_traj(n_ee);

  // Retrieve the current contacts
  for (size_t j = 0; j < n_ee; ++j) {
    contact_traj[j] = i->ee_contact[j];
  }

  return contact_traj;
}

/**
 * Takes a trajectory msg and assigns the corresponding joints for
 * both legs
 */
void PrepareTrajMsg(talos_wbc_controller::JointContactTrajectory &msg) {
  msg.header.frame_id = "";
  msg.header.stamp = ros::Time::now();

  // Ordinary trajectory names
  msg.trajectory.joint_names.emplace_back("leg_left_1_joint");
  msg.trajectory.joint_names.emplace_back("leg_left_2_joint");
  msg.trajectory.joint_names.emplace_back("leg_left_3_joint");
  msg.trajectory.joint_names.emplace_back("leg_left_4_joint");
  msg.trajectory.joint_names.emplace_back("leg_left_5_joint");
  msg.trajectory.joint_names.emplace_back("leg_left_6_joint");
  msg.trajectory.joint_names.emplace_back("leg_right_1_joint");
  msg.trajectory.joint_names.emplace_back("leg_right_2_joint");
  msg.trajectory.joint_names.emplace_back("leg_right_3_joint");
  msg.trajectory.joint_names.emplace_back("leg_right_4_joint");
  msg.trajectory.joint_names.emplace_back("leg_right_5_joint");
  msg.trajectory.joint_names.emplace_back("leg_right_6_joint");

  // Center of mass trajectory
  msg.com_trajectory.joint_names.emplace_back("center_of_mass_x");
  msg.com_trajectory.joint_names.emplace_back("center_of_mass_y");
  msg.com_trajectory.joint_names.emplace_back("center_of_mass_z");

  // Contact names
  msg.contact_link_names.emplace_back("left_sole_link");
  msg.contact_link_names.emplace_back("right_sole_link");
}

/**
 * Takes a ROS bag of optimization results, and generates a
 * trajectory_msgs/JointTrajectory
 */
int main(int argc, char *argv[]) {
  ros::init(argc, argv, "rosbag_trajectory_player");
  ros::NodeHandle nh;

  auto pub = nh.advertise<talos_wbc_controller::JointContactTrajectory>(
      "/talos_trajectory_wbc_controller/command", 100);

  // Get bag file, if not specified, use towr default bag
  std::string name;
  if (argc == 1) {
    struct passwd *pw = getpwuid(getuid());
    std::string home_dir(pw->pw_dir);
    name = home_dir + "/.ros/towr_trajectory";
  } else {
    name = argv[1];
  }

  // Open bag
  rosbag::Bag bag_r;
  bag_r.open(name + ".bag", rosbag::bagmode::Read);
  ROS_INFO_STREAM("Reading from bag " + bag_r.getFileName());

  // Topic to load
  std::string robot_state_topic("/xpp/state_des");
  std::vector<std::string> topics;
  topics.push_back(robot_state_topic);

  rosbag::View view(bag_r, rosbag::TopicQuery(topics));

  // Prepare trajectory messages
  talos_wbc_controller::JointContactTrajectory traj;
  PrepareTrajMsg(traj);

  // TODO automatically get frecuency
  ROS_INFO_STREAM("Calculating joint positions...");
  xpp::InverseKinematicsTalos ik;
  size_t current_t = 0;
  for (rosbag::MessageInstance const m : view) {
    xpp_msgs::RobotStateCartesian::ConstPtr i =
	m.instantiate<xpp_msgs::RobotStateCartesian>();

    if (i) {
      // Perform inverse kinematics
      Eigen::VectorXd q, qd, qdd;
      Eigen::Vector3d com_pos, com_vel, com_acc;
      GetJointStates(i, ik, q, qd, qdd);
      GetCenterOfMassState(i, ik, q, qd, qdd, com_pos, com_vel, com_acc);

      // Calculate corresponding time for position
      traj.trajectory.points.emplace_back();
      traj.trajectory.points.back().time_from_start = i->time_from_start;
      // Add joint values to JointTrajectory
      for (int i = 0; i < 12; ++i) {
	traj.trajectory.points.back().positions.push_back(q[i]);
	traj.trajectory.points.back().velocities.push_back(qd[i]);
	traj.trajectory.points.back().accelerations.push_back(qdd[i]);
      }
      // Center of mass point
      traj.com_trajectory.points.emplace_back();
      traj.com_trajectory.points.back().time_from_start = i->time_from_start;
      // Add com values to JointTrajectory
      for (int i = 0; i < 3; ++i) {
	traj.com_trajectory.points.back().positions.push_back(com_pos(i));
	traj.com_trajectory.points.back().velocities.push_back(com_vel(i));
	traj.com_trajectory.points.back().accelerations.push_back(com_acc(i));
      }

      // Add values to the contact sequence
      CurrentContacts curr_contacts = GetCurrentContact(i);
      talos_wbc_controller::JointContactTrajectoryContacts cont_msg;
      cont_msg.contacts.resize(2);
      for (int i = 0; i < 2; ++i)
	cont_msg.contacts.at(i) = curr_contacts.at(i);
      cont_msg.time_from_start = traj.trajectory.points.back().time_from_start;
      traj.contacts.push_back(cont_msg);
    }

    current_t++;
  }

  // Publish trajectory msgs
  ROS_INFO_STREAM("Publishing messages");

  // Wait for publishers to be ready
  ros::Duration(0.2).sleep();

  // Play optimized trajectory
  pub.publish(traj);

  ros::spinOnce();

  ROS_INFO_STREAM("Done.");
  return 0;
}
