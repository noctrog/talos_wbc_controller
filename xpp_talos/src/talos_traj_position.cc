#include <pwd.h>
#include <sys/types.h>
#include <unistd.h>

#include <iostream>

#include <xpp_talos/inverse_kinematics_talos.h>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <xpp_msgs/RobotStateCartesian.h>
#include <xpp_states/convert.h>

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
    ee_B_acc.at(ee) = B_R_W * (cart.ee_motion_.at(ee).a_ - cart.base_.lin.a_);
    ee_R.at(ee) = B_R_W;
  }

  q   = ik.GetAllJointAngles(ee_B_pos, ee_R).ToVec();
  qd  = ik.GetAllJointVelocities(ee_B_vel, q).ToVec();
  qdd = ik.GetAllJointAccelerations(ee_B_acc, q, qd).ToVec();
}
/**
 * Takes a trajectory msg and assigns the corresponding joints for
 * both legs
 */
void PrepareTrajMsg(trajectory_msgs::JointTrajectory &msg) {
  msg.header.frame_id = "";
  msg.header.stamp = ros::Time::now();
  msg.joint_names.emplace_back("leg_left_1_joint");
  msg.joint_names.emplace_back("leg_left_2_joint");
  msg.joint_names.emplace_back("leg_left_3_joint");
  msg.joint_names.emplace_back("leg_left_4_joint");
  msg.joint_names.emplace_back("leg_left_5_joint");
  msg.joint_names.emplace_back("leg_left_6_joint");
  msg.joint_names.emplace_back("leg_right_1_joint");
  msg.joint_names.emplace_back("leg_right_2_joint");
  msg.joint_names.emplace_back("leg_right_3_joint");
  msg.joint_names.emplace_back("leg_right_4_joint");
  msg.joint_names.emplace_back("leg_right_5_joint");
  msg.joint_names.emplace_back("leg_right_6_joint");
}
/**
 * Takes a ROS bag of optimization results, and generates a
 * trajectory_msgs/JointTrajectory
 */
int main(int argc, char *argv[]) {
  ros::init(argc, argv, "rosbag_trajectory_player");
  ros::NodeHandle nh;

  // auto pub_l =
  // nh.advertise<trajectory_msgs::JointTrajectory>("/left_leg_controller/command",
  // 100); auto pub_r =
  // nh.advertise<trajectory_msgs::JointTrajectory>("/right_leg_controller/command",
  // 100);
  auto pub = nh.advertise<trajectory_msgs::JointTrajectory>(
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
  // Move Talos to initial pose
  trajectory_msgs::JointTrajectory init_traj;
  PrepareTrajMsg(init_traj);
  // Perform movement
  trajectory_msgs::JointTrajectory traj;
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
      GetJointStates(i, ik, q, qd, qdd);

      // Calculate corresponding time for position
      traj.points.emplace_back();
      traj.points.back().time_from_start = i->time_from_start;
      // Add values to JointTrajectory
      for (int i = 0; i < 12; ++i) {
	// Positions
	traj.points.back().positions.push_back(q[i]);
      }

      // If this is the initial pose, save it in the initialization
      // trajectory
      if (current_t == 0) {
	// Talos initial pose, all zeros
	init_traj.points.emplace_back();
	init_traj.points.back().time_from_start = ros::Duration(0.1);
	init_traj.points.back().positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
					     0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

	// Trajectory initial pose
	init_traj.points.emplace_back(traj.points.back());
	init_traj.points.back().time_from_start = ros::Duration(1.0);
      }
    }

    current_t++;
  }

  // Publish trajectory msgs
  ROS_INFO_STREAM("Publishing messages");

  // Wait for publishers to be ready
  ros::Duration(0.2).sleep();

  // Initialize robot
  pub.publish(init_traj);
  ros::Duration(1.5).sleep();

  // Play optimized trajectory
  pub.publish(traj);

  ros::spinOnce();

  ROS_INFO_STREAM("Done.");
  return 0;
}
