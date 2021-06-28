#include <xpp_talos/inverse_kinematics_talos.h>
#include <xpp_msgs/topic_names.h>

#include <xpp_talos/talos_cartesian_joint_converter.h>
#include <xpp_vis/urdf_visualizer.h>

#include <xpp_states/endeffector_mappings.h>

#include <ros/ros.h>

using namespace xpp;

int main(int argc, char *argv[]) {
  ::ros::init(argc, argv, "talos_urdf_visualizer");
  const std::string joint_desired_talos = "xpp/joint_talos_des";

  auto ik = std::make_shared<InverseKinematicsTalos>();
  TalosCartesianJointConverter inv_kin_converter(ik,
						 xpp_msgs::robot_state_desired,
						 joint_desired_talos);

  std::vector<UrdfVisualizer::URDFName> joint_names(12);
  joint_names.at(LL1) = "leg_left_1_joint";
  joint_names.at(LL2) = "leg_left_2_joint";
  joint_names.at(LL3) = "leg_left_3_joint";
  joint_names.at(LL4) = "leg_left_4_joint";
  joint_names.at(LL5) = "leg_left_5_joint";
  joint_names.at(LL6) = "leg_left_6_joint";
  joint_names.at(LR1) = "leg_right_1_joint";
  joint_names.at(LR2) = "leg_right_2_joint";
  joint_names.at(LR3) = "leg_right_3_joint";
  joint_names.at(LR4) = "leg_right_4_joint";
  joint_names.at(LR5) = "leg_right_5_joint";
  joint_names.at(LR6) = "leg_right_6_joint";

  std::string urdf = "talos_rviz_urdf_robot_description";
  UrdfVisualizer node(urdf, joint_names, "base_link", "world",
		      joint_desired_talos, "talos");

  ::ros::spin();

  return 1;
}
