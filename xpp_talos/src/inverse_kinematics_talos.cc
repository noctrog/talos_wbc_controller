#include <cmath>
#include <iostream>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/kinematics-derivatives.hpp>
#include <xpp_talos/inverse_kinematics_talos.h>

#include <xpp_states/endeffector_mappings.h>

#include <ros/ros.h>
#include <ros/package.h>

namespace xpp {

  InverseKinematicsTalos::InverseKinematicsTalos()
  {
    // Create model and data objects
    talos_model_.reset(new pinocchio::Model);
    talos_data_.reset(new pinocchio::Data);

    // Load robot model
    ROS_INFO("Loading URDF model...");
    std::string xpp_talos_path = ros::package::getPath("xpp_talos");
    if (xpp_talos_path.size() == 0) {
      ROS_ERROR("You need to install the xpp_talos package: https://github.com/noctrog/xpp_talos");
      exit(-1);
    }
    std::string urdf_path = xpp_talos_path + "/urdf/talos_full_legs_v2.urdf";
    std::cout << urdf_path << std::endl;
    // JointModelFreeFlyer indicates that the root of the robot is not fixed to the world
    pinocchio::urdf::buildModel(urdf_path, pinocchio::JointModelFreeFlyer(), *talos_model_);
    ROS_INFO("Pinocchio model loaded success, robot name: %s", talos_model_->name.c_str());

    // Initialize pinocchio model data
    *talos_data_ = pinocchio::Data(*talos_model_);

    left_sole_id = talos_model_->getFrameId("left_sole_link");
    right_sole_id = talos_model_->getFrameId("right_sole_link");
  }

  Joints 
  InverseKinematicsTalos::GetAllJointAngles(const EndeffectorsPos& pos_B) const
  {
    using namespace biped;
    std::vector<Eigen::VectorXd> q_vec;

    // make sure always exactly 2 elements
    auto x_biped_B = pos_B.ToImpl();
    x_biped_B.resize(2, x_biped_B.front());

    /* TODO: Los vectores a la base <02-11-20, Ramón Calvo González> */
    q_vec.emplace_back(legs.GetJointAngles(LEFTLEG, 
					   x_biped_B.at(L) + Vector3d(0.0, 0.0, 0.0)));
    q_vec.emplace_back(legs.GetJointAngles(RIGHTLEG, 
					   x_biped_B.at(R) + Vector3d(0.0, 0.0, 0.0)));

    return Joints(q_vec);
  }

  Joints 
  InverseKinematicsTalos::GetAllJointAngles(const EndeffectorsPos& pos_B,
					    const EndeffectorsRot& rot_B) const
  {
    using namespace biped;
    std::vector<Eigen::VectorXd> q_vec;

    // make sure always exactly 2 elements
    auto x_biped_B = pos_B.ToImpl();
    x_biped_B.resize(2, x_biped_B.front());

    /* TODO: Los vectores a la base <02-11-20, Ramón Calvo González> */
    q_vec.emplace_back(legs.GetJointAngles(LEFTLEG, 
					   x_biped_B.at(L) + Vector3d(0.0, 0.0, 0.0),
					   rot_B.at(L)));
    q_vec.emplace_back(legs.GetJointAngles(RIGHTLEG, 
					   x_biped_B.at(R) + Vector3d(0.0, 0.0, 0.0),
					   rot_B.at(R)));

    return Joints(q_vec);
  }

  Joints
  InverseKinematicsTalos::GetAllJointVelocities(const EndeffectorsVel &vel_B,
			       const Joints &pos_j) const
  {
    // Retrieve the end effectors velocities (towr does not calculate orientations)
    Eigen::VectorXd left_sole_vel(6), right_sole_vel(6);
    left_sole_vel.setZero(); right_sole_vel.setZero();
    left_sole_vel.head(3) << vel_B.at(0);
    right_sole_vel.head(3) << vel_B.at(1);

    // Retrieve the current robot state (spatial position)
    // XYZRPY of the base link do not matter
    Eigen::VectorXd q = Eigen::VectorXd::Zero(7 + pos_j.GetNumJoints());
    for (size_t i = 0; i < pos_j.GetNumJoints(); ++i)
      q(7+i) = pos_j.GetJoint(i);

    // Perform the necessary precalculations for the Jacobians
    pinocchio::computeJointJacobians(*talos_model_, *talos_data_, q);
    pinocchio::framesForwardKinematics(*talos_model_, *talos_data_, q);

    // Compute contact Jacobians
    Eigen::MatrixXd J_left_sole = GetFrameJacobian(left_sole_id);
    Eigen::MatrixXd J_right_sole = GetFrameJacobian(right_sole_id);

    // Slice to obtain the Jacobian for each kinematic cheain (each leg)
    Eigen::MatrixXd J_left_slice = J_left_sole.block(0, 6, 6, 6);
    Eigen::MatrixXd J_right_slice = J_right_sole.block(0, 12, 6, 6);

    // Compute velocities: qd = Jinv * v
    std::vector<Eigen::VectorXd> joint_velocities;
    joint_velocities.emplace_back(J_left_slice.inverse() * left_sole_vel);
    joint_velocities.emplace_back(J_right_slice.inverse() * right_sole_vel);

    return Joints(joint_velocities);
  }

  Joints
  InverseKinematicsTalos::GetAllJointAccelerations(const EndeffectorsAcc &acc_B,
						   const Joints &pos_j,
						   const Joints &vel_j) const
  {
    // Retrieve the end effector accelerations (towr does not calculate orientations)
    Eigen::VectorXd left_sole_acc(6), right_sole_acc(6);
    left_sole_acc.setZero(); right_sole_acc.setZero();
    left_sole_acc.head(3) << acc_B.at(0);
    right_sole_acc.head(3) << acc_B.at(1);

    // Retrieve the current robot state (spatial position and velocity)
    // TODO: XYZRPY velocities matter!!!!!!
    Eigen::VectorXd q  = Eigen::VectorXd::Zero(7 + pos_j.GetNumJoints());
    Eigen::VectorXd qd = Eigen::VectorXd::Zero(6 + vel_j.GetNumJoints());
    for (size_t i = 0; i < pos_j.GetNumJoints(); ++i) {
      q(7+i)  = pos_j.GetJoint(i);
      qd(6+i) = vel_j.GetJoint(i);
    }

    // Perform the necessary precalculations
    pinocchio::computeJointJacobiansTimeVariation(*talos_model_, *talos_data_, q, qd);
    pinocchio::framesForwardKinematics(*talos_model_, *talos_data_, q);

    // Compute all the jacobians needed and slice
    Eigen::MatrixXd J_left = GetFrameJacobian(left_sole_id).block(0, 6, 6, 6);
    Eigen::MatrixXd J_right = GetFrameJacobian(right_sole_id).block(0, 12, 6, 6);
    Eigen::MatrixXd dJ_left = GetFrameJacobianTimeDerivative(left_sole_id).block(0, 6, 6, 6);
    Eigen::MatrixXd dJ_right = GetFrameJacobianTimeDerivative(right_sole_id).block(0, 12, 6, 6);

    // Compute the joint acceleration: qdd = Jinv * (a - Jd * qd)
    std::vector<Eigen::VectorXd> joint_accelerations;
    joint_accelerations.emplace_back(J_left.inverse() * (left_sole_acc - dJ_left * qd));
    joint_accelerations.emplace_back(J_right.inverse() * (right_sole_acc - dJ_right * qd));

    return Joints(joint_accelerations);
  }

  Eigen::MatrixXd
  InverseKinematicsTalos::GetFrameJacobian(int id) const
  {
    if (not talos_model_ or not talos_data_) return Eigen::MatrixXd::Zero(1,1);

    // Initialize the Jacobian matrix with 0s (needed)
    Eigen::MatrixXd J(6, talos_model_->nv); J.setZero();
    // Compute the jacobian
    pinocchio::getFrameJacobian(*talos_model_, *talos_data_, id,
				pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
				J);
    return J;
  }

  Eigen::MatrixXd
  InverseKinematicsTalos::GetFrameJacobianTimeDerivative(int id) const
  {
    if (not talos_model_ or not talos_data_) return Eigen::MatrixXd::Zero(1,1);

    // Initialize the Jacobian matrix with 0s (needed)
    Eigen::MatrixXd dJ(6, talos_model_->nv); dJ.setZero();
    // Compute the jacobian time derivative
    pinocchio::getFrameJacobianTimeVariation(*talos_model_, *talos_data_, id,
					     pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
					     dJ);
    return dJ;
  }

  Joints InverseKinematicsTalos::GetAllJointVelocities(const EndeffectorsVel &vel_B,
			       const Eigen::VectorXd &q) const
  {
    std::vector<Eigen::VectorXd> pos;
    pos.emplace_back(q.head<6>());
    pos.emplace_back(q.tail<6>());
    Joints joint_positions(pos);
    return GetAllJointVelocities(vel_B, joint_positions);
  }

  Joints InverseKinematicsTalos::GetAllJointAccelerations(const EndeffectorsAcc &acc_B,
				  const Eigen::VectorXd &q,
                                  const Eigen::VectorXd &qd) const
  {
    std::vector<Eigen::VectorXd> pos;
    pos.emplace_back(q.head<6>());
    pos.emplace_back(q.tail<6>());
    Joints joint_positions(pos);

    std::vector<Eigen::VectorXd> vel;
    vel.emplace_back(qd.head<6>());
    vel.emplace_back(qd.tail<6>());
    Joints joint_velocities(vel);

    return GetAllJointAccelerations(acc_B, joint_positions, joint_velocities);
  }

  Eigen::Vector3d
  InverseKinematicsTalos::GetCenterOfMassPosition(const Eigen::VectorXd& q) const
  {
    pinocchio::centerOfMass(*talos_model_, *talos_data_, q, false);
    return talos_data_->com[0];
  }

  Eigen::Vector3d
  InverseKinematicsTalos::GetCenterOfMassVelocity(const Eigen::VectorXd& q,
						  const Eigen::VectorXd& qd) const
  {
    pinocchio::centerOfMass(*talos_model_, *talos_data_, q, qd, false);
    return talos_data_->vcom[0];
  }

  void
  InverseKinematicsTalos::GetCenterOfMassPositionAndVelocity(const Eigen::VectorXd& q,
							     const Eigen::VectorXd& qd,
							     Eigen::Vector3d& com_pos,
							     Eigen::Vector3d& com_vel) const
  {
    pinocchio::centerOfMass(*talos_model_, *talos_data_, q, qd, false);
    com_pos = talos_data_->com[0];
    com_vel = talos_data_->vcom[0];
  }

  void
  InverseKinematicsTalos::GetCenterOfMassPositionVelocityAcceleration(const Eigen::VectorXd& q,
								      const Eigen::VectorXd& qd,
								      const Eigen::VectorXd& qdd,
								      Eigen::Vector3d& com_pos,
								      Eigen::Vector3d& com_vel,
								      Eigen::Vector3d& com_acc) const
  {
    pinocchio::centerOfMass(*talos_model_, *talos_data_, q, qd, qdd, false);
    com_pos = talos_data_->com[0];
    com_vel = talos_data_->vcom[0];
    com_acc = talos_data_->acom[0];
  }
} /* namespace xpp */
