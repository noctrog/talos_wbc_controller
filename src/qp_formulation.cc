#include <types.h>
#include <talos_wbc_controller/qp_formulation.hpp>

#include <Eigen/Core>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/kinematics-derivatives.hpp>

#include <ros/ros.h>
#include <ros/package.h>

namespace talos_wbc_controller {

  QpFormulation::QpFormulation()
    : joint_task_weight_(0.5), Kp_(1.0), Kv_(1.0)
  {
    // Create model and data objects
    model_ = std::make_shared<Model>();

    // Load robot model
    ROS_INFO("Loading URDF model...");
    std::string xpp_talos_path = ros::package::getPath("talos_wbc_controller");
    if (xpp_talos_path.size() == 0) {
      ROS_ERROR("You need to install the xpp_talos package: https://github.com/noctrog/talos_wbc_controller");
      exit(-1);
    }
    std::string urdf_path = xpp_talos_path + "/urdf/talos_full_legs_v2.urdf";
    std::cout << urdf_path << std::endl;
    // JointModelFreeFlyer indicates that the root of the robot is not fixed to the world
    pinocchio::urdf::buildModel(urdf_path, pinocchio::JointModelFreeFlyer(), *model_);
    ROS_INFO("Pinocchio model loaded success, robot name: %s", model_->name.c_str());

    // Initialize pinocchio model data
    data_ = std::make_shared<pinocchio::Data>(*model_);
  }
  
  void
  QpFormulation::SetRobotState(const JointPos& q_, const JointVel& qd_, const JointAcc& qdd_,
		     const ContactNames contact_names)
  {
    // Convert to Eigen without copying memory
    Eigen::VectorXd q   = Eigen::VectorXd::Map(q_.data(), q_.size());
    Eigen::VectorXd qd  = Eigen::VectorXd::Map(qd_.data(), q_.size());
    Eigen::VectorXd qdd = Eigen::VectorXd::Map(qdd_.data(), q_.size());

    // Retrieve contact frame ids
    std::vector<int> contact_frames_ids;
    for (const auto& name : contact_names) {
      int id = model_->getFrameId(name);
      contact_frames_ids.push_back(id);
    }

    // Computes the joint space inertia matrix (M)
    pinocchio::crba(*model_, *data_, q);  // This only computes the upper triangular part
    data_->M.triangularView<Eigen::StrictlyLower>() = data_->M.transpose().triangularView<Eigen::StrictlyLower>();
    // Compute nonlinear effects
    pinocchio::nonLinearEffects(*model_, *data_, q, qd);

    // Compute contact jacobians
    contact_jacobians_.clear();
    contact_jacobians_derivatives_.clear();
    // Needed for the contact jacobian time variation
    if (contact_frames_ids.size()) pinocchio::computeJointJacobiansTimeVariation(*model_, *data_, q, qd);
    for (const auto id : contact_frames_ids) {
      // Compute the contact jacobian
      Eigen::MatrixXd J(6, model_->nv); J.setZero();
      pinocchio::computeFrameJacobian(*model_, *data_, q, id,
				      pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
				      J);
      contact_jacobians_.push_back(J);
      // Compute the contact jacobian time variation
      J.setZero();
      pinocchio::getFrameJacobianTimeVariation(*model_, *data_, id,
					       pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
					       J);
      contact_jacobians_derivatives_.push_back(J);
    }
  }

  void
  QpFormulation::SetPositionErrors(const PosErrors& ep)
  {
    ep_ = ep;
  }

  void
  QpFormulation::SetVelocityErrors(const VelErrors& ev)
  {
    ev_ = ev;
  }

  void
  QpFormulation::SetReferenceAccelerations(const AccVector& qrdd)
  {
    qrdd_ = qrdd;
  }

  void
  QpFormulation::SetKP(double Kp)
  {
    Kp_ = Kp;
  }

  void
  QpFormulation::SetKV(double Kv)
  {
    Kv_ = Kv;
  }

  QpFormulation::AccVector
  QpFormulation::GetDesiredAccelerations(void)
  {
    if (ep_.empty() or ev_.empty() or qrdd_.empty() or
	not (ep_.size() == ev_.size()) or not (ep_.size() == qrdd_.size())) return {};

    AccVector acc_desired(ep_.size());
    for (size_t i = 0; i < acc_desired.size(); ++i) {
      acc_desired[i] = qrdd_[i] + Kv_ * ev_[i] + Kp_ * ep_[i];
    }

    return acc_desired;
  }

}
