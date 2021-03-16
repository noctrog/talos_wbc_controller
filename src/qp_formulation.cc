#include <types.h>
#include <talos_wbc_controller/qp_formulation.hpp>

#include <Eigen/Core>

#include <OsqpEigen/OsqpEigen.h>

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
    : joint_task_weight_(0.5), Kp_(1.0), Kv_(1.0), mu_(0.4), bWarmStart(false)
  {
    // Create model and data objects
    model_ = std::make_shared<Model>();

    // Load robot model
    ROS_INFO("Loading URDF model...");
    std::string xpp_talos_path = ros::package::getPath("talos_wbc_controller");
    if (xpp_talos_path.size() == 0) {
      std::runtime_error("Could not find the urdf model! Check if it is located in the urdf folder!");
    }
    std::string urdf_path = xpp_talos_path + "/urdf/talos_full_legs_v2.urdf";
    std::cout << urdf_path << std::endl;
    // JointModelFreeFlyer indicates that the root of the robot is not fixed to the world
    pinocchio::urdf::buildModel(urdf_path, pinocchio::JointModelFreeFlyer(), *model_);
    ROS_INFO("Pinocchio model loaded success, robot name: %s", model_->name.c_str());

    // Initialize pinocchio model data
    data_ = std::make_shared<pinocchio::Data>(*model_);

    // Compute the selection matrix, which remains always constant
    S_ = Eigen::MatrixXd((model_->njoints - 2), model_->nv);
    S_ << Eigen::MatrixXd::Zero((model_->njoints - 2), 6), Eigen::MatrixXd::Identity((model_->njoints - 2), (model_->njoints - 2));

    // TODO: Actuation limits
    u_max_ = Eigen::VectorXd::Constant((model_->njoints - 2), 100.0);

    q_  = Eigen::VectorXd::Constant((model_->njoints - 2) + 7, 0.0);
    qd_ = Eigen::VectorXd::Constant((model_->njoints - 2) + 6, 0.0);

    // Initialize the QP solver
    SetSolverParameters();
  }
  
  void
  QpFormulation::SetRobotState(const SpatialPos& base_pos, const SpatialVel& base_vel,
			       const JointPos& q, const JointVel& qd, const ContactNames contact_names)
  {
    if (base_pos.size() != 7 or base_vel.size() != 6) {
      ROS_ERROR("SetRobotState: size of base_link position or velocity is wrong! Must be 7 and 6 respectively");
      return;
    }
    if (q.size() != (model_->njoints - 2) or qd.size() != (model_->njoints - 2)) {
      ROS_ERROR("SetRobotState: number of joints does not match with the robot model");
      return;
    }

    // If the number of contacts has changed, the problem cannot be warm started
    // TODO: Warm start using the past information?
    if (contact_names.size() != contact_jacobians_.size()) {
      bWarmStart = false;
    }

    // Convert to Eigen without copying memory (TODO: Posicion y orientacion son relevantes!(por lo menos rotacion))
    q_  << Eigen::VectorXd::Map(base_pos.data(), base_pos.size()), Eigen::VectorXd::Map(q.data(), q.size());
    qd_ << Eigen::VectorXd::Map(base_vel.data(), base_vel.size()), Eigen::VectorXd::Map(qd.data(), qd.size());

    // Retrieve contact frame ids
    std::vector<int> contact_frames_ids;
    for (const auto& name : contact_names) {
      int id = model_->getFrameId(name);
      contact_frames_ids.push_back(id);
    }

    // Computes the joint space inertia matrix (M)
    pinocchio::crba(*model_, *data_, q_);  // This only computes the upper triangular part
    data_->M.triangularView<Eigen::StrictlyLower>() = data_->M.transpose().triangularView<Eigen::StrictlyLower>();
    // Compute nonlinear effects
    pinocchio::nonLinearEffects(*model_, *data_, q_, qd_);

    // // Compute contact jacobians
    contact_jacobians_.clear();
    contact_jacobians_derivatives_.clear();
    // Needed for the contact jacobian time variation
    if (contact_frames_ids.size()) pinocchio::computeJointJacobiansTimeVariation(*model_, *data_, q_, qd_);
    for (const auto id : contact_frames_ids) {
      // Compute the contact jacobian
      Eigen::MatrixXd J(6, model_->nv); J.setZero();
      pinocchio::computeFrameJacobian(*model_, *data_, q_, id,
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

  void
  QpFormulation::UpdateHessianMatrix(void)
  {
    // Joint task cost
    // The joint task cost is proportional to the identity matrix
    typedef Eigen::Triplet<double> T;
    std::vector<T> triplet_v;
    triplet_v.reserve(model_->njoints - 2);
    for (size_t i = 0; i < model_->njoints - 2; ++i)
      triplet_v.emplace_back(i+6, i+6, 1.0);

    // Get matrix dimensions
    const int n_jac = contact_jacobians_.size();
    const int cols = model_->nv + 6 * n_jac + (model_->njoints - 2);

    Eigen::SparseMatrix<double> P_joint_task(cols, cols);
    P_joint_task.setFromTriplets(triplet_v.begin(), triplet_v.end());

    // TODO: Center of mass task

    // Join all tasks
    P_ = P_joint_task * (joint_task_weight_ / 2.0);
  }

  void
  QpFormulation::UpdateGradientMatrix(void)
  {
    // Joint task cost
    // Convert joint values to Eigen Vectors
    Eigen::VectorXd ep  = Eigen::VectorXd::Map(ep_.data(), ep_.size());
    Eigen::VectorXd ev  = Eigen::VectorXd::Map(ev_.data(), ev_.size());
    Eigen::VectorXd qrdd = Eigen::VectorXd::Map(qrdd_.data(), qrdd_.size());

    // Get matrix dimensions
    const int n_jac = contact_jacobians_.size();
    const int cols = model_->nv + 6 * n_jac + (model_->njoints - 2);
    // Calculate joint gradient matrix
    Eigen::VectorXd q_joint(cols);
    q_joint << Eigen::VectorXd::Constant(6, 0.0), -(qrdd + Kp_ * ep + Kv_ * ev),
      Eigen::VectorXd::Constant(cols - 6 - ep.size(), 0.0);

    // TODO: Center of mass task

    // Join all tasks
    g_ = q_joint * joint_task_weight_;
  }

  void
  QpFormulation::UpdateBounds(void)
  {
    // Calculate the transpose stacked contact jacobian time derivative
    // TODO: jacobiana entera o jacobiana a cachos?
    size_t n_jac = contact_jacobians_derivatives_.size();
    Eigen::MatrixXd dJ(6 * n_jac, model_->nv); dJ.setZero();
    for (size_t i = 0; i < n_jac; ++i) {
      dJ.block(i * 6, 0, 6, model_->nv) = contact_jacobians_[i];
    }

    auto contact_constraint = -dJ * qd_;

    // Lower bound
    l_ = Eigen::VectorXd::Zero(model_->nv + 6 * n_jac + (model_->njoints - 2) + 5 * n_jac);
    l_ <<
      -data_->nle,                                             // Dynamics
      contact_constraint,                                      // Contacts
      -u_max_,                                                 // Torque limits
      -Eigen::VectorXd::Constant(5*n_jac, OsqpEigen::INFTY);   // Friction cone
    // Upper bound
    u_ = Eigen::VectorXd::Zero(model_->nv + 6 * n_jac + (model_->njoints - 2) + 5 * n_jac);
    u_ <<
      -data_->nle,                              // Dynamics
      contact_constraint,                       // Contacts
      u_max_,                                   // Torque limits
      Eigen::VectorXd::Constant(5*n_jac, 0.0);  // Friction cone
  }

  void
  QpFormulation::UpdateLinearConstraints(void)
  {
    // Calculate the stacked contact jacobian
    size_t n_jac = contact_jacobians_.size();
    Eigen::MatrixXd J(6 * n_jac, model_->nv); J.setZero();
    for (size_t i = 0; i < n_jac; ++i) {
      J.block(i * 6, 0, 6, model_->nv) = contact_jacobians_[i];
    }

    // Initialize new sparse matrix to 0
    const int rows = model_->nv + 6 * n_jac + (model_->njoints - 2) + 5*n_jac;
    const int cols = model_->nv + 6 * n_jac + (model_->njoints - 2);
    A_.resize(rows, cols); A_.data().squeeze();
    // Reserve memory
    Eigen::VectorXi n_values_per_col(cols);
    n_values_per_col << Eigen::VectorXi::Constant(model_->nv, model_->nv + 6*n_jac),
      Eigen::VectorXi::Constant(6*n_jac, model_->nv + 5), // 5 not multplied by n_jac (only one contact per force)
      Eigen::VectorXi::Constant((model_->njoints - 2), model_->nv + 1); // 1 for Identity matrix
    A_.reserve(n_values_per_col);

    // Dynamics: [M -Jt -St]
    Eigen::MatrixXd dynamics(model_->nv, model_->nv + 6*n_jac + (model_->njoints - 2));
    dynamics << data_->M, -J.transpose(), -S_.transpose();
    for (size_t i = 0; i < dynamics.rows(); ++i)
      for (size_t j = 0; j < dynamics.cols(); ++j)
	A_.insert(i, j) = dynamics(i, j);

    // Contacts: stacked jacobian matrix
    for (size_t i = 0; i < J.rows(); ++i)
      for (size_t j = 0; j < J.cols(); ++j)
	A_.insert(model_->nv + i, j) = J(i, j);

    // Actuation limits: Identity matrix
    for (size_t i = 0; i < (model_->njoints - 2); ++i)
      A_.insert(model_->nv + 6*n_jac + i, model_->nv + 6*n_jac + i) = 1.0;

    // Contact stability
    // For the moment ignore torques
    Eigen::Vector3d ti(1.0, 0.0, 0.0), bi(0.0, 1.0, 0.0), ni(0.0, 0.0, 1.0);
    Eigen::MatrixXd friction = Eigen::MatrixXd::Zero(5*n_jac , 6*n_jac);
    for (size_t i = 0; i < n_jac; ++i) {
      // Force pointing upwards (negative to keep all bounds equal)
      friction.block<1,3>(i*5, i*6) = -ni;
      // Aproximate friction cone
      friction.block<1, 3>(i*5+1, i*6) = (ti - mu_ * ni);
      friction.block<1, 3>(i*5+2, i*6) = (ti + mu_ * ni);
      friction.block<1, 3>(i*5+3, i*6) = (bi - mu_ * ni);
      friction.block<1, 3>(i*5+4, i*6) = (bi + mu_ * ni);
    }

    // Save friction matrix in sparse matrix
    for (size_t i = 0; i < friction.rows(); ++i)
      for (size_t j = 0; j < friction.cols(); ++j)
	A_.insert(model_->nv + 6*n_jac + (model_->njoints - 2) + i,
		  model_->nv + j) = friction(i, j);
  }

  void
  QpFormulation::BuildProblem(void)
  {
    UpdateHessianMatrix();
    UpdateGradientMatrix();
    UpdateBounds();
    UpdateLinearConstraints();

    ROS_INFO("P shape: (%ld, %ld)", P_.rows(), P_.cols());
    ROS_INFO("g shape: (%ld)", g_.size());
    ROS_INFO("A shape: (%ld, %ld)", A_.rows(), A_.cols());
    ROS_INFO("l shape: (%ld)", l_.size());
    ROS_INFO("u shape: (%ld)", u_.size());
  }

  void
  QpFormulation::SolveProblem(void)
  {
    if (bWarmStart) {
      // Update the problem
      if (not solver_.updateHessianMatrix(P_)) std::runtime_error("Could not update Hessian matrix!");
      if (not solver_.updateGradient(g_)) std::runtime_error("Could not update gradient matrix!");
      if (not solver_.updateLinearConstraintsMatrix(A_)) std::runtime_error("Could not update linear constraint matrix!");
      if (not solver_.updateBounds(l_, u_)) std::runtime_error("Could not update the bounds!");
      ROS_INFO("Using warm start");
    } else {
      // Set the number of variables and constraints
      const int n_jac = contact_jacobians_.size();
      const int rows = model_->nv + 6 * n_jac + (model_->njoints - 2) + 5 * n_jac;
      const int cols = model_->nv + 6 * n_jac + (model_->njoints - 2);
      solver_.data()->setNumberOfVariables(cols);
      solver_.data()->setNumberOfConstraints(rows);

      // Create a new problem
      if (not solver_.data()->setHessianMatrix(P_)) std::runtime_error("Could not set Hessian matrix!");
      if (not solver_.data()->setGradient(g_)) std::runtime_error("Could not set gradient matrix!");
      if (not solver_.data()->setLinearConstraintsMatrix(A_)) std::runtime_error("Could not set linear constraint matrix!");
      if (not solver_.data()->setLowerBound(l_)) std::runtime_error("Could not set lower bound!");
      if (not solver_.data()->setUpperBound(u_)) std::runtime_error("Could not set upper bound!");

      // Init the solver
      if (not solver_.initSolver()) std::runtime_error("Fail initializing solver!");
      ROS_INFO("Solver initialized");
      // Set the next iteration to be warm started
      bWarmStart = true;
    }

    // Solve the QP problem
    ROS_INFO("Solver solve() is starting...");
    if (not solver_.solve()) std::runtime_error("Solution not found!");
    ROS_INFO("Solver solve() finished");

    // Retrieve the solution
    solution_ = solver_.getSolution();

    // TODO Delete
    ROS_INFO_STREAM("solution: " << solution_.transpose());
  }

  void
  QpFormulation::SetSolverParameters(void)
  {
    solver_.settings()->setWarmStart(bWarmStart);
    solver_.settings()->setAlpha(1.0);
  }

  void
  QpFormulation::ResetWarmStart(void)
  {
    bWarmStart = false;
  }

  Eigen::VectorXd
  QpFormulation::GetSolution(void)
  {
    return solution_;
  }
}
