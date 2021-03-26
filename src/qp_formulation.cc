#include <types.h>
#include <functional>

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
    : joint_task_weight_(0.5), Kp_(100.0), Kv_(0.05), mu_(0.4),
      bWarmStart(false), active_constraints_{} {
  // Create model and data objects
  model_ = std::make_shared<Model>();

  // Load robot model
  ROS_INFO("Loading URDF model...");
  std::string xpp_talos_path = ros::package::getPath("talos_wbc_controller");
  if (xpp_talos_path.size() == 0) {
    std::runtime_error("Could not find the urdf model! Check if it is located "
                       "in the urdf folder!");
  }
  std::string urdf_path = xpp_talos_path + "/urdf/talos_full_legs_v2.urdf";
  std::cout << urdf_path << std::endl;
  // JointModelFreeFlyer indicates that the root of the robot is not fixed to
  // the world
  pinocchio::urdf::buildModel(urdf_path, pinocchio::JointModelFreeFlyer(),
                              *model_);
  ROS_INFO("Pinocchio model loaded success, robot name: %s",
           model_->name.c_str());

  // Initialize pinocchio model data
  data_ = std::make_shared<pinocchio::Data>(*model_);

  // Compute the selection matrix, which remains always constant
  S_ = Eigen::MatrixXd((model_->njoints - 2), model_->nv);
  S_ << Eigen::MatrixXd::Zero((model_->njoints - 2), 6),
      Eigen::MatrixXd::Identity((model_->njoints - 2), (model_->njoints - 2));

  // TODO: Actuation limits
  u_max_ = Eigen::VectorXd::Constant((model_->njoints - 2), 1000.0);

  q_ = Eigen::VectorXd::Constant((model_->njoints - 2) + 7, 0.0);
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
    if (bWarmStart and A_.rows() != GetNumConstraints()) {
      bWarmStart = false;
    }

    // Convert to Eigen without copying memory (TODO: Posicion y orientacion son relevantes!(por lo menos rotacion))
    q_  << Eigen::VectorXd::Map(base_pos.data(), base_pos.size()), Eigen::VectorXd::Map(q.data(), q.size());
    qd_ << Eigen::VectorXd::Map(base_vel.data(), base_vel.size()), Eigen::VectorXd::Map(qd.data(), qd.size());

    // Save the new contact names
    contact_names_ = contact_names;

    // Retrieve contact frame ids
    contact_frames_ids_.clear();
    for (const auto& name : contact_names_) {
      int id = model_->getFrameId(name);
      if (id < model_->frames.size())
	contact_frames_ids_.push_back(id);
    }

    // Computes the joint space inertia matrix (M)
    pinocchio::crba(*model_, *data_, q_);  // This only computes the upper triangular part
    data_->M.triangularView<Eigen::StrictlyLower>() = data_->M.transpose().triangularView<Eigen::StrictlyLower>();
    // Compute nonlinear effects
    pinocchio::nonLinearEffects(*model_, *data_, q_, qd_);

    // Compute contact jacobians
    contact_jacobians_.clear();
    for (const auto id : contact_frames_ids_) {
      Eigen::MatrixXd J(6, model_->nv); J.setZero();
      pinocchio::computeFrameJacobian(*model_, *data_, q_, id,
				      pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
				      J);
      contact_jacobians_.push_back(J);
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

  double
  QpFormulation::GetKP(void)
  {
    return Kp_;
  }

  void
  QpFormulation::SetKV(double Kv)
  {
    Kv_ = Kv;
  }

  double
  QpFormulation::GetKV(void)
  {
    return Kv_;
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
    const int cols = GetNumVariables();

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
    const int cols = GetNumVariables();
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
    const int n_constraints = GetNumConstraints();
    int current_row = 0;

    // Lower bound
    l_ = Eigen::VectorXd::Zero(n_constraints);
    u_ = Eigen::VectorXd::Zero(n_constraints);
    for (const auto constraint : active_constraints_) {
      switch (constraint) {
      case ConstraintName::EQUATION_OF_MOTION:
	l_.segment(current_row, data_->nle.size()) = -data_->nle;
	u_.segment(current_row, data_->nle.size()) = -data_->nle;
	current_row += data_->nle.size();
	break;
      case ConstraintName::FIXED_CONTACT_CONDITION: {
	auto dJqd = ComputedJqd();
	l_.segment(current_row, dJqd.size()) = -dJqd;
	u_.segment(current_row, dJqd.size()) = -dJqd;
	current_row += dJqd.size();
	break;
      }
      case ConstraintName::ACTUATION_LIMITS:
	l_.segment(current_row, u_max_.size()) = -u_max_;
	u_.segment(current_row, u_max_.size()) =  u_max_;
	current_row += u_max_.size();
	break;
      case ConstraintName::CONTACT_STABILITY: {
        int n_jac = contact_frames_ids_.size();
        const auto lower = -Eigen::VectorXd::Constant(5 * n_jac, OsqpEigen::INFTY);
        const auto upper =  Eigen::VectorXd::Constant(5 * n_jac, 0.0);
        l_.segment(current_row, lower.size()) = lower;
        u_.segment(current_row, upper.size()) = upper;
        current_row += lower.size();
        break;
      }
      default:
        std::runtime_error("You have to define your constraint bounds here!");
      }
    }
  }

  void
  QpFormulation::UpdateLinearConstraints(void)
  {
    // Calculate the stacked contact jacobian
    size_t n_jac = contact_frames_ids_.size();
    Eigen::MatrixXd J(6 * n_jac, model_->nv); J.setZero();
    for (size_t i = 0; i < n_jac; ++i) {
      J.block(i * 6, 0, 6, model_->nv) = contact_jacobians_[i];
    }

    // Initialize new sparse matrix to 0
    const int rows = GetNumConstraints();
    const int cols = GetNumVariables();
    A_.resize(rows, cols); A_.data().squeeze();
    // Reserve memory
    Eigen::VectorXi n_values_per_col(cols);
    n_values_per_col << Eigen::VectorXi::Constant(model_->nv, model_->nv + 6*n_jac),
      Eigen::VectorXi::Constant(6*n_jac, model_->nv + 5), // 5 not multplied by n_jac (only one contact per force)
      Eigen::VectorXi::Constant((model_->njoints - 2), model_->nv + 1); // 1 for Identity matrix
    A_.reserve(n_values_per_col);

    // Function to insert dense martices in the sparse matrix
    auto insert_in_A = [this](const Eigen::MatrixXd &m, int i, int j) {
      for (size_t x = 0; x < m.rows(); ++x)
	for (size_t y = 0; y < m.cols(); ++y)
	  A_.insert(i + x, j + y) = m(x, y);
    };

    // Function to insert diagonal matrices in the sparse matrix
    auto diagonal_insert_in_A = [this](const Eigen::VectorXd &v, int i, int j) {
      for (size_t k = 0; k < v.size(); ++k)
	A_.insert(i + k, j + k) = v(k);
    };

    size_t current_row = 0;

    for (const auto constraint : active_constraints_) {
      switch (constraint) {
      case ConstraintName::EQUATION_OF_MOTION: {      // Dynamics: [M -Jt -St]
	Eigen::MatrixXd dynamics(model_->nv, cols);
	dynamics << data_->M, -J.transpose(), -S_.transpose();
	insert_in_A(dynamics, current_row, 0);
        current_row += dynamics.rows();
        break;
      }
      case ConstraintName::FIXED_CONTACT_CONDITION:
	insert_in_A(J, current_row, 0);
	current_row += J.rows();
	break;
      case ConstraintName::ACTUATION_LIMITS:
	diagonal_insert_in_A(Eigen::VectorXd::Constant(model_->njoints - 2, 1.0),
			     current_row, model_->nv + 6*n_jac);
	current_row += model_->njoints - 2;
	break;
      case ConstraintName::CONTACT_STABILITY: {
	if (n_jac > 0) {
	  // For the moment ignore torques
	  Eigen::Vector3d ti(1.0, 0.0, 0.0), bi(0.0, 1.0, 0.0),
	      ni(0.0, 0.0, 1.0);
	  Eigen::MatrixXd friction =
	      Eigen::MatrixXd::Zero(5 * n_jac, 6 * n_jac);
	  for (size_t i = 0; i < n_jac; ++i) {
	    // Force pointing upwards (negative to keep all bounds equal)
	    friction.block<1, 3>(i * 5, i * 6) = -ni;
	    // Aproximate friction cone
	    friction.block<1, 3>(i * 5 + 1, i * 6) = (ti - mu_ * ni);
	    friction.block<1, 3>(i * 5 + 2, i * 6) = (ti + mu_ * ni);
	    friction.block<1, 3>(i * 5 + 3, i * 6) = (bi - mu_ * ni);
            friction.block<1, 3>(i * 5 + 4, i * 6) = (bi + mu_ * ni);
          }

	  insert_in_A(friction, current_row, model_->nv);
	  current_row += friction.rows();
          break;
	}
        default:
          std::runtime_error("Implement your constraint here!");
        }
      }
    }
  }

  void
  QpFormulation::BuildProblem(void)
  {
    UpdateHessianMatrix();
    UpdateGradientMatrix();
    UpdateBounds();
    UpdateLinearConstraints();
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
    } else {
      // Set the number of variables and constraints
      const int rows = GetNumConstraints();
      const int cols = GetNumVariables();
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
      // Set the next iteration to be warm started
      bWarmStart = true;
    }

    // Solve the QP problem
    if (not solver_.solve()) std::runtime_error("Solution not found!");

    // Retrieve the solution
    solution_ = solver_.getSolution();
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

  int
  QpFormulation::GetNumVariables(void) const
  {
    const int n_jac = contact_frames_ids_.size();
    return model_->nv + 6 * n_jac + (model_->njoints - 2);
  }

  int
  QpFormulation::GetNumConstraints(void) const
  {
    return std::accumulate(
	active_constraints_.begin(), active_constraints_.end(), 0,
	[this](auto &a, auto &b) { return a + this->GetNumConstraintRows(b); });
  }

  void
  QpFormulation::ClearConstraints(void)
  {
    active_constraints_.clear();
  }

  void
  QpFormulation::PushConstraint(const ConstraintName constraint)
  {
    active_constraints_.push_back(constraint);
  }

  int
  QpFormulation::GetNumConstraintRows(const ConstraintName constraint) const
  {
    switch (constraint) {
    case ConstraintName::EQUATION_OF_MOTION:
      if (model_)
	return model_->nv;
      else
	return 0;
    case ConstraintName::FIXED_CONTACT_CONDITION:
      return 6 * contact_frames_ids_.size();
    case ConstraintName::ACTUATION_LIMITS:
      if (model_)
	return model_->njoints - 2;
      else
	return 0;
    case ConstraintName::CONTACT_STABILITY:
      return 5 * contact_frames_ids_.size();
    default:
      std::runtime_error("Please define the number of rows of the constraint!");
    }
  }

  Eigen::MatrixXd
  QpFormulation::ComputedJqd(void) const
  {
    const int n_jac = contact_frames_ids_.size();
    Eigen::VectorXd dJqd;
    if (n_jac > 0) {
      dJqd = Eigen::VectorXd(n_jac * 6); // Reserve memory

      // Compute the needed forward kinematics for the current robot state
      pinocchio::forwardKinematics(*model_, *data_, q_, qd_, 0 * qd_);
      // Compute the frame acceleration constraint for every contact
      for (size_t i = 0; i < n_jac; ++i) {
	auto a = pinocchio::getFrameClassicalAcceleration(
	    *model_, *data_, contact_frames_ids_[i],
	    pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);
	dJqd.segment(i * 6, 6) << a.linear(), a.angular();
      }

    } else {
      dJqd = Eigen::VectorXd::Constant(0, 0.0);
    }

    return dJqd;
  }

} // namespace talos_wbc_controller
