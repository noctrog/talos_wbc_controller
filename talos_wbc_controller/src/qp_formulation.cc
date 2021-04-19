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
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/center-of-mass-derivatives.hpp>
#include <pinocchio/algorithm/centroidal.hpp>

namespace talos_wbc_controller {

  QpFormulation::QpFormulation(const std::string& urdf_path)
    : task_weight_{0.4, 0.6}, joint_task_dynamics_{16000.0, 252.0},
      com_task_dynamics_{10000.0, 200.0},
      mu_(0.4), bWarmStart_(false), active_constraints_{}, last_num_constraints_(0),
      des_com_pos_{0.0, 0.0, 1.0}, des_com_vel_{0.0, 0.0, 0.0},
      contact_families_{}
{
  // Create model and data objects
  model_ = std::make_shared<Model>();

  // Load robot model
  // JointModelFreeFlyer indicates that the root of the robot is not fixed to
  // the world
  pinocchio::urdf::buildModel(urdf_path, pinocchio::JointModelFreeFlyer(),
                              *model_);
  std::cout << "Pinocchio model loaded success, robot name: " << model_->name.c_str() << '\n';

  // Initialize pinocchio model data
  data_ = std::make_shared<pinocchio::Data>(*model_);

  const int njoints = model_->njoints - 2;
  // Compute the selection matrix, which remains always constant
  S_ = Eigen::MatrixXd(njoints, model_->nv);
  S_ << Eigen::MatrixXd::Zero(njoints, 6),
      Eigen::MatrixXd::Identity(njoints, njoints);

  // TODO: Actuation limits
  u_max_ = Eigen::VectorXd::Constant(njoints, 1000.0);

  q_ = Eigen::VectorXd::Constant(njoints + 7, 0.0);
  qd_ = Eigen::VectorXd::Constant(njoints + 6, 0.0);

  // Initialize the QP solver
  SetSolverParameters();
}

  void
  QpFormulation::SetRobotState(const SpatialPos& base_pos, const SpatialVel& base_vel,
			       const JointPos& q, const JointVel& qd,
			       const ContactNameList contact_names)
  {
    if (base_pos.size() != 7 or base_vel.size() != 6) {
      std::cerr << "SetRobotState: size of base_link position or velocity is wrong! Must be 7 and 6 respectively";
      return;
    }
    if (q.size() != (model_->njoints - 2) or qd.size() != (model_->njoints - 2)) {
      std::cerr << "SetRobotState: number of joints does not match with the robot model";
      return;
    }

    // Convert to Eigen without copying memory 
    q_  << Eigen::VectorXd::Map(base_pos.data(), base_pos.size()), Eigen::VectorXd::Map(q.data(), q.size());
    qd_ << Eigen::VectorXd::Map(base_vel.data(), base_vel.size()), Eigen::VectorXd::Map(qd.data(), qd.size());

    // Save the new contact names
    contact_names_ = contact_names;

    // Retrieve contact frame ids
    contact_frames_ids_.clear();
    for (const ContactName& name : contact_names_) {
      // If contact is the name of a family, include all of its contacts
      const auto cf = std::find_if(std::begin(contact_families_), std::end(contact_families_),
				   [name](const ContactFamily& cf){return cf.family_name.compare(name) == 0;});
      if (cf != std::end(contact_families_)) {
	for (const ContactName& contact_name : cf->contact_names) {
	  // There is no need to check because every contact in the family is checked in advance
	  contact_frames_ids_.push_back(model_->getFrameId(contact_name));
	}
      } else {
	// Add a normal contact
	const int id = model_->getFrameId(name);
	if (id < model_->frames.size())
	  contact_frames_ids_.push_back(id);
      }
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
				      pinocchio::ReferenceFrame::WORLD,
				      J);
      contact_jacobians_.push_back(J.block(0, 0, 3, model_->nv));   // Ignore the contact wrenches
    }

    // Center of mass computations
    // This computes de CoM position and velocity, as well as the term
    // dJ * dq, where dJ is the jacobian of the center of mass. dJ *
    // qd is accessible through data_->acom[0]. This is based in the
    // same principle used in the method QpFormulation::computedJqd
    pinocchio::centerOfMass(*model_, *data_, q_, qd_, 0*qd_);
    // Compute the CoM jacobian
    pinocchio::jacobianCenterOfMass(*model_, *data_, q_);
  }

  void
  QpFormulation::SetContactFamily(const ContactName& family_name,
				  const ContactNameList& contact_link_names)
  {
    // Check if every link name exists in the current model
    for (const auto& link_name : contact_link_names) {
      const int id = model_->getFrameId(link_name);
      // If frame does not exist, do not add the contact family
      if (id >= model_->frames.size()) {
	std::runtime_error("Frame link: " + link_name + " does not exist in the model!");
      }
    }

    contact_families_.emplace_back(family_name, contact_link_names);
  }

  void
  QpFormulation::ClearContactFamily(void)
  {
    contact_families_.clear();
  }

  void
  QpFormulation::SetDesiredCoM(const ComPos& com_pos, const ComVel& com_vel)
  {
    des_com_pos_ = Eigen::Vector3d::Map(com_pos.data(), com_pos.size());
    des_com_vel_ = Eigen::Vector3d::Map(com_vel.data(), com_vel.size());
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
  QpFormulation::SetJointTaskWeight(double w)
  {
    task_weight_.joint = w;
  }

  void
  QpFormulation::SetComTaskWeight(double w)
  {
    task_weight_.com = w;
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

    // Center of mass task
    Eigen::SparseMatrix<double> P_com_task(cols, cols);
    auto insert_in_sparse = [](Eigen::SparseMatrix<double>& S, const Eigen::MatrixXd &m, int i, int j) {
      for (size_t x = 0; x < m.rows(); ++x)
	for (size_t y = 0; y < m.cols(); ++y)
	  S.insert(i + x, j + y) = m(x, y);
    };
    const auto& Jcom = data_->Jcom;
    insert_in_sparse(P_com_task, Jcom.transpose() * Jcom, 0, 0);

    // Join all tasks
    P_ = P_joint_task * task_weight_.joint + P_com_task * task_weight_.com;
  }

  void
  QpFormulation::UpdateGradientMatrix(void)
  {
    // Joint task cost
    // Convert joint values to Eigen Vectors
    Eigen::VectorXd ep  = Eigen::VectorXd::Map(ep_.data(), ep_.size());
    Eigen::VectorXd ev  = Eigen::VectorXd::Map(ev_.data(), ev_.size());
    Eigen::VectorXd qrdd = Eigen::VectorXd::Map(qrdd_.data(), qrdd_.size());
    double Kp, Kv;

    // Get matrix dimensions
    const int cols = GetNumVariables();
    // Calculate joint gradient matrix
    Eigen::VectorXd q_joint(cols);
    Kp = joint_task_dynamics_.Kp;
    Kv = joint_task_dynamics_.Kv;
    q_joint << Eigen::VectorXd::Constant(6, 0.0), -(qrdd + Kp * ep + Kv * ev),
      Eigen::VectorXd::Constant(cols - model_->nv, 0.0);

    // Center of mass task
    Eigen::VectorXd q_com(cols);
    const Eigen::Vector3d& dJqd = data_->acom[0];
    const Eigen::Vector3d& ep_m = des_com_pos_ - data_->com[0];
    const Eigen::Vector3d& ev_m = des_com_vel_ - data_->vcom[0];
    Kp = com_task_dynamics_.Kp;
    Kv = com_task_dynamics_.Kv;
    const Eigen::VectorXd q_aux = -(dJqd + Kp * ep_m + Kv * ev_m).transpose() * data_->Jcom;
    q_com << q_aux, Eigen::VectorXd::Constant(cols - q_aux.size(), 0.0);

    // Join all tasks
    g_ = q_joint * task_weight_.joint + q_com * task_weight_.com;
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
    Eigen::MatrixXd J(3 * n_jac, model_->nv); J.setZero();
    for (size_t i = 0; i < n_jac; ++i) {
      J.block(i * 3, 0, 3, model_->nv) = contact_jacobians_[i];
    }

    // Initialize new sparse matrix to 0
    const int rows = GetNumConstraints();
    const int cols = GetNumVariables();
    A_.resize(rows, cols); A_.data().squeeze();
    // Reserve memory
    Eigen::VectorXi n_values_per_col(cols);
    n_values_per_col << Eigen::VectorXi::Constant(model_->nv, model_->nv + 3*n_jac),
      Eigen::VectorXi::Constant(3*n_jac, model_->nv + 5), // 5 not multplied by n_jac (only one contact per force)
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
			     current_row, model_->nv + 3*n_jac);
	current_row += model_->njoints - 2;
	break;
      case ConstraintName::CONTACT_STABILITY: {
	if (n_jac > 0) {
	  // For the moment ignore torques
	  Eigen::Vector3d ti(1.0, 0.0, 0.0), bi(0.0, 1.0, 0.0),
	      ni(0.0, 0.0, 1.0);
	  Eigen::MatrixXd friction =
	      Eigen::MatrixXd::Zero(5 * n_jac, 3 * n_jac);
	  for (size_t i = 0; i < n_jac; ++i) {
	    // Force pointing upwards (negative to keep all bounds equal)
	    friction.block<1, 3>(i * 5, i * 3) = -ni;
	    // Aproximate friction cone
	    friction.block<1, 3>(i * 5 + 1, i * 3) =  (ti - mu_ * ni);
	    friction.block<1, 3>(i * 5 + 2, i * 3) = -(ti + mu_ * ni);
	    friction.block<1, 3>(i * 5 + 3, i * 3) =  (bi - mu_ * ni);
            friction.block<1, 3>(i * 5 + 4, i * 3) = -(bi + mu_ * ni);
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

    auto current_num_constraints = GetNumConstraints();
    if (last_num_constraints_ != current_num_constraints) {
      bWarmStart_ = false;
      last_num_constraints_ = current_num_constraints;
    }
  }

  void
  QpFormulation::SolveProblem(void)
  {
    if (bWarmStart_) {
      // Update the problem
      if (not solver_.updateHessianMatrix(P_)) std::runtime_error("Could not update Hessian matrix!");
      if (not solver_.updateGradient(g_)) std::runtime_error("Could not update gradient matrix!");
      if (not solver_.updateLinearConstraintsMatrix(A_)) std::runtime_error("Could not update linear constraint matrix!");
      if (not solver_.updateBounds(l_, u_)) std::runtime_error("Could not update the bounds!");
    } else {
      std::cout << "Reseteando el solver!\n";

      // Reset the solver
      if (solver_.isInitialized()) {
	solver_.clearSolver();
	solver_.data()->clearHessianMatrix();
	solver_.data()->clearLinearConstraintsMatrix();
      }

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
      bWarmStart_ = true;
    }

    // Solve the QP problem
    if (not solver_.solve()) std::runtime_error("Solution not found!");

    // Retrieve the solution
    solution_ = solver_.getSolution();
  }

  void
  QpFormulation::SetSolverParameters(void)
  {
    solver_.settings()->setWarmStart(bWarmStart_);
    solver_.settings()->setAlpha(1.0);
    solver_.settings()->setVerbosity(false);
  }

  void
  QpFormulation::ResetWarmStart(void)
  {
    bWarmStart_ = false;
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
    return model_->nv + 3 * n_jac + (model_->njoints - 2);
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
      return 3 * contact_frames_ids_.size();
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
      dJqd = Eigen::VectorXd(n_jac * 3); // Reserve memory

      // Compute the needed forward kinematics for the current robot state
      pinocchio::forwardKinematics(*model_, *data_, q_, qd_, 0 * qd_);
      // Compute the frame acceleration constraint for every contact
      for (size_t i = 0; i < n_jac; ++i) {
	auto a = pinocchio::getFrameClassicalAcceleration(
	    *model_, *data_, contact_frames_ids_[i],
	    pinocchio::ReferenceFrame::WORLD);
	dJqd.segment(i * 3, 3) << a.linear();
      }

    } else {
      dJqd = Eigen::VectorXd::Constant(0, 0.0);
    }

    return dJqd;
  }

  Eigen::VectorXd
  QpFormulation::ComputeCoM(void) const
  {
    // Compute the CenterOfMass
    auto com = pinocchio::centerOfMass(*model_, *data_, q_, qd_);

    // Return the corresponding Eigen Vector
    Eigen::VectorXd com_v(3);
    com_v << com.x(), com.y(), com.z();
    return com_v;
  }

  Eigen::MatrixXd
  QpFormulation::ComputeCoMJacobian(void) const
  {
    pinocchio::getJacobianComFromCrba(*model_, *data_);
    return data_->Jcom;
  }

  Eigen::MatrixXd
  QpFormulation::ComputeCoMJacobianTimeVariation(void) const
  {
    // https://github.com/stack-of-tasks/pinocchio/issues/1297
    Eigen::MatrixXd dJ(3, model_->nv); dJ.setZero();
    auto dAg = pinocchio::computeCentroidalMapTimeVariation(*model_, *data_, q_, qd_);
    auto dJcom = dAg.block(0, 0, 3, model_->nv) / data_->mass[0];
    return dJcom;
  }


  Eigen::Vector3d
  QpFormulation::GetCenterOfMass(void) const
  {
    return data_->com[0];
  }

  Eigen::Vector3d
  QpFormulation::GetCenterOfMassVelocity(void) const
  {
    return data_->vcom[0];
  }

  void
  QpFormulation::SetJointTaskDynamics(const double kp, const double kv)
  {
    joint_task_dynamics_.Kp = kp;
    joint_task_dynamics_.Kv = kv;
  }

  void
  QpFormulation::SetComTaskDynamics(const double kp, const double kv)
  {
    com_task_dynamics_.Kp = kp;
    com_task_dynamics_.Kv = kv;
  }

} // namespace talos_wbc_controller