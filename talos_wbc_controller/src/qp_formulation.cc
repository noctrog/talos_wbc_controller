#include <types.h>
#include <functional>
#include <utility>

#include <talos_wbc_controller/qp_formulation.hpp>

#include <Eigen/Core>

#include <OsqpEigen/OsqpEigen.h>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/kinematics-derivatives.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/center-of-mass-derivatives.hpp>
#include <pinocchio/algorithm/centroidal.hpp>

namespace talos_wbc_controller {

  QpFormulation::QpFormulation(const std::string& urdf_path)
    : task_weight_{}, task_dynamics_{},
      mu_(0.4), bWarmStart_(false), last_num_constraints_(0),
      des_com_pos_{0.0, 0.0, 1.0}, des_com_vel_{0.0, 0.0, 0.0},
      contact_families_{}, active_tasks_{}, active_constraints_{}
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
  QpFormulation::SaveContacts(const ContactNameList& contact_names,
			      const ContactOrientationList& contact_orientations)
  {
    const int n_contacts = contact_names.size();
    ContactList new_contacts(n_contacts);
    for (int i = 0; i < n_contacts; ++i) {
      // Save contact name
      new_contacts[i].contact_name = contact_names[i];

      // Save contact frames
      // If contact is the name of a family, include all of its contacts
      const auto& name = new_contacts[i].contact_name;
      const auto cf = std::find_if(std::begin(contact_families_), std::end(contact_families_),
				   [name](const ContactFamily& cf){return cf.family_name.compare(name) == 0;});
      if (cf != std::end(contact_families_)) {
	for (const ContactName& contact_name : cf->contact_names) {
	  // There is no need to check because every contact in the family is checked in advance
	  new_contacts[i].contact_frames_ids.push_back(model_->getFrameId(contact_name));
	}
      } else {
	// Add a normal contact
	const int id = model_->getFrameId(name);
	if (id < model_->frames.size())
	  new_contacts[i].contact_frames_ids = { id };
      }

      // Save contact orientations
      if (not contact_orientations.empty()) {
	new_contacts[i].contact_orientation = contact_orientations[i];
      } else {
	// By default, the contact surface is a horizontal plane
	new_contacts[i].contact_orientation = Eigen::Matrix3d::Identity();
      }
    }
    contacts_ = std::move(new_contacts);
  }

  void
  QpFormulation::SetRobotState(const SpatialPos& base_pos, const SpatialVel& base_vel,
			       const JointPos& q, const JointVel& qd,
			       const ContactNameList& contact_names,
			       const ContactOrientationList& contact_orientations)
  {
    if (base_pos.size() != 7 or base_vel.size() != 6) {
      std::cerr << "SetRobotState: size of base_link position or velocity is wrong! Must be 7 and 6 respectively";
      return;
    }
    if (q.size() != (model_->njoints - 2) or qd.size() != (model_->njoints - 2)) {
      std::cerr << "SetRobotState: number of joints does not match with the robot model";
      return;
    }
    if (not contact_orientations.empty() && contact_orientations.size() != contact_names.size()) {
      std::cerr << "SetRobotState: number of contact names and orientations do not match";
      return;
    }

    // Convert to Eigen without copying memory 
    q_  << Eigen::VectorXd::Map(base_pos.data(), base_pos.size()), Eigen::VectorXd::Map(q.data(), q.size());
    qd_ << Eigen::VectorXd::Map(base_vel.data(), base_vel.size()), Eigen::VectorXd::Map(qd.data(), qd.size());

    // Save the new contacts
    SaveContacts(contact_names, contact_orientations);

    // Computes the joint space inertia matrix (M)
    pinocchio::crba(*model_, *data_, q_);  // This only computes the upper triangular part
    data_->M.triangularView<Eigen::StrictlyLower>() = data_->M.transpose().triangularView<Eigen::StrictlyLower>();
    // Compute nonlinear effects
    pinocchio::nonLinearEffects(*model_, *data_, q_, qd_);

    // Compute contact jacobians
    contact_jacobians_.clear();
    for (const auto& contact : contacts_) {
      for (const auto& id : contact.contact_frames_ids) {
	Eigen::MatrixXd J(6, model_->nv); J.setZero();
	pinocchio::computeFrameJacobian(*model_, *data_, q_, id,
					pinocchio::ReferenceFrame::WORLD,
					J);
	contact_jacobians_.push_back(J.block(0, 0, 3, model_->nv));   // Ignore the contact wrenches
      }
    }

    // Center of mass computations
    // This computes de CoM position and velocity, as well as the term
    // dJ * dq, where dJ is the jacobian of the center of mass. dJ *
    // qd is accessible through data_->acom[0]. This is based in the
    // same principle used in the method QpFormulation::computedJqd
    pinocchio::centerOfMass(*model_, *data_, q_, qd_, 0*qd_);
    // Compute the CoM jacobian expressed in WORLD frame
    pinocchio::jacobianCenterOfMass(*model_, *data_, q_);

    // Compute the jacobian of the base_link frame.
    // This jacobian is used for the orientation task.
    // const int base_frame_id = 2; // TODO: is this always true for freeflyers?
    const int base_frame_id = model_->getFrameId("base_link");
    base_link_jacobian_ = Eigen::MatrixXd::Zero(6, model_->nv);
    pinocchio::computeJointJacobians(*model_, *data_, q_);
    pinocchio::framesForwardKinematics(*model_, *data_, q_);
    pinocchio::computeFrameJacobian(*model_, *data_, q_, base_frame_id,
				    pinocchio::ReferenceFrame::WORLD,
				    base_link_jacobian_);
    pinocchio::forwardKinematics(*model_, *data_, q_, qd_, 0.0*qd_);
    const auto atpl = pinocchio::getFrameClassicalAcceleration(*model_, *data_, base_frame_id,
							       pinocchio::ReferenceFrame::WORLD);
    base_link_wdot_ << atpl.angular(); // TODO: always returns 0
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
  QpFormulation::SetDesiredCoM(const ComPos& com_pos,
			       const ComVel& com_vel,
			       const ComAcc& com_acc)
  {
    // Position
    des_com_pos_ = Eigen::Vector3d::Map(com_pos.data(), com_pos.size());

    // Velocity
    if (com_vel.size() > 0) {
      des_com_vel_ = Eigen::Vector3d::Map(com_vel.data(), com_vel.size());
      b_com_vel_specified_ = true;
    }
    else {
      des_com_vel_ = Eigen::Vector3d::Zero();
      b_com_vel_specified_ = false;
    }

    // Acceleration
    if (com_acc.size() > 0) {
      des_com_acc_ = Eigen::Vector3d::Map(com_acc.data(), com_acc.size());
      b_com_acc_specified_ = true;
    }
    else {
      des_com_vel_ = Eigen::Vector3d::Zero();
      b_com_acc_specified_ = false;
    }
  }

  void
  QpFormulation::SetDesiredBaseOrientation(const BaseRot& base_rot, const BaseAngVel& base_ang_vel)
  {
    des_base_rot_ = Eigen::Vector3d::Map(base_rot.data(), base_rot.size());
    des_base_ang_vel_ = Eigen::Vector3d::Map(base_ang_vel.data(), base_ang_vel.size());
  }

  void
  QpFormulation::SetDesiredFrameOrientation(const std::string& frame_name, const FrameRot& frame_rot,
					    const FrameAngVel& frame_ang_vel)
  {
    // Retrieve the frame id
    const int frame_id = model_->getFrameId(frame_name);
    // Only insert if the frame and vectors are valid
    if (frame_id < model_->frames.size() && frame_rot.size() == 3 && frame_ang_vel.size() == 3) {
      // Eigen::Vector3d des_frame_rot = Eigen::Vector3d::Map(frame_rot.data(), frame_rot.size());
      // Eigen::Vector3d des_frame_vel = Eigen::Vector3d::Map(frame_ang_vel.data(), frame_ang_vel.size());
      // Perform a copy since this is persistant
      Eigen::Vector3d des_frame_rot, des_frame_vel;
      des_frame_rot.x() = frame_rot[0];
      des_frame_rot.y() = frame_rot[1];
      des_frame_rot.z() = frame_rot[2];
      des_frame_vel.x() = frame_ang_vel[0];
      des_frame_vel.y() = frame_ang_vel[1];
      des_frame_vel.z() = frame_ang_vel[2];

      desired_orientations_.insert(std::make_pair(frame_id, OrientationState{.orientation = des_frame_rot,
									     .angular_vel = des_frame_vel}));
    }
  }

  void
  QpFormulation::EraseDesiredFrameOrientation(const std::string& frame_name)
  {
    // Retrieve the frame id
    const int frame_id = model_->getFrameId(frame_name);
    // Only erase if the frame is valid
    if (frame_id < model_->frames.size()) {
      desired_orientations_.erase(frame_id);
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
  QpFormulation::SetTaskWeight(const TaskName task, const Weight w)
  {
    if (TaskName::TOTAL_TASKS != task)
      task_weight_[static_cast<size_t>(task)] = w;
  }

  QpFormulation::Weight
  QpFormulation::GetTaskWeight(const TaskName task)
  {
    if (TaskName::TOTAL_TASKS != task)
      return task_weight_[static_cast<size_t>(task)];
    else
      return 0.0;
  }

  void
  QpFormulation::UpdateHessianMatrix(void)
  {
    // Get matrix dimensions
    const int cols = GetNumVariables();
    // Reset sparse matrix
    P_ = Eigen::SparseMatrix<double>(cols, cols);
    
    auto insert_in_sparse = [](Eigen::SparseMatrix<double>& S, const Eigen::MatrixXd &m, int i, int j) {
      for (size_t x = 0; x < m.rows(); ++x)
	for (size_t y = 0; y < m.cols(); ++y)
	  S.insert(i + x, j + y) = m(x, y);
    };

    for (const auto task : active_tasks_) {
      switch (task) {
      case TaskName::FOLLOW_JOINT: {
	// The joint task cost is proportional to the identity matrix
	typedef Eigen::Triplet<double> T;
	std::vector<T> triplet_v;
	triplet_v.reserve(model_->njoints - 2);
	for (size_t i = 0; i < model_->njoints - 2; ++i)
	  triplet_v.emplace_back(i+6, i+6, 1.0);
	Eigen::SparseMatrix<double> P_joint_task(cols, cols);
	P_joint_task.setFromTriplets(triplet_v.begin(), triplet_v.end());
	P_ += P_joint_task * GetTaskWeight(task);
	break;
      }
      case TaskName::FOLLOW_COM: {
	Eigen::SparseMatrix<double> P_com_task(cols, cols);
	const auto& Jcom = data_->Jcom;
	insert_in_sparse(P_com_task, Jcom.transpose() * Jcom, 0, 0);
	P_ += P_com_task * GetTaskWeight(task);
	break;
      }
      case TaskName::FOLLOW_BASE_ORIENTATION: {
	Eigen::SparseMatrix<double> P_orientation_task(cols, cols);
	const auto J = base_link_jacobian_.block(3, 0, 3, model_->nv);
	insert_in_sparse(P_orientation_task, J.transpose() * J, 0, 0);
	P_ += P_orientation_task * GetTaskWeight(task);
	break;
      }
      case TaskName::FOLLOW_ORIENTATION: {
	Eigen::SparseMatrix<double> P_orientation_task(cols, cols);
	Eigen::MatrixXd JtJ_sum(model_->nv, model_->nv);
	JtJ_sum.setZero();
	for (const auto& p : desired_orientations_) {
	  // Compute frame jacobian
	  const int id = p.first;
	  Eigen::MatrixXd J(6, model_->nv); J.setZero();
	  pinocchio::computeFrameJacobian(*model_, *data_, q_, id,
					  pinocchio::ReferenceFrame::WORLD,
					  J);
	  std::cout << "J:\n" << J << std::endl;
	  JtJ_sum += J.transpose() * J;
	}

	std::cout << "JtJ_sum: \n" << JtJ_sum << std::endl;
	insert_in_sparse(P_orientation_task, JtJ_sum, 0, 0);
	P_ += P_orientation_task * GetTaskWeight(task);
      }
      default:
	std::runtime_error("Task hessian matrix not implemented!");
      }
    }
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

    // Reset gradient matrix
    g_ = Eigen::VectorXd::Zero(cols);

    for (const auto task : active_tasks_) {
      switch (task) {
      case TaskName::FOLLOW_JOINT: {
	Eigen::VectorXd q_joint(cols);
	GetTaskDynamics(task, Kp, Kv);
	q_joint << Eigen::VectorXd::Constant(6, 0.0), -(qrdd + Kp * ep + Kv * ev),
	  Eigen::VectorXd::Constant(cols - model_->nv, 0.0);
	g_ += q_joint * GetTaskWeight(task);
	break;
      }
      case TaskName::FOLLOW_COM: {
	Eigen::VectorXd q_com(cols);
	const Eigen::Vector3d dJqd = b_com_acc_specified_ * data_->acom[0];
	const Eigen::Vector3d ep_m = des_com_pos_ - data_->com[0];
	const Eigen::Vector3d ev_m = b_com_vel_specified_ * (des_com_vel_ - data_->vcom[0]);
	const Eigen::Vector3d des_acc = b_com_acc_specified_ * des_com_acc_;
	GetTaskDynamics(task, Kp, Kv);
	const Eigen::VectorXd q_aux = -(des_acc - dJqd + Kp * ep_m + Kv * ev_m).transpose() * data_->Jcom;
	q_com << q_aux, Eigen::VectorXd::Constant(cols - q_aux.size(), 0.0);
	g_ += q_com * GetTaskWeight(task);
	break;
      }
      case TaskName::FOLLOW_BASE_ORIENTATION: {
	Eigen::VectorXd q_base(cols);
	const auto J = base_link_jacobian_.block(3, 0, 3, model_->nv);
	// Calculate RPY of the current base orientation
	const Eigen::Quaterniond q(Eigen::Vector4d(q_.segment(3, 4)));
	const Eigen::Matrix3d R = q.toRotationMatrix();
	const Eigen::Matrix3d dR = Eigen::AngleAxisd(des_base_rot_(0), Eigen::Vector3d::UnitX()) *
	  Eigen::AngleAxisd(des_base_rot_(1), Eigen::Vector3d::UnitY()) *
	  Eigen::AngleAxisd(des_base_rot_(2), Eigen::Vector3d::UnitZ()) * R.transpose();
	// Retrieve the term dJ * qd
	const Eigen::Vector3d& dJqd = base_link_wdot_;
	// Compute the errors
	const Eigen::Vector3d ep = 0.5 * Eigen::Vector3d {dR(0, 1) - dR(1, 2),
							  dR(0, 2) - dR(2, 0),
							  dR(1, 0) - dR(2, 1)};
	const Eigen::Vector3d ev = des_base_ang_vel_ - qd_.segment(3, 3);
	GetTaskDynamics(task, Kp, Kv);
	const Eigen::VectorXd q_aux = -(/*-dJqd*/ + Kp * ep + Kv * ev).transpose() * J;
	q_base << q_aux, Eigen::VectorXd::Constant(cols - q_aux.size(), 0.0);
	g_ += q_base * GetTaskWeight(task);
	break;
      }
      case TaskName::FOLLOW_ORIENTATION: {
	Eigen::SparseMatrix<double> P_orientation_task(cols, cols);
	Eigen::VectorXd grad_sum = Eigen::VectorXd::Constant(model_->nv, 0.0);
	Eigen::VectorXd q_base(cols);
	for (const auto& p : desired_orientations_) {
	  // Compute frame jacobian
	  const int id = p.first;
	  Eigen::MatrixXd J(6, model_->nv); J.setZero();
	  pinocchio::computeFrameJacobian(*model_, *data_, q_, id,
					  pinocchio::ReferenceFrame::WORLD,
					  J);
	  J = J.block(3, 0, 3, model_->nv);

	  // Compute the current gradient vector
	  // TODO this is ugly
	  const auto& desired_rot = p.second.orientation;
	  const auto& desired_angvel = p.second.angular_vel;
	  Eigen::Matrix3d R = data_->oMf[id].rotation();
	  const Eigen::Matrix3d dR = Eigen::AngleAxisd(desired_rot(0), Eigen::Vector3d::UnitX()) *
	    Eigen::AngleAxisd(desired_rot(1), Eigen::Vector3d::UnitY()) *
	    Eigen::AngleAxisd(desired_rot(2), Eigen::Vector3d::UnitZ()) * R.transpose();
	  const Eigen::Vector3d ep = 0.5 * Eigen::Vector3d {dR(0, 1) - dR(1, 2),
							    dR(0, 2) - dR(2, 0),
							    dR(1, 0) - dR(2, 1)};
	  const auto veltlp = pinocchio::getFrameVelocity(*model_, *data_,
							  id, pinocchio::ReferenceFrame::WORLD);
	  const Eigen::Vector3d current_angvel(veltlp.angular().x(),
					       veltlp.angular().y(),
					       veltlp.angular().z());
	  const Eigen::Vector3d ev = desired_angvel - current_angvel;
	  GetTaskDynamics(task, Kp, Kv);
	  const Eigen::VectorXd q_aux = -(/*-dJqd*/ + Kp * ep + Kv * ep).transpose() * J;

	  grad_sum += q_aux;
	}

	Eigen::VectorXd qbase(cols);
	qbase << grad_sum, Eigen::VectorXd::Constant(cols - grad_sum.size(), 0.0);
	
	g_ += qbase * GetTaskWeight(task);
      }
      default:
	std::runtime_error("Task gradient matrix not implemented!");
      }
    }
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
        int n_jac = contact_jacobians_.size();
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
    size_t n_jac = contact_jacobians_.size();
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
	  Eigen::MatrixXd friction = Eigen::MatrixXd::Zero(5 * n_jac, 3 * n_jac);
	  int i = 0;
	  for (const auto& contact : contacts_) {
	    for (const auto& id : contact.contact_frames_ids) {
	      // For the moment ignore torques
	      const Eigen::Vector3d ti = contact.contact_orientation.col(0);
	      const Eigen::Vector3d bi = contact.contact_orientation.col(1);
	      const Eigen::Vector3d ni = contact.contact_orientation.col(2);
	      // Force pointing upwards (negative to keep all bounds equal)
	      friction.block<1, 3>(i * 5, i * 3) = -ni;
	      // Aproximate friction cone
	      friction.block<1, 3>(i * 5 + 1, i * 3) =  (ti - mu_ * ni);
	      friction.block<1, 3>(i * 5 + 2, i * 3) = -(ti + mu_ * ni);
	      friction.block<1, 3>(i * 5 + 3, i * 3) =  (bi - mu_ * ni);
	      friction.block<1, 3>(i * 5 + 4, i * 3) = -(bi + mu_ * ni);

	      ++i;
	    }
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
    const int n_jac = contact_jacobians_.size();
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
  QpFormulation::ClearTasks(void)
  {
    active_tasks_.clear();
  }

  void
  QpFormulation::ClearConstraints(void)
  {
    active_constraints_.clear();
  }

  void
  QpFormulation::PushTask(const TaskName task)
  {
    // Insert the task if it is not currently present
    if (std::find(std::begin(active_tasks_), std::end(active_tasks_), task)
	== active_tasks_.end()) {
      active_tasks_.push_back(task);
    }
  }

  void
  QpFormulation::PushTask(const TaskName task, const double Kp, const double Kv)
  {
    // Insert task
    PushTask(task);
    // Update dynamic parameters
    SetTaskDynamics(task, Kp, Kv);
  }

  void
  QpFormulation::PushConstraint(const ConstraintName constraint)
  {
    // Insert constraint if it is not currently present
    if (std::find(std::begin(active_constraints_), std::end(active_constraints_), constraint)
	== active_constraints_.end()) {
      active_constraints_.push_back(constraint);
    }
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
      return 3 * contact_jacobians_.size();
    case ConstraintName::ACTUATION_LIMITS:
      if (model_)
	return model_->njoints - 2;
      else
	return 0;
    case ConstraintName::CONTACT_STABILITY:
      return 5 * contact_jacobians_.size();
    default:
      std::runtime_error("Please define the number of rows of the constraint!");
      return 0;
    }
  }

  Eigen::MatrixXd
  QpFormulation::ComputedJqd(void) const
  {
    const int n_jac = contact_jacobians_.size();
    Eigen::VectorXd dJqd;
    if (n_jac > 0) {
      dJqd = Eigen::VectorXd(n_jac * 3); // Reserve memory

      // Compute the needed forward kinematics for the current robot state
      pinocchio::forwardKinematics(*model_, *data_, q_, qd_, 0 * qd_);
      // Compute the frame acceleration constraint for every contact
      size_t i = 0;
      for (const auto& contact : contacts_) {
	for (const auto& id : contact.contact_frames_ids) {
	  auto a = pinocchio::getFrameClassicalAcceleration(
		      *model_, *data_, id, pinocchio::ReferenceFrame::WORLD);
	  dJqd.segment(i * 3, 3) << a.linear();

	  ++i;
	}
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
  QpFormulation::SetTaskDynamics(const TaskName task, const double kp, const double kv)
  {
    if (TaskName::TOTAL_TASKS != task) {
      TaskDynamics& td = task_dynamics_[static_cast<size_t>(task)];
      td.Kp = kp;
      td.Kv = kv;
    }
  }

  void
  QpFormulation::GetTaskDynamics(const TaskName task, double& kp, double& kv)
  {
    if (TaskName::TOTAL_TASKS != task) {
      TaskDynamics& td = task_dynamics_[static_cast<size_t>(task)];
      kp = td.Kp;
      kv = td.Kv;
    } else {
      kp = 0.0;
      kv = 0.0;
    }
  }

  void
  QpFormulation::SetFrictionCoefficient(double mu)
  {
    mu_ = mu;
  }

} // namespace talos_wbc_controller
