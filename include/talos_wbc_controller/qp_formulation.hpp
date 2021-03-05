#ifndef QP_FORMULATION_H
#define QP_FORMULATION_H

#include <vector>
#include <memory>

#include <OsqpEigen/Solver.hpp>

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

namespace talos_wbc_controller {
  
class QpFormulation
{
public:

  // QP formulation cost weight
  typedef double Weight;
  // Robot state
  typedef std::vector<double> JointPos;
  typedef std::vector<double> JointVel;
  typedef std::vector<double> JointAcc;
  // Robot feedback
  typedef std::vector<double> PosErrors;
  typedef std::vector<double> VelErrors;
  typedef std::vector<double> AccVector;
  // Contact names and jacobians
  typedef std::vector<std::string> ContactNames;
  typedef std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>> ContactJacobians;
  // Robot representation
  typedef pinocchio::Model Model;
  typedef pinocchio::Data Data;
  typedef std::shared_ptr<Model> ModelPtr;
  typedef std::shared_ptr<Data> DataPtr;
  
  QpFormulation();
  virtual ~QpFormulation() = default;

  /**
   * Updates the robot state. It automatically computes the inertia,
   * nonlinear, stacked contact jacobian and stacked contact jacobian
   * time derivative.
   */
  void SetRobotState(const JointPos&, const JointVel&, const JointAcc&,
		     const ContactNames);

  // Joint state task
  /** 
   * Sets the current position error, used to calculate the desired acceleration.
   */
  void SetPositionErrors(const PosErrors&);

  /** 
   * Sets the current velocity error, used to calculate the desired acceleration.
   */
  void SetVelocityErrors(const VelErrors&);

  /** 
   * Sets the reference acceleration, used to calculate the desired acceleration.
   */
  void SetReferenceAccelerations(const AccVector&);

  /** 
   * Sets the Kp constant that multiplies the position error. Used to
   * calculate the desired acceleration.
   */
  void SetKP(double Kp);

  /** 
   * Sets the Kp constant that multiplies the velocity error. Used to
   * calculate the desired acceleration.
   */
  void SetKV(double Kv);

private:

  // QP Formulation
  /**
   * Updates the @ref P_ matrix according to the robot state
   */
  void UpdateHessianMatrix(void);
  /**
   *  Update the @ref q_ matrix according to the robot state.
   */
  bool updateGradientMatrix(void);
  /**
   *  Update the @ref l_ and @ref u_ matrices according to the robot state.
   */
  bool updateBounds(void);
  /**
   *  Update the @ref A_ matrix according to the robot state.
   */
  bool updateLinearConstraints(void);

  /// QP Solver instance
  OsqpEigen::Solver solver_;
  /// QP Hessian matrix
  Eigen::SparseMatrix<double> P_;
  /// Gradient matrix
  Eigen::VectorXd q_;
  /// Linear constraint matrix
  Eigen::SparseMatrix<double> A_;
  /// Lower bound matrix
  Eigen::VectorXd l_;
  /// Upper bound matrix
  Eigen::VectorXd u_;

  AccVector GetDesiredAccelerations(void);

  // Joint state task
  Weight joint_task_weight_;
  PosErrors ep_;
  VelErrors ev_;
  AccVector qrdd_;
  double Kp_, Kv_;

  // The current contact jacobians
  ContactJacobians contact_jacobians_;
  ContactJacobians contact_jacobians_derivatives_;

  // Robot internal representation
  ModelPtr model_;
  DataPtr data_;
};

}

#endif /* QP_FORMULATION_H */
