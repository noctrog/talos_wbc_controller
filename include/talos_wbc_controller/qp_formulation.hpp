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
  void SetRobotState(const JointPos&, const JointVel&, const ContactNames);

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

  /**
   * @brief Builds all the matrices for the QP program.
   * 
   * You need to call @ref SetRobotState, @ref SetPositionErrors,
   * @ref SetVelocityErrors and @ref SetReferenceAccelerations first.
   *
   */
  void BuildProblem(void);

  /**
   * @brief Solves the QP problem. 
   *
   * You need to call @ref BuildProblem before.
   *
   */
  void SolveProblem(void);

  /**
   * @brief Resets the warm start
   */
  void ResetWarmStart(void);

  /**
   * @brief Returns the last solution. You need to call @ref SolveProblem first.
   */
  Eigen::VectorXd GetSolution(void);

private:

  /** 
   * @brief Initializes the solver parameters
   */
  void SetSolverParameters(void);

  // QP Formulation
  /**
   * Updates the @ref P_ matrix according to the robot state
   */
  void UpdateHessianMatrix(void);
  /**
   *  Update the @ref g_ matrix according to the robot state.
   *  You must call @ref SetPositionErrors, @ref SetVelocityErrors and 
   *  @ref SetReferenceAccelerations first.
   */
  void UpdateGradientMatrix(void);
  /**
   *  Update the @ref l_ and @ref u_ matrices according to the robot state.
   */
  void UpdateBounds(void);
  /**
   *  Update the @ref A_ matrix according to the robot state.
   */
  void UpdateLinearConstraints(void);

  /// QP Solver instance
  OsqpEigen::Solver solver_;
  /// Used to decide if warm start the problem with the previous solution
  bool bWarmStart;
  /// QP Hessian matrix
  Eigen::SparseMatrix<double> P_;
  /// Gradient matrix
  Eigen::VectorXd g_;
  /// Linear constraint matrix
  Eigen::SparseMatrix<double> A_;
  /// Lower bound matrix
  Eigen::VectorXd l_;
  /// Upper bound matrix
  Eigen::VectorXd u_;

  // Joint state task
  Weight joint_task_weight_;
  PosErrors ep_;
  VelErrors ev_;
  AccVector qrdd_;
  double Kp_, Kv_;

  // The current contact jacobians
  ContactJacobians contact_jacobians_;
  ContactJacobians contact_jacobians_derivatives_;
  double mu_; /// Friction coeficient

  // Selection matrix;
  Eigen::MatrixXd S_;

  // Robot internal representation
  ModelPtr model_;
  DataPtr data_;

  // Joint states
  Eigen::VectorXd q_, qd_;

  // Joint actuator limits
  Eigen::VectorXd u_max_;

  // Saves the last solution
  Eigen::VectorXd solution_;
};

}

#endif /* QP_FORMULATION_H */
