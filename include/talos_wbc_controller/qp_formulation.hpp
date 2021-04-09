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

  enum class ConstraintName {
    EQUATION_OF_MOTION,
    FIXED_CONTACT_CONDITION,
    ACTUATION_LIMITS,
    CONTACT_STABILITY
  };
  
  // QP formulation cost weight
  typedef double Weight;
  // Constraint list
  typedef std::vector<ConstraintName> ConstraintList;
  // Robot state
  typedef std::vector<double> SpatialPos;
  typedef std::vector<double> SpatialVel;
  typedef std::vector<double> JointPos;
  typedef std::vector<double> JointVel;
  typedef std::vector<double> ComPos;
  typedef std::vector<double> ComVel;
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
  void SetRobotState(const SpatialPos&, const SpatialVel&,
		     const JointPos&, const JointVel&,
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
   * Sets the reference for the CoM
   */
  void SetDesiredCoM(const ComPos& com_pos, const ComVel& com_vel);

  /** 
   * Sets the Kp constant that multiplies the position error. Used to
   * calculate the desired acceleration.
   */
  void SetKP(double Kp);

  /**
   * Returns the current Kp constant.
   */
  double GetKP(void);

  /** 
   * Sets the Kp constant that multiplies the velocity error. Used to
   * calculate the desired acceleration.
   */
  void SetKV(double Kv);

  /**
   * Returns the current Kv constant.
   */
  double GetKV(void);

  /**
   * @brief Builds all the matrices for the QP program.
   * 
   * You need to call @ref SetRobotState, @ref SetPositionErrors,
   * @ref SetVelocityErrors and @ref SetReferenceAccelerations first.
   *
   */
  void BuildProblem(void);

  /**
   * @brief Sets the joint task weight.
   */
  void SetJointTaskWeight(double w);

  /**
   * @brief Sets the center of mass task weight.
   */
  void SetComTaskWeight(double w);

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

  /** 
   * @brief Removes all active contraints.
   */
  void ClearConstraints(void);

  /**
   * @brief Inserts a constraint
   */
  void PushConstraint(const ConstraintName constraint);

  /**
   * @brief Returns the number of variables for the current formulation.
   */
  int GetNumVariables(void) const;

  /**
   * @brief Returns the number of constraints rows for the current formulation.
   */
  int GetNumConstraints(void) const;

private:

  // Dynamics algorithms
  /** 
   * @brief Computes the term dJ * qd for the current robot configuration
   */
  Eigen::MatrixXd ComputedJqd(void) const;

  /**
   * @brief Computes the center of mass of the robot with respect to WORLD.
   *
   * You need to call @ref SetRobotState first.
   */
  Eigen::VectorXd ComputeCoM(void) const;

  /**
   * @brief Compute and returns the center of mass jacobian with respect to WORLD.
   *
   * You need to call @ref SetRobotState first.
   */
  Eigen::MatrixXd ComputeCoMJacobian(void) const;

  /**
   * @brief Compute and returns the center of mass time jacobian with respect to WORLD.
   *
   * You need to call @ref SetRobotState and @ref ComputeCoM first, respectively.
   */
  Eigen::MatrixXd ComputeCoMJacobianTimeVariation(void) const;

  // QP Formulation
  /** 
   * @brief Initializes the solver parameters
   */
  void SetSolverParameters(void);

  /**
   * @brief Updates the @ref P_ matrix according to the robot state
   */
  void UpdateHessianMatrix(void);
  /**
   *  Update the @ref g_ matrix according to the robot state.
   *  You must call @ref SetPositionErrors, @ref SetVelocityErrors and 
   *  @ref SetReferenceAccelerations first.
   */
  void UpdateGradientMatrix(void);
  /**
   *  @brief Update the @ref l_ and @ref u_ matrices according to the robot state.
   */
  void UpdateBounds(void);
  /**
   *  @brief Update the @ref A_ matrix according to the robot state.
   */
  void UpdateLinearConstraints(void);

  /**
   * @brief Returns the number of rows for each constraint.
   * 
   * You must call @ref SetRobotState, since we need to know how many
   * contacts the robot is currently making.
   *
   */
  int GetNumConstraintRows(const ConstraintName constraint) const;

  /// QP Solver instance
  OsqpEigen::Solver solver_;
  /// Used to decide if warm start the problem with the previous solution
  bool bWarmStart_;
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

  // List of currently active constraints
  ConstraintList active_constraints_;

  // Joint state task
  struct {
    Weight joint;
    Weight com;
  } task_weight_;
  PosErrors ep_;
  VelErrors ev_;
  AccVector qrdd_;
  double Kp_, Kv_;

  // The current contact jacobians
  ContactJacobians contact_jacobians_;
  ContactNames contact_names_;
  std::vector<int> contact_frames_ids_;
  double mu_; /// Friction coeficient

  // Selection matrix;
  Eigen::MatrixXd S_;

  // Robot internal representation
  ModelPtr model_;
  DataPtr data_;

  // Joint states
  Eigen::VectorXd q_, qd_;
  // Desired CoM pos and vel
  Eigen::Vector3d des_com_pos_, des_com_vel_;

  // Joint actuator limits
  Eigen::VectorXd u_max_;

  // Saves the last solution
  Eigen::VectorXd solution_;

  // Saves the number of constraints in the most recent solved problem
  int last_num_constraints_;
};

} // namespace talos_wbc_controller

#endif /* QP_FORMULATION_H */
