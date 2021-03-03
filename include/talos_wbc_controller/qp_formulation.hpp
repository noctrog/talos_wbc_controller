#ifndef QP_FORMULATION_H
#define QP_FORMULATION_H

#include <vector>
#include <OsqpEigen/Solver.hpp>

namespace talos_wbc_controller {
  
class QpFormulation
{
public:

  typedef double Weight;
  typedef std::vector<double> PosErrors;
  typedef std::vector<double> VelErrors;
  typedef std::vector<double> AccVector;
  
  QpFormulation();
  virtual ~QpFormulation() = default;

  // Joint state task
  void SetPositionErrors(const PosErrors& ep);
  void SetVelocityErrors(const VelErrors& ev);
  void SetReferenceAccelerations(const AccVector& qrdd);
  void SetKP(double Kp);
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
};

}

#endif /* QP_FORMULATION_H */
