/**
 *   \file whole_body_controller.h
 *   \brief Header file for the Whole Body Controller.
 */

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <OsqpEigen/Solver.hpp>

namespace whole_body_controller_ns {

class WholeBodyController : public controller_interface::Controller<
				hardware_interface::EffortJointInterface> {
public:
  WholeBodyController();
  ~WholeBodyController();

  bool init(hardware_interface::EffortJointInterface *hw,
	    ros::NodeHandle &n) override;
  void update(const ros::Time &time, const ros::Duration &period) override;
  void starting(const ros::Time &time) override;
  void stopping(const ros::Time &time) override;

private:
  /**
   *  Initialize the default parameters for the QP solver.
   */
  void setSolverParameters(void);
  /** 
   *  Sends the @ref P_, @ref q_, @ref A_, @ref l_ and @ref u_
   *  matrices to the solver.
   */
  void setSolverData(void);
  /** 
   *  Updates the @ref P_, @ref q_, @ref A_, @ref l_ and @ref u_
   *  matrices to the solver.
   */
  void updateSolverData(void);

  /**
   *  Update the @ref P_, @ref q_, @ref A_, @ref l_ and @ref u_
   *  matrices according to the current robot state.
   */
  void updateQPFormulation(void);
  /**
   *  Update the @ref P_ matrix according to the robot state.
   */
  bool updateHessianMatrix(void);
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
  
  /// Handle for every controllable joint
  std::vector<hardware_interface::JointHandle> joint_handles_;
  /// Store initial positions
  std::vector<double> init_pos_;

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
};
} // namespace whole_body_controller_ns
