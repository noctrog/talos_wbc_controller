#include <talos_qp_legs_controller/whole_body_controller.h>
#include <eigen3/Eigen/Eigen>
#include <OsqpEigen/OsqpEigen.h>

namespace whole_body_controller_ns {

  WholeBodyController::WholeBodyController()
  {
    // Set matrix and variable sizes
    P_.resize(2,2);
    q_.resize(2);
    A_.resize(3,2);
    l_.resize(3);
    u_.resize(3);
  }

  WholeBodyController::~WholeBodyController()
  {

  }

  bool WholeBodyController::init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &nh)
  {
    std::vector<std::string> joint_names;

    // If parameters are not loaded to the parameter server, throw an error. Otherwise load them
    if (!nh.getParam("joints", joint_names)) {
      std::string error_string = "Failed to load " + nh.getNamespace() + "/joints from parameter server";
      ROS_ERROR_STREAM(error_string);
      return false;
    }

    // Save joint names
    for(size_t i = 0; i < joint_names.size(); ++i) {
      joint_handles_.push_back(hw->getHandle(joint_names.at(i)));
    }

    // Generate QP formulation matrices for initial state
    updateQPFormulation();

    // Set the QP solver default parameters
    setSolverParameters();
    // Initialize the matrices of the QP solver
    setSolverData();

    // Instantiate the solver
    if (not solver_.initSolver()) std::runtime_error("Could not instantiate solver!");

    return true;
  }

  void WholeBodyController::starting(const ros::Time &time)
  {
    init_pos_.clear();
    init_pos_.reserve(joint_handles_.size());

    for (size_t i = 0; i < joint_handles_.size(); i++) {
      init_pos_.push_back(joint_handles_[i].getPosition());
    }
  }

  void WholeBodyController::stopping(const ros::Time &time)
  {
    for (auto handle : joint_handles_) {
      handle.setCommand(0.0);
    }
  }

  void WholeBodyController::update(const ros::Time &time, const ros::Duration &period)
  {
    // Update the QP formulation from the current robot state
    updateQPFormulation();

    // Send the new matrices to the QP solver
    updateSolverData();

    // Solve the QP problem
    if (not solver_.solve()) std::runtime_error("Solution not found!");

    // Retrieve the solution
    Eigen::VectorXd solution;
    solution = solver_.getSolution();

    // Print solution
    std::stringstream sol_ss;
    sol_ss << "La matriz P es: " << std::endl << P_ << std::endl
	   << "La matriz A es: " << std::endl << A_ << std::endl
	   << "La solucion es: " << std::endl << solution << std::endl;
    ROS_INFO_STREAM(sol_ss.str());

    joint_handles_.at(2).setCommand(solution(1));
  }

  void WholeBodyController::setSolverParameters()
  {
    // Set warm start
    solver_.settings()->setWarmStart(true);
    solver_.settings()->setAlpha(1.0);

    // Set the number of variables and constraints
    solver_.data()->setNumberOfVariables(2);
    solver_.data()->setNumberOfConstraints(3);
  }

  void WholeBodyController::setSolverData()
  {
    if (not solver_.data()->setHessianMatrix(P_)) std::runtime_error("Could not set Hessian matrix!");
    if (not solver_.data()->setGradient(q_)) std::runtime_error("Could not set gradient matrix!");
    if (not solver_.data()->setLinearConstraintsMatrix(A_)) std::runtime_error("Could not set linear constraint matrix!");
    if (not solver_.data()->setLowerBound(l_)) std::runtime_error("Could not set lower bound!");
    if (not solver_.data()->setUpperBound(u_)) std::runtime_error("Could not set upper bound!");
  }

  void WholeBodyController::updateSolverData()
  {
    if (not solver_.updateHessianMatrix(P_)) std::runtime_error("Could not set Hessian matrix!");
    if (not solver_.updateGradient(q_)) std::runtime_error("Could not set gradient matrix!");
    if (not solver_.updateLinearConstraintsMatrix(A_)) std::runtime_error("Could not set linear constraint matrix!");
    if (not solver_.updateLowerBound(l_)) std::runtime_error("Could not set lower bound!");
    if (not solver_.updateUpperBound(u_)) std::runtime_error("Could not set upper bound!");
  }

  bool WholeBodyController::updateHessianMatrix()
  {
    // TODO
    P_.resize(2, 2);
    P_.insert(0, 0) = 4. + std::cos(ros::Time::now().toSec()); P_.insert(0, 1) = 1.;
    P_.insert(1, 0) = 1.; P_.insert(1, 1) = 2.;
    return true;
  }

  bool WholeBodyController::updateGradientMatrix()
  {
    // TODO
    q_ << 1., 1.;
    return true;
  }

  bool WholeBodyController::updateBounds()
  {
    // TODO
    l_ << 1., 0., 0.;
    u_ << 1., 0.7, 0.7;
    return true;
  }

  bool WholeBodyController::updateLinearConstraints()
  {
    // TODO
    A_.resize(3, 2);
    A_.insert(0, 0) = 1.; A_.insert(0, 1) = 1.;
    A_.insert(1, 0) = 1.; A_.insert(1, 1) = 0.;
    A_.insert(2, 0) = 0.; A_.insert(2, 1) = 1.;
    return true;
  }

  void WholeBodyController::updateQPFormulation()
  {
    // Update the QP formulation from the current robot state
    if (not updateHessianMatrix()) std::runtime_error("Error while calculating hessian matrix");
    if (not updateGradientMatrix()) std::runtime_error("Error computing the gradient matrix");
    if (not updateBounds()) std::runtime_error("Error computing the bounds");
    if (not updateLinearConstraints()) std::runtime_error("Error computing the linear constraint matrix");
  }

}

// Register the plugin
PLUGINLIB_EXPORT_CLASS(whole_body_controller_ns::WholeBodyController, controller_interface::ControllerBase);
