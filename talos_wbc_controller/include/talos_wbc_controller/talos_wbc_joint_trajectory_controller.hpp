#ifndef TALOS_WBC_JOINT_TRAJECTORY_CONTROLLER_HPP
#define TALOS_WBC_JOINT_TRAJECTORY_CONTROLLER_HPP

// QP Solver
#include <pinocchio/fwd.hpp>
#include <talos_wbc_controller/qp_formulation.hpp>

// C++ standard
#include <cassert>
#include <iterator>
#include <stdexcept>
#include <string>

// Boost
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/dynamic_bitset.hpp>

// ROS
#include <ros/node_handle.h>
#include <ros/package.h>

// URDF
#include <urdf/model.h>

// ROS messages
#include <control_msgs/JointTrajectoryControllerState.h>
#include <control_msgs/QueryTrajectoryState.h>
#include <trajectory_msgs/JointTrajectory.h>

// actionlib
#include <actionlib/server/action_server.h>

// realtime_tools
#include <realtime_tools/realtime_box.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

// ros_controls
#include <realtime_tools/realtime_server_goal_handle.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/internal/demangle_symbol.h>

// Project
#include <trajectory_interface/trajectory_interface.h>

#include <talos_wbc_controller/init_contact_joint_trajectory.hpp>
#include <joint_trajectory_controller/hardware_interface_adapter.h>

#include <talos_wbc_controller/contact_joint_trajectory_segment.hpp>
#include <talos_wbc_controller/contact_segment.hpp>
#include <talos_wbc_controller/JointContactTrajectory.h>
#include <talos_wbc_controller/hardware_interface_direct_effort.hpp>
#include <talos_wbc_controller/FollowContactJointTrajectoryAction.h>
#include <talos_wbc_controller/JointContactTrajectoryControllerState.h>

#include <nav_msgs/Odometry.h>

#include <ddynamic_reconfigure/ddynamic_reconfigure.h>

namespace joint_trajectory_controller
{

/**
 * \brief Controller for executing joint-space trajectories on a group of joints.
 *
 * \note Non-developer documentation and usage instructions can be found in the package's ROS wiki page.
 *
 * \tparam SegmentImpl Trajectory segment representation to use. The type must comply with the following structure:
 * \code
 * class FooSegment
 * {
 * public:
 *   // Required types
 *   typedef double                 Scalar; // Scalar can be anything convertible to double
 *   typedef Scalar                 Time;
 *   typedef PosVelAccState<Scalar> State;
 *
 *   // Default constructor
 *   FooSegment();
 *
 *   // Constructor from start and end states (boundary conditions)
 *   FooSegment(const Time&  start_time,
 *              const State& start_state,
 *              const Time&  end_time,
 *              const State& end_state);
 *
 *   // Start and end states initializer (the guts of the above constructor)
 *   // May throw std::invalid_argument if parameters are invalid
 *   void init(const Time&  start_time,
 *             const State& start_state,
 *             const Time&  end_time,
 *             const State& end_state);
 *
 *   // Sampler (realtime-safe)
 *   void sample(const Time& time, State& state) const;
 *
 *   // Accesors (realtime-safe)
 *   Time startTime()    const;
 *   Time endTime()      const;
 *   unsigned int size() const;
 * };
 * \endcode
 *
 * \tparam HardwareInterface Controller hardware interface. Currently \p hardware_interface::PositionJointInterface,
 * \p hardware_interface::VelocityJointInterface, and \p hardware_interface::EffortJointInterface are supported 
 * out-of-the-box.
 */
template <class SegmentImpl, class HardwareInterface, class HardwareAdapter = HardwareInterface>
class JointTrajectoryWholeBodyController : public controller_interface::Controller<HardwareInterface>
{
public:

  JointTrajectoryWholeBodyController();

  /** \name Non Real-Time Safe Functions
   *\{*/
  bool init(HardwareInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
  /*\}*/

  /** \name Real-Time Safe Functions
   *\{*/
  /** \brief Holds the current position. */
  void starting(const ros::Time& time);

  /** \brief Cancels the active action goal, if any. */
  void stopping(const ros::Time& /*time*/);

  void update(const ros::Time& time, const ros::Duration& period);
  /*\}*/

private:

  bool realtime_busy_;

  struct TimeData
  {
    TimeData() : time(0.0), period(0.0), uptime(0.0) {}

    ros::Time     time;   ///< Time of last update cycle
    ros::Duration period; ///< Period of last update cycle
    ros::Time     uptime; ///< Controller uptime. Set to zero at every restart.
  };

  typedef actionlib::ActionServer<talos_wbc_controller::FollowContactJointTrajectoryAction>                  ActionServer;
  typedef boost::shared_ptr<ActionServer>                                                     ActionServerPtr;
  typedef ActionServer::GoalHandle                                                            GoalHandle;
  typedef realtime_tools::RealtimeServerGoalHandle<talos_wbc_controller::FollowContactJointTrajectoryAction> RealtimeGoalHandle;
  typedef boost::shared_ptr<RealtimeGoalHandle>                                               RealtimeGoalHandlePtr;
  typedef talos_wbc_controller::JointContactTrajectory::ConstPtr                              JointTrajectoryConstPtr;
  typedef realtime_tools::RealtimePublisher<talos_wbc_controller::JointContactTrajectoryControllerState>             StatePublisher;
  typedef boost::scoped_ptr<StatePublisher>                                                   StatePublisherPtr;

  // Data types related to the continuous trajectory
  typedef ContactJointTrajectorySegment<SegmentImpl> Segment;
  typedef std::vector<Segment> TrajectoryPerJoint;
  typedef std::vector<TrajectoryPerJoint> Trajectory;
  typedef boost::shared_ptr<Trajectory> TrajectoryPtr;
  typedef boost::shared_ptr<TrajectoryPerJoint> TrajectoryPerJointPtr;
  typedef realtime_tools::RealtimeBox<TrajectoryPtr> TrajectoryBox;
  typedef typename Segment::Scalar Scalar;

  // Data types related to the contact sequence
  typedef std::vector<ContactSegment> ContactPerLink;     ///< counter part to TrajectoryPerJoint
  typedef std::vector<ContactPerLink> ContactTrajectory; ///< counter part to ContactTrajectory
  typedef boost::shared_ptr<ContactTrajectory> ContactTrajectoryPtr;
  typedef boost::shared_ptr<ContactPerLink> ContactPerLinkPtr;
  typedef realtime_tools::RealtimeBox<ContactTrajectoryPtr> ContactTrajectoryBox;

  typedef DirectEffortHardwareInterfaceAdapter<typename Segment::State> HwIfaceAdapter;
  typedef typename HardwareInterface::ResourceHandleType JointHandle;

  bool                      verbose_;            ///< Hard coded verbose flag to help in debugging
  std::string               name_;               ///< Controller name.
  std::vector<JointHandle>  joints_;             ///< Handles to controlled joints.
  std::vector<bool>         angle_wraparound_;   ///< Whether controlled joints wrap around or not.
  std::vector<std::string>  joint_names_;        ///< Controlled joint names.
  std::vector<std::string>  contact_link_names_; ///< Names of links that can have contacts
  SegmentTolerances<Scalar> default_tolerances_; ///< Default trajectory segment tolerances.
  HwIfaceAdapter            hw_iface_adapter_;   ///< Adapts desired trajectory state to HW interface.

  RealtimeGoalHandlePtr     rt_active_goal_;     ///< Currently active action goal, if any.

  /**
   * Thread-safe container with a smart pointer to trajectory currently being followed.
   * Can be either a hold trajectory or a trajectory received from a ROS message.
   *
   * We store the hold trajectory in a separate class member because the \p starting(time) method must be realtime-safe.
   * The (single segment) hold trajectory is preallocated at initialization time and its size is kept unchanged.
   */
  TrajectoryBox curr_trajectory_box_;
  TrajectoryPtr hold_trajectory_ptr_; ///< Last hold trajectory values.

  TrajectoryBox curr_com_trajectory_box_;
  TrajectoryPtr hold_com_trajectory_ptr_;

  ContactTrajectoryBox curr_contact_trajectory_box_;
  ContactTrajectoryPtr hold_contact_trajectory_ptr_;

  typename Segment::State current_state_;         ///< Preallocated workspace variable.
  typename Segment::State desired_state_;         ///< Preallocated workspace variable.
  typename Segment::State state_error_;           ///< Preallocated workspace variable.
  typename Segment::State desired_joint_state_;   ///< Preallocated workspace variable.
  typename Segment::State state_joint_error_;     ///< Preallocated workspace variable.
  typename Segment::State current_com_state_;
  typename Segment::State desired_com_state_;
  typename Segment::State error_com_state_;

  realtime_tools::RealtimeBuffer<TimeData> time_data_;

  ros::Duration state_publisher_period_;
  ros::Duration action_monitor_period_;

  typename Segment::Time stop_trajectory_duration_;
  boost::dynamic_bitset<> successful_joint_traj_;
  bool allow_partial_joints_goal_;

  // ROS API
  ros::NodeHandle    controller_nh_;
  ros::Subscriber    trajectory_command_sub_;
  ActionServerPtr    action_server_;
  ros::ServiceServer query_state_service_;
  StatePublisherPtr  state_publisher_;

  ros::Timer         goal_handle_timer_;
  ros::Time          last_state_publish_time_;

  typedef talos_wbc_controller::JointContactTrajectory JointContactTrajectory;
  typedef JointContactTrajectory::ConstPtr JointContactTrajectoryConstPtr;

  bool updateTrajectoryCommand(const JointContactTrajectoryConstPtr& msg, RealtimeGoalHandlePtr gh);
  void trajectoryCommandCB(const JointContactTrajectoryConstPtr& msg);
  void goalCB(GoalHandle gh);
  void cancelCB(GoalHandle gh);
  void preemptActiveGoal();
  bool queryStateService(control_msgs::QueryTrajectoryState::Request&  req,
                         control_msgs::QueryTrajectoryState::Response& resp);

  /**
   * \brief Publish current controller state at a throttled frequency.
   * \note This method is realtime-safe and is meant to be called from \ref update, as it shares data with it without
   * any locking.
   */
  void publishState(const ros::Time& time);

  /**
   * \brief Hold the current position.
   *
   * Substitutes the current trajectory with a single-segment one going from the current position and velocity to the
   * current position and zero velocity.
   * \note This method is realtime-safe.
   */
  void setHoldPosition(const ros::Time& time, RealtimeGoalHandlePtr gh=RealtimeGoalHandlePtr());

  bool
  getContactsAtInstant(const ContactPerLink& curr_contact_traj,
		       const typename Segment::Scalar &time);

  // QP Solver
  typedef talos_wbc_controller::QpFormulation Solver;
  typedef std::shared_ptr<Solver> SolverPtr;

  SolverPtr solver_;

  // Dynamic reconfigure, used to tune the solver's Kp and Kv constants
  std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> ddr_;
  void paramJointTaskWeight(double w);
  void paramComTaskWeight(double w);
  void paramBaseOriTaskWeight(double w);
  void paramMu(double mu);
  void paramJointKpCB(double new_kp);
  void paramComKpCB(double new_kp);
  void paramBaseOriKpCB(double new_kp);
  // Active constraints
  struct {
    bool b_equation_of_motion_constraint;
    bool b_fixed_contact_condition_constraint;
    bool b_actuation_limits_constraint;
    bool b_contact_stability_constraint;
  } SolverConstraints_;
  // Task weights
  struct {
    double joint_task_weight;
    double com_task_weight;
    double base_orientation_task_weight;
  } SolverWeights_;
  // Friction coefficient
  double mu_;
  // Task dynamics
  struct TaskDynamics {
    double Kp, Kv;
  };
  TaskDynamics JointTaskDynamics_;
  TaskDynamics ComTaskDynamics_;
  TaskDynamics BaseOriTaskDynamics_;
  // Setters
  void paramEquationOfMotion(bool activate);
  void paramFixedContactCondition(bool activate);
  void paramActuationLimits(bool activate);
  void paramContactStability(bool activate);
  
  ros::Subscriber robot_base_link_state_; // Retrieve the position and velocity of the robot's base_link
  void baseLinkCB(const nav_msgs::OdometryConstPtr& msg);
  nav_msgs::Odometry last_base_link_state_;
};

} // namespace

#include <talos_wbc_controller/talos_wbc_joint_trajectory_controller.hxx>

#endif // header guard
