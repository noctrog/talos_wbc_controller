#ifndef TALOS_WBC_JOINT_TRAJECTORY_CONTROLLER_HXX
#define TALOS_WBC_JOINT_TRAJECTORY_CONTROLLER_HXX

namespace joint_trajectory_controller
{

template <class SegmentImpl, class HardwareInterface, class HardwareAdapter>
inline bool JointTrajectoryWholeBodyController<SegmentImpl, HardwareInterface, HardwareAdapter>::
getContactsAtInstant(const ContactPerLink& curr_traj,
		     const typename Segment::Scalar& time)
{
  typedef typename Segment::Scalar Time;
  typedef typename ContactPerLink::const_iterator SegmentIterator;
  typedef typename std::iterator_traits<SegmentIterator>::value_type Segment;

  SegmentIterator first = curr_traj.begin();
  SegmentIterator last = curr_traj.end();

  auto isBeforeSegment = [](const Time& time, const Segment& segment){return time < segment.getTime();};

  // Get the corresponding iterator for the current segment of the trajectory
  SegmentIterator it =
    (first == last || isBeforeSegment(time, *first))
	  ? last // Optimization when time preceeds all segments, or when an empty range is passed
	  : --std::upper_bound(first, last, time, isBeforeSegment);

  // Get the corresponding position of the contact sequence
  size_t contact_id = 0;
  if (it != last) {
    // Segment found at specified time
    return it->getContact();
  } else if (!curr_traj.empty()) {
    // Specified time preceeds trajectory start time
    return first->getContact();
  }
}

template <class SegmentImpl, class HardwareInterface, class HardwareAdapter>
inline void JointTrajectoryWholeBodyController<SegmentImpl, HardwareInterface, HardwareAdapter>::
starting(const ros::Time& time)
{
  // Update time data
  TimeData time_data;
  time_data.time   = time;
  time_data.uptime = ros::Time(0.0);
  time_data_.initRT(time_data);

  // Hold current position
  setHoldPosition(time_data.uptime);

  // Initialize last state update time
  last_state_publish_time_ = time_data.uptime;

  // Hardware interface adapter
  hw_iface_adapter_.starting(time_data.uptime);
}

template <class SegmentImpl, class HardwareInterface, class HardwareAdapter>
inline void JointTrajectoryWholeBodyController<SegmentImpl, HardwareInterface, HardwareAdapter>::
stopping(const ros::Time& /*time*/)
{
  preemptActiveGoal();
}

template <class SegmentImpl, class HardwareInterface, class HardwareAdapter>
inline void JointTrajectoryWholeBodyController<SegmentImpl, HardwareInterface, HardwareAdapter>::
trajectoryCommandCB(const JointContactTrajectoryConstPtr& msg)
{
  const bool update_ok = updateTrajectoryCommand(msg, RealtimeGoalHandlePtr());
  if (update_ok) {preemptActiveGoal();}
}

template <class SegmentImpl, class HardwareInterface, class HardwareAdapter>
inline void JointTrajectoryWholeBodyController<SegmentImpl, HardwareInterface, HardwareAdapter>::
preemptActiveGoal()
{
  RealtimeGoalHandlePtr current_active_goal(rt_active_goal_);

  // Cancels the currently active goal
  if (current_active_goal)
  {
    // Marks the current goal as canceled
   while(realtime_busy_){
       ros::Duration(0.001).sleep();
    }
    rt_active_goal_.reset();
    current_active_goal->gh_.setCanceled();
  }
}

template <class SegmentImpl, class HardwareInterface, class HardwareAdapter>
JointTrajectoryWholeBodyController<SegmentImpl, HardwareInterface, HardwareAdapter>::
JointTrajectoryWholeBodyController()
  : verbose_(false), // Set to true during debugging
    hold_trajectory_ptr_(new Trajectory),
    hold_contact_trajectory_ptr_(new ContactTrajectory),
    solver_(new Solver)
{
  // The verbose parameter is for advanced use as it breaks real-time safety
  // by enabling ROS logging services
  if (verbose_)
  {
    ROS_WARN_STREAM(
        "The joint_trajectory_controller verbose flag is enabled. "
        << "This flag breaks real-time safety and should only be "
        << "used for debugging");
  }
}

template <class SegmentImpl, class HardwareInterface, class HardwareAdapter>
bool JointTrajectoryWholeBodyController<SegmentImpl, HardwareInterface, HardwareAdapter>::init(HardwareInterface* hw,
                                                                     ros::NodeHandle&   root_nh,
                                                                     ros::NodeHandle&   controller_nh)
{
  realtime_busy_ = false;
  using namespace internal;

  // Cache controller node handle
  controller_nh_ = controller_nh;

  // Controller name
  name_ = getLeafNamespace(controller_nh_);

  // State publish rate
  double state_publish_rate = 50.0;
  controller_nh_.getParam("state_publish_rate", state_publish_rate);
  ROS_DEBUG_STREAM_NAMED(name_, "Controller state will be published at " << state_publish_rate << "Hz.");
  state_publisher_period_ = ros::Duration(1.0 / state_publish_rate);

  // Action status checking update rate
  double action_monitor_rate = 20.0;
  controller_nh_.getParam("action_monitor_rate", action_monitor_rate);
  action_monitor_period_ = ros::Duration(1.0 / action_monitor_rate);
  ROS_DEBUG_STREAM_NAMED(name_, "Action status changes will be monitored at " << action_monitor_rate << "Hz.");

  // Stop trajectory duration
  stop_trajectory_duration_ = 0.0;
  if (!controller_nh_.getParam("stop_trajectory_duration", stop_trajectory_duration_))
  {
    // TODO: Remove this check/warning in Indigo
    if (controller_nh_.getParam("hold_trajectory_duration", stop_trajectory_duration_))
    {
      ROS_WARN("The 'hold_trajectory_duration' has been deprecated in favor of the 'stop_trajectory_duration' parameter. Please update your controller configuration.");
    }
  }
  ROS_DEBUG_STREAM_NAMED(name_, "Stop trajectory has a duration of " << stop_trajectory_duration_ << "s.");

  // Checking if partial trajectories are allowed
  controller_nh_.param<bool>("allow_partial_joints_goal", allow_partial_joints_goal_, false);
  if (allow_partial_joints_goal_)
  {
    ROS_DEBUG_NAMED(name_, "Goals with partial set of joints are allowed");
  }

  // List of controlled joints
  joint_names_ = getStrings(controller_nh_, "joints");
  if (joint_names_.empty()) {return false;}
  const unsigned int n_joints = joint_names_.size();

  // URDF joints
  urdf::ModelSharedPtr urdf = getUrdf(root_nh, "robot_description");
  if (!urdf) {return false;}

  std::vector<urdf::JointConstSharedPtr> urdf_joints = getUrdfJoints(*urdf, joint_names_);
  if (urdf_joints.empty()) {return false;}
  assert(n_joints == urdf_joints.size());

  // Initialize members
  joints_.resize(n_joints);
  angle_wraparound_.resize(n_joints);
  for (unsigned int i = 0; i < n_joints; ++i)
  {
    // Joint handle
    try {joints_[i] = hw->getHandle(joint_names_[i]);}
    catch (...)
    {
      ROS_ERROR_STREAM_NAMED(name_, "Could not find joint '" << joint_names_[i] << "' in '" <<
                                    this->getHardwareInterfaceType() << "'.");
      return false;
    }

    // Whether a joint is continuous (ie. has angle wraparound)
    angle_wraparound_[i] = urdf_joints[i]->type == urdf::Joint::CONTINUOUS;
    const std::string not_if = angle_wraparound_[i] ? "" : "non-";

    ROS_DEBUG_STREAM_NAMED(name_, "Found " << not_if << "continuous joint '" << joint_names_[i] << "' in '" <<
                                  this->getHardwareInterfaceType() << "'.");
  }

  assert(joints_.size() == angle_wraparound_.size());
  ROS_DEBUG_STREAM_NAMED(name_, "Initialized controller '" << name_ << "' with:" <<
                         "\n- Number of joints: " << joints_.size() <<
                         "\n- Hardware interface type: '" << this->getHardwareInterfaceType() << "'" <<
                         "\n- Trajectory segment type: '" << hardware_interface::internal::demangledTypeName<SegmentImpl>() << "'");

  // Default tolerances
  ros::NodeHandle tol_nh(controller_nh_, "constraints");
  default_tolerances_ = getSegmentTolerances<Scalar>(tol_nh, joint_names_);

  // Hardware interface adapter
  hw_iface_adapter_.init(joints_, controller_nh_);

  // ROS API: Subscribed topics
  trajectory_command_sub_ = controller_nh_.subscribe("command", 1, &JointTrajectoryWholeBodyController::trajectoryCommandCB, this);

  // ROS API: Published topics
  state_publisher_.reset(new StatePublisher(controller_nh_, "state", 1));

  // ROS API: Action interface
  action_server_.reset(new ActionServer(controller_nh_, "follow_joint_trajectory",
                                        boost::bind(&JointTrajectoryWholeBodyController::goalCB,   this, _1),
                                        boost::bind(&JointTrajectoryWholeBodyController::cancelCB, this, _1),
                                        false));
  action_server_->start();

  // ROS API: Provided services
  query_state_service_ = controller_nh_.advertiseService("query_state",
                                                         &JointTrajectoryWholeBodyController::queryStateService,
                                                         this);

  // Preeallocate resources
  current_state_       = typename Segment::State(n_joints);
  desired_state_       = typename Segment::State(n_joints);
  state_error_         = typename Segment::State(n_joints);
  desired_joint_state_ = typename Segment::State(1);
  state_joint_error_   = typename Segment::State(1);

  successful_joint_traj_ = boost::dynamic_bitset<>(joints_.size());

  // Initialize trajectory with all joints
  typename Segment::State current_joint_state_ = typename Segment::State(1);
  for (unsigned int i = 0; i < n_joints; ++i)
  {
	  current_joint_state_.position[0]= current_state_.position[i];
	  current_joint_state_.velocity[0]= current_state_.velocity[i];
	  Segment hold_segment(0.0, current_joint_state_, 0.0, current_joint_state_);

	  TrajectoryPerJoint joint_segment;
	  joint_segment.resize(1, hold_segment);
	  hold_trajectory_ptr_->push_back(joint_segment);
  }

  // Initialize contacts
  contact_link_names_ = {"left_sole_link", "right_sole_link"}; // Both feet are on the ground
  for (unsigned int i = 0; i < 2; ++i) {
    contact_segment.emplace_back(true, 0.0);
  }
  
  {
    state_publisher_->lock();
    state_publisher_->msg_.joint_names = joint_names_;
    state_publisher_->msg_.desired.positions.resize(n_joints);
    state_publisher_->msg_.desired.velocities.resize(n_joints);
    state_publisher_->msg_.desired.accelerations.resize(n_joints);
    state_publisher_->msg_.actual.positions.resize(n_joints);
    state_publisher_->msg_.actual.velocities.resize(n_joints);
    state_publisher_->msg_.error.positions.resize(n_joints);
    state_publisher_->msg_.error.velocities.resize(n_joints);
    state_publisher_->unlock();
  }

  return true;
}

template <class SegmentImpl, class HardwareInterface, class HardwareAdapter>
void JointTrajectoryWholeBodyController<SegmentImpl, HardwareInterface, HardwareAdapter>::
update(const ros::Time& time, const ros::Duration& period)
{
  realtime_busy_ = true;
  // Get currently followed trajectory
  TrajectoryPtr curr_traj_ptr;
  curr_trajectory_box_.get(curr_traj_ptr);
  Trajectory& curr_traj = *curr_traj_ptr;
  ContactTrajectoryPtr curr_contact_traj_ptr;
  curr_contact_trajectory_box_.get(curr_contact_traj_ptr);
  ContactTrajectory& curr_contact_traj = *curr_contact_traj_ptr;

  // Update time data
  TimeData time_data;
  time_data.time   = time;                                     // Cache current time
  time_data.period = period;                                   // Cache current control period
  time_data.uptime = time_data_.readFromRT()->uptime + period; // Update controller uptime
  time_data_.writeFromNonRT(time_data); // TODO: Grrr, we need a lock-free data structure here!

  // NOTE: It is very important to execute the two above code blocks in the specified sequence: first get current
  // trajectory, then update time data. Hopefully the following paragraph sheds a bit of light on the rationale.
  // The non-rt thread responsible for processing new commands enqueues trajectories that can start at the _next_
  // control cycle (eg. zero start time) or later (eg. when we explicitly request a start time in the future).
  // If we reverse the order of the two blocks above, and update the time data first; it's possible that by the time we
  // fetch the currently followed trajectory, it has been updated by the non-rt thread with something that starts in the
  // next control cycle, leaving the current cycle without a valid trajectory.

  // Debug
  // ROS_INFO("update(): se tienen %d contactos. Sin embargo se tienen %d links",
	   // curr_contact_traj.size(), contact_link_names_.size());
  // Sample current contacts if there is any contact trajectory
  std::vector<ContactSegment> curr_contacts(contact_link_names_.size());
  std::vector<std::string> curr_contact_frame_names;
  // Get contacts for at current time, if any
  if (curr_contact_traj.size() > 0) {
    for (size_t i = 0; i < curr_contact_traj.size(); ++i) {
      bool bContact = getContactsAtInstant(curr_contact_traj[i], time_data.uptime.toSec());
      curr_contacts.emplace_back(ContactSegment(bContact, time_data.uptime.toSec()));
      if (bContact)
	curr_contact_frame_names.push_back(contact_link_names_.at(i));
    }
  }


  // Update current state and state error
  for (unsigned int i = 0; i < joints_.size(); ++i)
  {
    current_state_.position[i] = joints_[i].getPosition();
    current_state_.velocity[i] = joints_[i].getVelocity();
    // There's no acceleration data available in a joint handle

    typename TrajectoryPerJoint::const_iterator segment_it = sample(curr_traj[i], time_data.uptime.toSec(), desired_joint_state_);
    if (curr_traj[i].end() == segment_it)
    {
      // Non-realtime safe, but should never happen under normal operation
      ROS_ERROR_NAMED(name_,
                      "Unexpected error: No trajectory defined at current time. Please contact the package maintainer.");
      return;
    }
    desired_state_.position[i] = desired_joint_state_.position[0];
    desired_state_.velocity[i] = desired_joint_state_.velocity[0];
    desired_state_.acceleration[i] = desired_joint_state_.acceleration[0];

    state_joint_error_.position[0] = angles::shortest_angular_distance(current_state_.position[i],desired_joint_state_.position[0]);
    state_joint_error_.velocity[0] = desired_joint_state_.velocity[0] - current_state_.velocity[i];
    state_joint_error_.acceleration[0] = 0.0;

    state_error_.position[i] = angles::shortest_angular_distance(current_state_.position[i],desired_joint_state_.position[0]);
    state_error_.velocity[i] = desired_joint_state_.velocity[0] - current_state_.velocity[i];
    state_error_.acceleration[i] = 0.0;

    //Check tolerances
    const RealtimeGoalHandlePtr rt_segment_goal = segment_it->getGoalHandle();
    if (rt_segment_goal && rt_segment_goal == rt_active_goal_)
    {
      // Check tolerances
      if (time_data.uptime.toSec() < segment_it->endTime())
      {
        // Currently executing a segment: check path tolerances
        const SegmentTolerancesPerJoint<Scalar>& joint_tolerances = segment_it->getTolerances();
        if (!checkStateTolerancePerJoint(state_joint_error_, joint_tolerances.state_tolerance))
        {
          if (verbose_)
          {
            ROS_ERROR_STREAM_NAMED(name_,"Path tolerances failed for joint: " << joint_names_[i]);
            checkStateTolerancePerJoint(state_joint_error_, joint_tolerances.state_tolerance, true);
          }

          if(rt_segment_goal && rt_segment_goal->preallocated_result_)
          {
            rt_segment_goal->preallocated_result_->error_code =
	      talos_wbc_controller::FollowContactJointTrajectoryResult::PATH_TOLERANCE_VIOLATED;
            rt_segment_goal->setAborted(rt_segment_goal->preallocated_result_);
            rt_active_goal_.reset();
            successful_joint_traj_.reset();
          }
          else{
            ROS_ERROR_STREAM("rt_segment_goal->preallocated_result_ NULL Pointer");
          }
        }
      }
      else if (segment_it == --curr_traj[i].end())
      {
        if (verbose_)
          ROS_DEBUG_STREAM_THROTTLE_NAMED(1,name_,"Finished executing last segment, checking goal tolerances");

        // Controller uptime
        const ros::Time uptime = time_data_.readFromRT()->uptime;

        // Checks that we have ended inside the goal tolerances
        const SegmentTolerancesPerJoint<Scalar>& tolerances = segment_it->getTolerances();
        const bool inside_goal_tolerances = checkStateTolerancePerJoint(state_joint_error_, tolerances.goal_state_tolerance);

        if (inside_goal_tolerances)
        {
          successful_joint_traj_[i] = 1;
        }
        else if (uptime.toSec() < segment_it->endTime() + tolerances.goal_time_tolerance)
        {
          // Still have some time left to meet the goal state tolerances
        }
        else
        {
          if (verbose_)
          {
            ROS_ERROR_STREAM_NAMED(name_,"Goal tolerances failed for joint: "<< joint_names_[i]);
            // Check the tolerances one more time to output the errors that occurs
            checkStateTolerancePerJoint(state_joint_error_, tolerances.goal_state_tolerance, true);
          }

          if(rt_segment_goal){
            rt_segment_goal->preallocated_result_->error_code = talos_wbc_controller::FollowContactJointTrajectoryResult::GOAL_TOLERANCE_VIOLATED;
            rt_segment_goal->setAborted(rt_segment_goal->preallocated_result_);
          }
          else
          {
            ROS_ERROR_STREAM("rt_segment_goal->preallocated_result_ NULL Pointer");
          }
          rt_active_goal_.reset();
          successful_joint_traj_.reset();
        }
      }
    }
  }


  //If there is an active goal and all segments finished successfully then set goal as succeeded
  RealtimeGoalHandlePtr current_active_goal(rt_active_goal_);
  if (current_active_goal && current_active_goal->preallocated_result_ && successful_joint_traj_.count() == joints_.size())
  {
    current_active_goal->preallocated_result_->error_code = talos_wbc_controller::FollowContactJointTrajectoryResult::SUCCESSFUL;
    current_active_goal->setSucceeded(current_active_goal->preallocated_result_);
    rt_active_goal_.reset();
    successful_joint_traj_.reset();
  }

  // TODO Solve QP problem
  solver_->SetRobotState(current_state_.position, current_state_.velocity,
			 curr_contact_frame_names);


  // TODO: Hardware interface adapter: send torque commands through acceleration
  hw_iface_adapter_.updateCommand(time_data.uptime, time_data.period,
                                  desired_state_, state_error_);

  // Set action feedback
  if (rt_active_goal_ && rt_active_goal_->preallocated_feedback_)
  {
    rt_active_goal_->preallocated_feedback_->header.stamp          = time_data_.readFromRT()->time;
    rt_active_goal_->preallocated_feedback_->desired.positions     = desired_state_.position;
    rt_active_goal_->preallocated_feedback_->desired.velocities    = desired_state_.velocity;
    rt_active_goal_->preallocated_feedback_->desired.accelerations = desired_state_.acceleration;
    rt_active_goal_->preallocated_feedback_->actual.positions      = current_state_.position;
    rt_active_goal_->preallocated_feedback_->actual.velocities     = current_state_.velocity;
    rt_active_goal_->preallocated_feedback_->error.positions       = state_error_.position;
    rt_active_goal_->preallocated_feedback_->error.velocities      = state_error_.velocity;
    rt_active_goal_->setFeedback( rt_active_goal_->preallocated_feedback_ );
  }

  // Publish state
  publishState(time_data.uptime);
  realtime_busy_ = false;
}

template <class SegmentImpl, class HardwareInterface, class HardwareAdapter>
bool JointTrajectoryWholeBodyController<SegmentImpl, HardwareInterface, HardwareAdapter>::
updateTrajectoryCommand(const JointContactTrajectoryConstPtr& msg, RealtimeGoalHandlePtr gh)
{
  typedef InitContactJointTrajectoryOptions<Trajectory, ContactTrajectory> Options;

  // Preconditions
  if (!this->isRunning())
  {
    ROS_ERROR_NAMED(name_, "Can't accept new commands. Controller is not running.");
    return false;
  }

  if (!msg)
  {
    ROS_WARN_NAMED(name_, "Received null-pointer trajectory message, skipping.");
    return false;
  }

  // Get contact link names
  contact_link_names_ = msg->contact_link_names;

  // Time data
  TimeData* time_data = time_data_.readFromRT(); // TODO: Grrr, we need a lock-free data structure here!

  // Time of the next update
  const ros::Time next_update_time = time_data->time + time_data->period;

  // Uptime of the next update
  ros::Time next_update_uptime = time_data->uptime + time_data->period;

  // Hold current position if trajectory is empty
  if (msg->trajectory.points.empty())
  {
    setHoldPosition(time_data->uptime, gh);
    ROS_DEBUG_NAMED(name_, "Empty trajectory command, stopping.");
    return true;
  }

  // Trajectory initialization options
  TrajectoryPtr curr_traj_ptr;
  curr_trajectory_box_.get(curr_traj_ptr);
  ContactTrajectoryPtr curr_cont_traj_ptr;
  curr_contact_trajectory_box_.get(curr_cont_traj_ptr);

  Options options;
  options.other_time_base            = &next_update_uptime;
  options.current_trajectory         = curr_traj_ptr.get();
  options.current_contact_trajectory = curr_cont_traj_ptr.get();
  options.joint_names                = &joint_names_;
  options.angle_wraparound           = &angle_wraparound_;
  options.rt_goal_handle             = gh;
  options.default_tolerances         = &default_tolerances_;
  options.allow_partial_joints_goal  = allow_partial_joints_goal_;

  // Update currently executing trajectory
  try
  {
    TrajectoryPtr traj_ptr(new Trajectory);
    ContactTrajectoryPtr cont_traj_ptr(new ContactTrajectory);
    initContactJointTrajectory<Trajectory, ContactTrajectory> (*msg, next_update_time,
							       *traj_ptr, *cont_traj_ptr, options);
    if (!traj_ptr->empty())
    {
      curr_trajectory_box_.set(traj_ptr);
      curr_contact_trajectory_box_.set(cont_traj_ptr);
    }
    else
    {
      // All trajectory points are in the past, nothing new to execute. Keep on executing current trajectory
      return false;
    }
  }
  catch(const std::invalid_argument& ex)
  {
    ROS_ERROR_STREAM_NAMED(name_, ex.what());
    return false;
  }
  catch(...)
  {
    ROS_ERROR_NAMED(name_, "Unexpected exception caught when initializing trajectory from ROS message data.");
    return false;
  }

  return true;
}

template <class SegmentImpl, class HardwareInterface, class HardwareAdapter>
void JointTrajectoryWholeBodyController<SegmentImpl, HardwareInterface, HardwareAdapter>::
goalCB(GoalHandle gh)
{
  ROS_DEBUG_STREAM_NAMED(name_,"Received new action goal");

  // Precondition: Running controller
  if (!this->isRunning())
  {
    ROS_ERROR_NAMED(name_, "Can't accept new action goals. Controller is not running.");
    talos_wbc_controller::FollowContactJointTrajectoryResult result;
    result.error_code = talos_wbc_controller::FollowContactJointTrajectoryResult::INVALID_GOAL;
    gh.setRejected(result);
    return;
  }

  // If partial joints goals are not allowed, goal should specify all controller joints
  if (!allow_partial_joints_goal_)
  {
    if (gh.getGoal()->trajectory.trajectory.joint_names.size() != joint_names_.size())
    {
      ROS_ERROR_NAMED(name_, "Joints on incoming goal don't match the controller joints.");
      talos_wbc_controller::FollowContactJointTrajectoryResult result;
      result.error_code = talos_wbc_controller::FollowContactJointTrajectoryResult::INVALID_JOINTS;
      gh.setRejected(result);
      return;
    }
  }

  // Goal should specify valid controller joints (they can be ordered differently). Reject if this is not the case
  using internal::mapping;
  std::vector<unsigned int> mapping_vector = mapping(gh.getGoal()->trajectory.trajectory.joint_names, joint_names_);

  if (mapping_vector.empty())
  {
    ROS_ERROR_NAMED(name_, "Joints on incoming goal don't match the controller joints.");
    talos_wbc_controller::FollowContactJointTrajectoryResult result;
    result.error_code = talos_wbc_controller::FollowContactJointTrajectoryResult::INVALID_JOINTS;
    gh.setRejected(result);
    return;
  }

  // Try to update new trajectory
  RealtimeGoalHandlePtr rt_goal(new RealtimeGoalHandle(gh));
  const bool update_ok = updateTrajectoryCommand(internal::share_member(gh.getGoal(), gh.getGoal()->trajectory),
                                                 rt_goal);
  rt_goal->preallocated_feedback_->joint_names = joint_names_;

  if (update_ok)
  {
    // Accept new goal
    preemptActiveGoal();
    gh.setAccepted();
    while(realtime_busy_){
       ros::Duration(0.001).sleep();
    }
    rt_active_goal_ = rt_goal;

    // Setup goal status checking timer
    goal_handle_timer_ = controller_nh_.createTimer(action_monitor_period_,
                                                    &RealtimeGoalHandle::runNonRealtime,
                                                    rt_goal);
    goal_handle_timer_.start();
  }
  else
  {
    // Reject invalid goal
    talos_wbc_controller::FollowContactJointTrajectoryResult result;
    result.error_code = talos_wbc_controller::FollowContactJointTrajectoryResult::INVALID_GOAL;
    gh.setRejected(result);
  }
}

template <class SegmentImpl, class HardwareInterface, class HardwareAdapter>
void JointTrajectoryWholeBodyController<SegmentImpl, HardwareInterface, HardwareAdapter>::
cancelCB(GoalHandle gh)
{
  RealtimeGoalHandlePtr current_active_goal(rt_active_goal_);

  // Check that cancel request refers to currently active goal (if any)
  if (current_active_goal && current_active_goal->gh_ == gh)
  {
    // Reset current goal
    while(realtime_busy_){
       ros::Duration(0.001).sleep();
    }
    rt_active_goal_.reset();

    // Controller uptime
    const ros::Time uptime = time_data_.readFromRT()->uptime;

    // Enter hold current position mode
    setHoldPosition(uptime);
    ROS_DEBUG_NAMED(name_, "Canceling active action goal because cancel callback recieved from actionlib.");

    // Mark the current goal as canceled
    current_active_goal->gh_.setCanceled();
  }
}

template <class SegmentImpl, class HardwareInterface, class HardwareAdapter>
bool JointTrajectoryWholeBodyController<SegmentImpl, HardwareInterface, HardwareAdapter>::
queryStateService(control_msgs::QueryTrajectoryState::Request&  req,
                  control_msgs::QueryTrajectoryState::Response& resp)
{
  // Preconditions
  if (!this->isRunning())
  {
    ROS_ERROR_NAMED(name_, "Can't sample trajectory. Controller is not running.");
    return false;
  }

  // Convert request time to internal monotonic representation
  TimeData* time_data = time_data_.readFromRT();
  const ros::Duration time_offset = req.time - time_data->time;
  const ros::Time sample_time = time_data->uptime + time_offset;

  // Sample trajectory at requested time
  TrajectoryPtr curr_traj_ptr;
  curr_trajectory_box_.get(curr_traj_ptr);
  Trajectory& curr_traj = *curr_traj_ptr;

  typename Segment::State response_point = typename Segment::State(joint_names_.size());

  for (unsigned int i = 0; i < joints_.size(); ++i)
  {
    typename Segment::State state;
    typename TrajectoryPerJoint::const_iterator segment_it = sample(curr_traj[i], sample_time.toSec(), state);
    if (curr_traj[i].end() == segment_it)
    {
      ROS_ERROR_STREAM_NAMED(name_, "Requested sample time precedes trajectory start time.");
      return false;
    }

    response_point.position[i]     = state.position[0];
    response_point.velocity[i]     = state.velocity[0];
    response_point.acceleration[i] = state.acceleration[0];
  }

  // Populate response
  resp.name         = joint_names_;
  resp.position     = response_point.position;
  resp.velocity     = response_point.velocity;
  resp.acceleration = response_point.acceleration;

  return true;
}

template <class SegmentImpl, class HardwareInterface, class HardwareAdapter>
void JointTrajectoryWholeBodyController<SegmentImpl, HardwareInterface, HardwareAdapter>::
publishState(const ros::Time& time)
{
  // Check if it's time to publish
  if (!state_publisher_period_.isZero() && last_state_publish_time_ + state_publisher_period_ < time)
  {
    if (state_publisher_ && state_publisher_->trylock())
    {
      last_state_publish_time_ += state_publisher_period_;

      state_publisher_->msg_.header.stamp          = time_data_.readFromRT()->time;
      state_publisher_->msg_.desired.positions     = desired_state_.position;
      state_publisher_->msg_.desired.velocities    = desired_state_.velocity;
      state_publisher_->msg_.desired.accelerations = desired_state_.acceleration;
      state_publisher_->msg_.actual.positions      = current_state_.position;
      state_publisher_->msg_.actual.velocities     = current_state_.velocity;
      state_publisher_->msg_.error.positions       = state_error_.position;
      state_publisher_->msg_.error.velocities      = state_error_.velocity;

      state_publisher_->unlockAndPublish();
    }
  }
}

template <class SegmentImpl, class HardwareInterface, class HardwareAdapter>
void JointTrajectoryWholeBodyController<SegmentImpl, HardwareInterface, HardwareAdapter>::
setHoldPosition(const ros::Time& time, RealtimeGoalHandlePtr gh)
{
  // Settle position in a fixed time. We do the following:
  // - Create segment that goes from current (pos,vel) to (pos,-vel) in 2x the desired stop time
  // - Assuming segment symmetry, sample segment at its midpoint (desired stop time). It should have zero velocity
  // - Create segment that goes from current state to above zero velocity state, in the desired time
  // NOTE: The symmetry assumption from the second point above might not hold for all possible segment types

  assert(joint_names_.size() == hold_trajectory_ptr_->size());

  typename Segment::State hold_start_state_ = typename Segment::State(1);
  typename Segment::State hold_end_state_ = typename Segment::State(1);

  const typename Segment::Time start_time  = time.toSec();
  const typename Segment::Time end_time    = time.toSec() + stop_trajectory_duration_;
  const typename Segment::Time end_time_2x = time.toSec() + 2.0 * stop_trajectory_duration_;

  // Create segment that goes from current (pos,vel) to (pos,-vel)
  const unsigned int n_joints = joints_.size();
  for (unsigned int i = 0; i < n_joints; ++i)
  {
    hold_start_state_.position[0]     =  joints_[i].getPosition();
    hold_start_state_.velocity[0]     =  joints_[i].getVelocity();
    hold_start_state_.acceleration[0] =  0.0;

    hold_end_state_.position[0]       =  joints_[i].getPosition();
    hold_end_state_.velocity[0]       = -joints_[i].getVelocity();
    hold_end_state_.acceleration[0]   =  0.0;

    (*hold_trajectory_ptr_)[i].front().init(start_time,  hold_start_state_,
                                                             end_time_2x, hold_end_state_);

    // Sample segment at its midpoint, that should have zero velocity
    (*hold_trajectory_ptr_)[i].front().sample(end_time, hold_end_state_);

    // Now create segment that goes from current state to one with zero end velocity
    (*hold_trajectory_ptr_)[i].front().init(start_time, hold_start_state_,
                                                             end_time,   hold_end_state_);

    // Set goal handle for the segment
    (*hold_trajectory_ptr_)[i].front().setGoalHandle(gh);
  }
  curr_trajectory_box_.set(hold_trajectory_ptr_);
  curr_contact_trajectory_box_.set(hold_contact_trajectory_ptr_);
}

} // namespace

#endif // header guard
