#ifndef CONTACT_JOINT_TRAJECTORY_CONTROLLER_INIT_JOINT_TRAJECTORY_H
#define CONTACT_JOINT_TRAJECTORY_CONTROLLER_INIT_JOINT_TRAJECTORY_H

// C++ standard
#include <algorithm>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <vector>

// Boost
#include <boost/shared_ptr.hpp>

// ROS messages
#include <talos_wbc_controller/FollowContactJointTrajectoryAction.h>
#include <talos_wbc_controller/JointContactTrajectoryContacts.h>
#include <talos_wbc_controller/JointContactTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

// ros_controls
#include <realtime_tools/realtime_server_goal_handle.h>

// Project
#include <joint_trajectory_controller/joint_trajectory_msg_utils.h>
#include <talos_wbc_controller/contact_joint_trajectory_segment.hpp>
#include <talos_wbc_controller/contact_segment.hpp>

namespace joint_trajectory_controller
{

namespace internal
{
/**
 * \return The map between \p t1 indices (implicitly encoded in return vector indices) to \t2 indices.
 * If \p t1 is <tt>"{C, B}"</tt> and \p t2 is <tt>"{A, B, C, D}"</tt>, the associated mapping vector is
 * <tt>"{2, 1}"</tt>.
 */
namespace contact {
template <class T>
inline std::vector<unsigned int> mapping(const T& t1, const T& t2)
{
  typedef unsigned int SizeType;
  
  // t1 must be a subset of t2
  if (t1.size() > t2.size()) {return std::vector<SizeType>();}

  std::vector<SizeType> mapping_vector(t1.size()); // Return value
  for (typename T::const_iterator t1_it = t1.begin(); t1_it != t1.end(); ++t1_it)
  {
    typename T::const_iterator t2_it = std::find(t2.begin(), t2.end(), *t1_it);
    if (t2.end() == t2_it) {return std::vector<SizeType>();}
    else
    {
      const SizeType t1_dist = std::distance(t1.begin(), t1_it);
      const SizeType t2_dist = std::distance(t2.begin(), t2_it);
      mapping_vector[t1_dist] = t2_dist;
    }
  }
  return mapping_vector;
}

}

} // namespace

/**
 * \brief Update data in \p tols from data in \p goal.
 *
 * \param[in] goal Action goal data containing tolerance values \p tols will be updated with.
 * \param[in] joint_names Names of joints in \p tols, with the same ordering.
 * \param[out] tols Tolerances values to update.
 */
template<class Scalar>
void updateSegmentTolerances(const talos_wbc_controller::FollowContactJointTrajectoryGoal& goal,
                             const std::vector<std::string>&                joint_names,
                             SegmentTolerances<Scalar>&                     tols
)
{
  // Preconditions
  assert(joint_names.size() == tols.state_tolerance.size());
  assert(joint_names.size() == tols.goal_state_tolerance.size());

  typedef typename std::vector<std::string>::const_iterator                  StringConstIterator;
  typedef typename std::vector<control_msgs::JointTolerance>::const_iterator TolMsgConstIterator;

  for (StringConstIterator names_it = joint_names.begin(); names_it != joint_names.end(); ++names_it)
  {
    const typename std::vector<std::string>::size_type id = std::distance(joint_names.begin(), names_it);

    // Update path tolerances
    const std::vector<control_msgs::JointTolerance>& state_tol = goal.path_tolerance;
    for(TolMsgConstIterator state_tol_it = state_tol.begin(); state_tol_it != state_tol.end(); ++state_tol_it)
    {
      if (*names_it == state_tol_it->name) {updateStateTolerances(*state_tol_it, tols.state_tolerance[id]);}
    }

    // Update goal state tolerances
    const std::vector<control_msgs::JointTolerance>& g_state_tol = goal.goal_tolerance;
    for(TolMsgConstIterator g_state_tol_it = g_state_tol.begin(); g_state_tol_it != g_state_tol.end(); ++g_state_tol_it)
    {
      if (*names_it == g_state_tol_it->name) {updateStateTolerances(*g_state_tol_it, tols.goal_state_tolerance[id]);}
    }
  }

  // Update goal time tolerance
  const ros::Duration& goal_time_tolerance = goal.goal_time_tolerance;
  if      (goal_time_tolerance < ros::Duration(0.0)) {tols.goal_time_tolerance = 0.0;}
  else if (goal_time_tolerance > ros::Duration(0.0)) {tols.goal_time_tolerance = goal_time_tolerance.toSec();}
}
/**
 * \brief Options used when initializing a joint trajectory from ROS message data.
 * \sa initJointTrajectory
 */
template <class Trajectory, class ContactTrajectory>
struct InitContactJointTrajectoryOptions
{
  typedef realtime_tools::RealtimeServerGoalHandle<talos_wbc_controller::FollowContactJointTrajectoryAction> RealtimeGoalHandle;
  typedef boost::shared_ptr<RealtimeGoalHandle>      RealtimeGoalHandlePtr;
  typedef typename Trajectory::value_type            TrajectoryPerJoint;
  typedef typename TrajectoryPerJoint::value_type    Segment;
  typedef typename ContactTrajectory::value_type     ContactPerLink;
  typedef typename Segment::Scalar                   Scalar;

  InitContactJointTrajectoryOptions()
    : current_trajectory(0),
      current_contact_trajectory(0),
      joint_names(0),
      angle_wraparound(0),
      rt_goal_handle(),
      default_tolerances(0),
      other_time_base(0),
      allow_partial_joints_goal(false)
  {}

  Trajectory*                current_trajectory;
  ContactTrajectory*         current_contact_trajectory;
  std::vector<std::string>*  joint_names;
  std::vector<bool>*         angle_wraparound;
  RealtimeGoalHandlePtr      rt_goal_handle;
  SegmentTolerances<Scalar>* default_tolerances;
  ros::Time*                 other_time_base;
  bool                       allow_partial_joints_goal;
};

namespace contact{
template <class Trajectory>
bool isNotEmpty(typename Trajectory::value_type trajPerJoint)
{
  return !trajPerJoint.empty();
};
}

/**
 * \brief Initialize a joint trajectory from ROS message data.
 *
 * \param msg Trajectory message.
 *
 * \param time Time from which data is to be extracted. All trajectory points in \p msg occurring \b after
 * \p time will be extracted; or put otherwise, all points occurring at a time <b>less or equal</b> than \p time
 * will be discarded. Set this value to zero to process all points in \p msg.
 *
 * \param options Options that change how the trajectory gets initialized.
 *
 * The \ref InitContactJointTrajectoryOptions "options" parameter is optional. The meaning of its different members follows:
 * - \b current_trajectory Currently executed trajectory. Use this parameter if you want to update an existing
 * trajectory with the data in \p msg; that is, keep the useful parts of \p current_trajectory and \p msg.
 * If specified, the output trajectory will not only contain data in \p msg occurring \b after \p time, but will also
 * contain data from \p current_trajectory \b between \p time and the start time of \p msg
 * (which might not be equal to \p time).
 *
 * - \b joint_names Joints expected to be found in \p msg. If specified, this function will return an empty trajectory
 * when \p msg contains joints that differ in number or names to \p joint_names. If \p msg contains the same joints as
 * \p  joint_names, but in a different order, the resulting trajectory will be ordered according to \p joint_names
 * (and not \p msg). If unspecified (empty), no checks will be performed against expected joints, and the resulting
 * trajectory will preserve the joint ordering of \p msg.
 *
 * - \b angle_wraparound Vector of booleans where true values correspond to joints that wrap around (ie. are continuous).
 * If specified, combining \p current_trajectory with \p msg will not result in joints performing multiple turns at the
 * transition. This parameter \b requires \p current_trajectory to also be specified, otherwise it is ignored.
 *
 * - \b rt_goal_handle Action goal handle associated to the new trajectory. If specified, newly added segments will have
 * a pointer to it, and to the trajectory tolerances it contains (if any).
 *
 * - \b default_tolerances Default trajectory tolerances. This option is only used when \p rt_goal_handle is also
 * specified. It contains the default tolernaces to check when executing an action goal. If the action goal specifies
 * tolerances (totally or partially), these values will take precedence over the defaults.
 *
 * - \b other_time_base When initializing a new trajectory, it might be the case that we desire the result expressed in
 * a \b different time base than that contained in \p msg. If specified, the value of this variable should be the
 * equivalent of the \p time parameter, but expressed in the desired time base.
 * If the \p current_trajectory option is also specified, it must be expressed in \p other_time_base.
 * The typical usecase for this variable is when the \p current_trajectory option is specified, and contains data in
 * a different time base (eg. monotonically increasing) than \p msg (eg. system-clock synchronized).
 *
 * \return Trajectory container.
 *
 * \tparam Trajectory Trajectory type. Should be a \e sequence container \e sorted by segment start time.
 * Additionally, the contained segment type must implement a constructor with the following signature:
 * \code
 * Segment(const ros::Time&                             traj_start_time,
 *         const trajectory_msgs::JointTrajectoryPoint& start_point,
 *         const trajectory_msgs::JointTrajectoryPoint& end_point,
 *         const std::vector<Scalar>&                   position_offset)
 * \endcode
 * The following function must also be defined to properly handle continuous joints:
 * \code
 * std::vector<Scalar> wraparoundOffset(const typename Segment::State&  prev_state,
 *                                      const typename Segment::State&  next_state,
 *                                      const std::vector<bool>&        angle_wraparound)
 * \endcode
 *
 * \note This function does not throw any exceptions by itself, but the segment constructor might.
 * In such a case, this method should be wrapped inside a \p try block.
 */
// TODO: Return useful bits of current trajectory if input msg is useless?
template <class Trajectory, class ContactTrajectory>
void
initContactJointTrajectory(const talos_wbc_controller::JointContactTrajectory& msg,
			   const ros::Time& time,
			   Trajectory& out_traj,
			   ContactTrajectory& out_contact_traj,
			   const InitContactJointTrajectoryOptions<Trajectory, ContactTrajectory>& options =
			   InitContactJointTrajectoryOptions<Trajectory, ContactTrajectory>())
{
  typedef typename Trajectory::value_type TrajectoryPerJoint;
  typedef typename TrajectoryPerJoint::value_type Segment;
  typedef typename Segment::Scalar Scalar;
  typedef typename TrajectoryPerJoint::const_iterator TrajIter;
  typedef typename ContactTrajectory::value_type     ContactPerLink;

  const unsigned int n_joints = msg.trajectory.joint_names.size();

  const ros::Time msg_start_time = internal::startTime(msg.trajectory, time); // Message start time

  ROS_DEBUG_STREAM("Figuring out new trajectory starting at time "
                   << std::fixed << std::setprecision(3) << msg_start_time.toSec());

  // Empty trajectory
  if (msg.trajectory.points.empty())
  {
    ROS_DEBUG("Trajectory message contains empty trajectory. Nothing to convert.");
    out_traj = Trajectory();
    out_contact_traj = ContactTrajectory();
    return;
  }

  // Non strictly-monotonic waypoints
  if (!isTimeStrictlyIncreasing(msg.trajectory))
  {
    ROS_ERROR("Trajectory message contains waypoints that are not strictly increasing in time.");
    out_traj = Trajectory();
    out_contact_traj = ContactTrajectory();
    return;
  }

  // Validate options
  const bool has_current_trajectory = options.current_trajectory && !options.current_trajectory->empty();
  const bool has_joint_names        = options.joint_names        && !options.joint_names->empty();
  const bool has_angle_wraparound   = options.angle_wraparound   && !options.angle_wraparound->empty();
  const bool has_rt_goal_handle     = options.rt_goal_handle.get();
  const bool has_other_time_base    = options.other_time_base;
  const bool has_default_tolerances = options.default_tolerances;

  if (!has_current_trajectory && has_angle_wraparound)
  {
    ROS_WARN("Vector specifying whether joints wrap around will not be used because no current trajectory was given.");
  }

  // Compute trajectory start time and data extraction time associated to the 'other' time base, if it applies
  // The o_ prefix indicates that time values are represented in this 'other' time base.
  ros::Time o_time;
  ros::Time o_msg_start_time;
  if (has_other_time_base)
  {
    ros::Duration msg_start_duration = msg_start_time - time;
    o_time = *options.other_time_base;
    o_msg_start_time = o_time + msg_start_duration;
    ROS_DEBUG_STREAM("Using alternate time base. In it, the new trajectory starts at time "
                     << std::fixed << std::setprecision(3) << o_msg_start_time.toSec());
  }
  else
  {
    o_time = time;
    o_msg_start_time = msg_start_time;
  }

  const std::vector<std::string> joint_names = has_joint_names ? *(options.joint_names) : msg.trajectory.joint_names;

  if (has_angle_wraparound)
  {
    // Preconditions
    const unsigned int n_angle_wraparound = options.angle_wraparound->size();
    if (n_angle_wraparound != joint_names.size())
    {
      ROS_ERROR("Cannot create trajectory from message. "
                "Vector specifying whether joints wrap around has an invalid size.");
      out_traj = Trajectory();
      out_contact_traj = ContactTrajectory();
      return;
    }
  }

  // If partial joints goals are not allowed, goal should specify all controller joints
  if (!options.allow_partial_joints_goal)
  {
    if (msg.trajectory.joint_names.size() != joint_names.size())
    {
      ROS_ERROR("Cannot create trajectory from message. It does not contain the expected joints.");
      out_traj = Trajectory();
      out_contact_traj = ContactTrajectory();
      return;
    }
  }

  // Mapping vector contains the map between the message joint order and the expected joint order
  // If unspecified, a trivial map is computed
  std::vector<unsigned int> mapping_vector = internal::contact::mapping(msg.trajectory.joint_names,joint_names);

  if (mapping_vector.empty())
  {
    ROS_ERROR("Cannot create trajectory from message. It does not contain the expected joints.");
    out_traj = Trajectory();
    out_contact_traj = ContactTrajectory();
    return;
  }

  // Tolerances to be used in all new segments
  SegmentTolerances<Scalar> tolerances = has_default_tolerances ?
                                         *(options.default_tolerances) : SegmentTolerances<Scalar>(n_joints);

  if (has_rt_goal_handle && options.rt_goal_handle->gh_.getGoal())
  {
    updateSegmentTolerances<Scalar>(*(options.rt_goal_handle->gh_.getGoal()), joint_names, tolerances);
  }

  // Find first point of new trajectory occurring after current time
  // This point is used later on in this function, but is computed here, in advance because if the trajectory message
  // contains a trajectory in the past, we can quickly return without spending additional computational resources
  std::vector<trajectory_msgs::JointTrajectoryPoint>::const_iterator
  msg_it = findPoint(msg.trajectory, time); // Points to last point occurring before current time (NOTE: Using time, not o_time)
  if (msg_it == msg.trajectory.points.end())
  {
    msg_it = msg.trajectory.points.begin();  // Entire trajectory is after current time
  }
  else
  {
    ++msg_it;                     // Points to first point after current time OR sequence end
    if (msg_it == msg.trajectory.points.end())
    {
      ros::Duration last_point_dur = time - (msg_start_time + (--msg_it)->time_from_start);
      ROS_WARN_STREAM("Dropping all " << msg.trajectory.points.size() <<
                      " trajectory point(s), as they occur before the current time.\n" <<
                      "Last point is " << std::fixed << std::setprecision(3) << last_point_dur.toSec() <<
                      "s in the past.");
      out_traj = Trajectory();
      out_contact_traj = ContactTrajectory();
      return;
    } else {
      ros::Duration next_point_dur = msg_start_time + msg_it->time_from_start - time;
      ROS_WARN_STREAM("Dropping first " << std::distance(msg.trajectory.points.begin(), msg_it) <<
                      " trajectory point(s) out of " << msg.trajectory.points.size() <<
                      ", as they occur before the current time.\n" <<
                      "First valid point will be reached in " << std::fixed << std::setprecision(3) <<
                      next_point_dur.toSec() << "s.");
    }
  }

  // Find the first contacts of the new trajectory occurring after current time
  std::vector<talos_wbc_controller::JointContactTrajectoryContacts>::const_iterator contact_msg_it =
    msg.contacts.begin() + std::distance(msg.trajectory.points.begin(), msg_it);

  // Initialize result trajectory: combination of:
  // - Useful segments of currently followed trajectory
  // - Useful segments of new trajectory (contained in ROS message)
  Trajectory result_traj; // Currently empty
  ContactTrajectory result_contact_traj;

  // Set active goal to segments after current time
  if (has_current_trajectory)
  {
    result_traj = *(options.current_trajectory);
    result_contact_traj = *(options.current_contact_trajectory);

    //Iterate to all segments after current time to set the new goal handler
    for (unsigned int joint_id=0; joint_id < joint_names.size();joint_id++)
    {
      const TrajectoryPerJoint& curr_joint_traj = result_traj[joint_id];
      TrajIter active_seg = findSegment(curr_joint_traj, o_time.toSec());   // Currently active segment

      while (std::distance(active_seg, curr_joint_traj.end())>=1)
      {
        (result_traj[joint_id])[std::distance(curr_joint_traj.begin(),active_seg)].setGoalHandle(options.rt_goal_handle);
        ++active_seg;
      }
    }
  }
  else {
    result_traj.resize(joint_names.size());
    result_contact_traj.resize(msg.contact_link_names.size());
  }

  // If there are a different number of possible contacts, we need to resize the trajectory
  if (msg.contact_link_names.size() !=
      options.current_contact_trajectory->size()) {

    options.current_contact_trajectory->resize(msg.contact_link_names.size());

    // Moreover, if now there are more contact links than before, we need to fill the past contacts
    if (msg.contact_link_names.size() > options.current_contact_trajectory->size()) {
      for (size_t j = options.current_contact_trajectory->size();
	   j < msg.contact_link_names.size(); ++j) {
	// Dirty but werks
        options.current_contact_trajectory->at(j).resize(options.current_contact_trajectory->at(0).size());
      }
    }
  }

  // Iterate through the contact links in the message and retrieve them
  for (unsigned int msg_link_it = 0; msg_link_it < msg.contact_link_names.size(); msg_link_it++) {
    std::vector<talos_wbc_controller::JointContactTrajectoryContacts>::const_iterator it = contact_msg_it;

    // Bridge current trajectory to new one (the bridge has the same value as the previous trajectory)
    if (has_current_trajectory) {
      typedef typename ContactPerLink::const_iterator ContactPerLinkIterator;
      const ContactPerLink& curr_contact_traj = (*options.current_contact_trajectory)[msg_link_it];

      // Get the last time and state that will be executed from the current trajectory
      const typename Segment::Time last_curr_time = std::max(o_msg_start_time.toSec(), o_time.toSec());
      // Get the current contact segment
      auto isBeforeContactSegment =
	  [](const ContactSegment::Time &time,
	     const ContactSegment &segment) {
	    return time < segment.getTime();
          };
      ContactPerLinkIterator contact_first = curr_contact_traj.begin();
      ContactPerLinkIterator contact_last = curr_contact_traj.end();
      ContactPerLinkIterator curr_contact_segment =
	  (contact_first == contact_last ||
	   isBeforeContactSegment(last_curr_time, *contact_first))
	      ? contact_last
	      : --std::upper_bound(contact_first, contact_last, last_curr_time,
                                   isBeforeContactSegment);
      // The next contact state remains equal to the actual contact state
      ContactSegment bridge = *curr_contact_segment;
      // Append bridge 
      result_contact_traj.at(msg_link_it).push_back(bridge);
    }

    // TODO: Append the rest of the contact state (last contact is not
    // appended because is the right end of the last segment)
    
  }

  // Iterate through the joints that are in the message, in the order of the
  // mapping vector for (unsigned int joint_id=0; joint_id <
  // joint_names.size();joint_id++)
  for (unsigned int msg_joint_it=0; msg_joint_it < mapping_vector.size();msg_joint_it++)
  {
    std::vector<trajectory_msgs::JointTrajectoryPoint>::const_iterator it = msg_it;
    if (!isValid(*it, it->positions.size()))
      throw(std::invalid_argument("Size mismatch in trajectory point position, velocity or acceleration data."));

    TrajectoryPerJoint result_traj_per_joint; // Currently empty
    unsigned int joint_id = mapping_vector[msg_joint_it];

    // Initialize offsets due to wrapping joints to zero
    std::vector<Scalar> position_offset(1, 0.0);

    //Initialize segment tolerance per joint
    SegmentTolerancesPerJoint<Scalar> tolerances_per_joint;
    tolerances_per_joint.state_tolerance = tolerances.state_tolerance[joint_id];
    tolerances_per_joint.goal_state_tolerance = tolerances.goal_state_tolerance[joint_id];
    tolerances_per_joint.goal_time_tolerance = tolerances.goal_time_tolerance;

    // Bridge current trajectory to new one
    if (has_current_trajectory)
    {
      const TrajectoryPerJoint& curr_joint_traj = (*options.current_trajectory)[joint_id];

      // Get the last time and state that will be executed from the current trajectory
      const typename Segment::Time last_curr_time = std::max(o_msg_start_time.toSec(), o_time.toSec()); // Important!
      typename Segment::State last_curr_state;
      sample(curr_joint_traj, last_curr_time, last_curr_state);

      // Get the first time and state that will be executed from the new trajectory
      trajectory_msgs::JointTrajectoryPoint point_per_joint;
      if (!it->positions.empty())     {point_per_joint.positions.resize(1, it->positions[msg_joint_it]);}
      if (!it->velocities.empty())    {point_per_joint.velocities.resize(1, it->velocities[msg_joint_it]);}
      if (!it->accelerations.empty()) {point_per_joint.accelerations.resize(1, it->accelerations[msg_joint_it]);}
      point_per_joint.time_from_start = it->time_from_start;

      const typename Segment::Time first_new_time = o_msg_start_time.toSec() + (it->time_from_start).toSec();
      typename Segment::State first_new_state(point_per_joint); // Here offsets are not yet applied

      // Compute offsets due to wrapping joints
      if (has_angle_wraparound)
      {
        position_offset[0] = contact::wraparoundJointOffset(last_curr_state.position[0],
							    first_new_state.position[0],
							    (*options.angle_wraparound)[joint_id]);
      }

      // Apply offset to first state that will be executed from the new trajectory
      first_new_state = typename Segment::State(point_per_joint, position_offset); // Now offsets are applied

      // Add useful segments of current trajectory to result
      {
        TrajIter first = findSegment(curr_joint_traj, o_time.toSec());   // Currently active segment
        TrajIter last  = findSegment(curr_joint_traj, last_curr_time); // Segment active when new trajectory starts
        if (first == curr_joint_traj.end() || last == curr_joint_traj.end())
        {
          ROS_ERROR("Unexpected error: Could not find segments in current trajectory. Please contact the package maintainer.");
	  out_traj = Trajectory();
	  out_contact_traj = ContactTrajectory();
	  return;
        }
        result_traj_per_joint.insert(result_traj_per_joint.begin(), first, ++last); // Range [first,last) will still be executed
      }

      // Add segment bridging current and new trajectories to result
      Segment bridge_seg(last_curr_time, last_curr_state,
                         first_new_time, first_new_state);
      bridge_seg.setGoalHandle(options.rt_goal_handle);
      if (has_rt_goal_handle) {bridge_seg.setTolerances(tolerances_per_joint);}
      result_traj_per_joint.push_back(bridge_seg);
    }

    // Constants used in log statement at the end
    const unsigned int num_old_segments = result_traj_per_joint.size() -1;
    const unsigned int num_new_segments = std::distance(it, msg.trajectory.points.end()) -1;

    // Add useful segments of new trajectory to result
    // - Construct all trajectory segments occurring after current time
    // - As long as there remain two trajectory points we can construct the next trajectory segment
    while (std::distance(it, msg.trajectory.points.end()) >= 2)
    {
      std::vector<trajectory_msgs::JointTrajectoryPoint>::const_iterator next_it = it; ++next_it;

      trajectory_msgs::JointTrajectoryPoint it_point_per_joint, next_it_point_per_joint;

      if (!isValid(*it, it->positions.size()))
            throw(std::invalid_argument("Size mismatch in trajectory point position, velocity or acceleration data."));
      if (!it->positions.empty())     {it_point_per_joint.positions.resize(1, it->positions[msg_joint_it]);}
      if (!it->velocities.empty())    {it_point_per_joint.velocities.resize(1, it->velocities[msg_joint_it]);}
      if (!it->accelerations.empty()) {it_point_per_joint.accelerations.resize(1, it->accelerations[msg_joint_it]);}
      it_point_per_joint.time_from_start = it->time_from_start;

      if (!isValid(*next_it, next_it->positions.size()))
            throw(std::invalid_argument("Size mismatch in trajectory point position, velocity or acceleration data."));
      if (!next_it->positions.empty()) {next_it_point_per_joint.positions.resize(1, next_it->positions[msg_joint_it]);}
      if (!next_it->velocities.empty()) {next_it_point_per_joint.velocities.resize(1, next_it->velocities[msg_joint_it]);}
      if (!next_it->accelerations.empty()) {next_it_point_per_joint.accelerations.resize(1, next_it->accelerations[msg_joint_it]);}
      next_it_point_per_joint.time_from_start = next_it->time_from_start;

      Segment segment(o_msg_start_time, it_point_per_joint, next_it_point_per_joint, position_offset);
      segment.setGoalHandle(options.rt_goal_handle);
      if (has_rt_goal_handle) {segment.setTolerances(tolerances_per_joint);}
      result_traj_per_joint.push_back(segment);
      ++it;
    }

    // Useful debug info
    std::stringstream log_str;
    log_str << "Trajectory of joint " << joint_names[joint_id] << "has " << result_traj_per_joint.size() << " segments";
    if (has_current_trajectory)
    {
      log_str << ":";
      log_str << "\n- " << num_old_segments << " segment(s) will still be executed from previous trajectory.";
      log_str << "\n- 1 segment added for transitioning between the current trajectory and first point of the input message.";
      if (num_new_segments > 0) {log_str << "\n- " << num_new_segments << " new segments (" << (num_new_segments + 1) <<
                                 " points) taken from the input trajectory.";}
    }
    else {log_str << ".";}
    ROS_DEBUG_STREAM(log_str.str());

    if (result_traj_per_joint.size() > 0)
      result_traj[joint_id] = result_traj_per_joint;
  }

  // If the trajectory for all joints is empty, empty the trajectory vector
  typename Trajectory::const_iterator trajIter =
    std::find_if(result_traj.begin(), result_traj.end(), contact::isNotEmpty<Trajectory>);

  if (trajIter == result_traj.end())
  {
    result_traj.clear();
    result_contact_traj.clear();
  }

  out_traj = result_traj;
  out_contact_traj = result_contact_traj;
  return;
}

} // namespace

#endif // header guard
