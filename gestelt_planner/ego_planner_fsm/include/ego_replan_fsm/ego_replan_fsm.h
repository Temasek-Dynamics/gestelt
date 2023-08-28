#ifndef _EGO_PLANNER_FSM_H_
#define _EGO_PLANNER_FSM_H_

#include <algorithm>

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>

#include <traj_utils/PolyTraj.h>
#include <traj_utils/MINCOTraj.h>

#include <ego_planner_manager/ego_planner_manager.h>

#include <swarm_benchmark/timebenchmark.h>

#include <trajectory_server_msgs/Waypoints.h>
#include <trajectory_server_msgs/TimeBenchmark.h>

using std::vector;

namespace ego_planner
{

class Waypoint
{
public:
  Waypoint(){
    reset();
  }

  void reset(){
    cur_wp_idx_ = 0;
    waypoints.clear();
  }
  
  bool addWP(const Eigen::Vector3d& wp){
    if (!WPWithinBounds(wp)){
      ROS_ERROR("Adding Out of bound Waypoints");
      return false;
    }
    waypoints.push_back(wp);
    return true;
  }

  /* Getter methods */

  Eigen::Vector3d getWP(const int& idx){
    if (!idxWithinBounds(idx)){
      ROS_ERROR("Retrieved wp beyond size of waypoints");
      throw "Retrieved wp beyond size of waypoints";
    }
    return waypoints[idx];
  }

  Eigen::Vector3d getNextWP(){
    if (!idxWithinBounds(cur_wp_idx_)){
      ROS_ERROR("Retrieved next wp beyond size of waypoints");
      throw "Retrieved next wp beyond size of waypoints";
    }
    return waypoints[cur_wp_idx_];
  }

  Eigen::Vector3d getStartWP() const {
    return next_start_wp_;
  }

  Eigen::Vector3d getLast() const {
    if (waypoints.empty()){
      ROS_ERROR("Trying to get last wp from empty waypoint list");
      throw "Trying to get last wp from empty waypoint list";
    }
    return waypoints.back();
  }

  size_t getSize() const {
    return waypoints.size();
  }

  int getCurIdx() const {
    return cur_wp_idx_;
  }

  /* Setter methods */

  void setStartWP(const Eigen::Vector3d& wp) {
    next_start_wp_ = wp;
  }

  void iterateNextWP() {
    if (!idxWithinBounds(cur_wp_idx_ + 1)){
      ROS_ERROR("Iterating index beyond size of waypoints");
      throw "Iterating index beyond size of waypoints";
    }
    else {
      cur_wp_idx_++;
    }
  }

  /* Checking methods */

  bool isFinalWP() {
    if (cur_wp_idx_ == waypoints.size() - 1){
      return true;
    }
    return false;
  }

  /**
   * @brief Iterate to next waypoint
   * 
   */
  // Eigen::Vector3d pop() {
  //   if (!idxWithinBounds(cur_wp_idx_)){
  //     throw "Popping beyond size of waypoints";
  //   }
  //   return waypoints[cur_wp_idx_];
  // }

private:
  bool idxWithinBounds(const int& idx){
    return (
      idx >= 0 
      && idx < waypoints.size()
    );
  }

  bool WPWithinBounds(Eigen::Vector3d wp){
    return (
      wp(2) > 0.05 
    );
  }
  
  Eigen::Vector3d next_start_wp_;

  std::vector<Eigen::Vector3d> waypoints;
  size_t cur_wp_idx_;

  std::queue<Eigen::Vector3d> wp_queue_;
};

template<typename ... Args>
std::string string_format( const std::string& format, Args ... args )
{
  int size_s = std::snprintf( nullptr, 0, format.c_str(), args ... ) + 1; // Extra space for '\0'
  if( size_s <= 0 ){ throw std::runtime_error( "Error during formatting." ); }
  auto size = static_cast<size_t>( size_s );
  std::unique_ptr<char[]> buf( new char[ size ] );
  std::snprintf( buf.get(), size, format.c_str(), args ... );
  return std::string( buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
}

class EGOReplanFSM
{
public:
  EGOReplanFSM() {}
  ~EGOReplanFSM() {}

  void init(ros::NodeHandle &nh);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  /* State machine states */
  enum ServerState
  {
    INIT,
    READY,
    PLAN_NEW_GLOBAL_TRAJ,
    PLAN_LOCAL_TRAJ,
    EXEC_TRAJ,
    EMERGENCY_STOP,
    PLAN_GLOBAL_TRAJ
  };

  /* State machine events */
  enum ServerEvent
  {
    EMPTY_E,             // 1
    READY_E,      // 2
    PLAN_NEW_GLOBAL_TRAJ_E,     // 3
    PLAN_LOCAL_TRAJ_E,      // 4
    EXEC_TRAJ_E,        // 5
    EMERGENCY_STOP_E,   // 6
    PLAN_GLOBAL_TRAJ_E // 7
  };

  /** @brief StateToString interprets the input server state **/
  const std::string StateToString(ServerState state)
  {
      switch (state)
      {
          case ServerState::INIT:   return "INIT";
          case ServerState::READY:   return "READY";
          case ServerState::PLAN_NEW_GLOBAL_TRAJ:   return "PLAN_NEW_GLOBAL_TRAJ";
          case ServerState::PLAN_LOCAL_TRAJ: return "PLAN_LOCAL_TRAJ";
          case ServerState::EXEC_TRAJ:   return "EXEC_TRAJ";
          case ServerState::EMERGENCY_STOP:   return "EMERGENCY_STOP";
          case ServerState::PLAN_GLOBAL_TRAJ:   return "PLAN_GLOBAL_TRAJ";
          default:      return "[Unknown State]";
      }
  }

  /** @brief StateToString interprets the input server event **/
  const std::string EventToString(ServerEvent event)
  {
      switch (event)
      {
          case ServerEvent::READY_E:   return "READY_E";
          case ServerEvent::PLAN_NEW_GLOBAL_TRAJ_E: return "PLAN_NEW_GLOBAL_TRAJ_E";
          case ServerEvent::PLAN_LOCAL_TRAJ_E:   return "PLAN_LOCAL_TRAJ_E";
          case ServerEvent::EXEC_TRAJ_E:   return "EXEC_TRAJ_E";
          case ServerEvent::EMERGENCY_STOP_E:   return "EMERGENCY_STOP_E";
          case ServerEvent::PLAN_GLOBAL_TRAJ_E:   return "PLAN_GLOBAL_TRAJ_E";
          case ServerEvent::EMPTY_E:   return "EMPTY_E";
          default:      return "[Unknown Event]";
      }
  }

  enum TARGET_TYPE
  {
    MANUAL_TARGET = 1,
    PRESET_TARGET = 2,
    REFENCE_PATH = 3 //TODO Remove
  };
  
  std::string node_name_{"EgoPlannerFSM"};
  int drone_id_;

  /* parameters */
  bool apply_frame_origin_offset_; // If true, applies a transformation from UAV origin frame to world
  geometry_msgs::Pose uav_origin_to_world_tf_; // Frame transformation from origin to world frame
  geometry_msgs::Pose world_to_uav_origin_tf_; // Frame transformation from world frame to origin

  int target_type_; // If value is 1, the goal is manually defined via a subscribed topic, else if 2, the goal is defined via pre-defined waypoints 

  double min_replan_dist_; // Min distance to replan
  double replan_time_thresh_; // Timeout for replanning to occur
  
  // Waypoints
  Waypoint waypoints_; // Waypoint handler object
  Eigen::Vector3d formation_pos_; // Position of drone relative to formation origin

  // Max planning distance for the local target
  double planning_horizen_;
  double emergency_time_; // Threshold time to deal with potential collision
  bool enable_fail_safe_; 
  bool flag_escape_emergency_; // Used to prevent repeated execution of estop functions
  bool potential_agent_to_agent_collision_{false}; // Indicates potential collision with other agents in the near future

  // Indicates that all agents within a swarm have a provided trajectory
  bool have_recv_pre_agent_; 
  bool have_trigger_, have_target_, have_odom_, have_new_target_, touch_goal_;
  ServerState current_state_{ServerState::INIT};
  ServerEvent server_event_{ServerEvent::EMPTY_E};
  int continously_called_times_{0};

  Eigen::Vector3d start_pt_, start_vel_, start_acc_;   // start state
  Eigen::Vector3d end_pt_;                             // goal state
  Eigen::Vector3d local_target_pt_, local_target_vel_; // local target state
  Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_;     // odometry state

  // Count number of FSM exec iterations
  int fsm_itr_num{0};

  /* ROS utils */
  ros::NodeHandle node_;

  // Timer to execute FSM callback
  ros::Timer pub_state_timer_;
  ros::Timer tick_state_timer_;
  ros::Timer exec_state_timer_;

  // Subscribers and publishers
  ros::Subscriber waypoint_sub_, odom_sub_, trigger_sub_, broadcast_ploytraj_sub_, mandatory_stop_sub_;
  ros::Subscriber waypoints_sub_;

  ros::Publisher poly_traj_pub_, broadcast_ploytraj_pub_, heartbeat_pub_, ground_height_pub_;

  ros::Publisher state_pub_;
  ros::Publisher time_benchmark_pub_;

  /* planning utils */
  EGOPlannerManager::Ptr planner_manager_;
  PlanningVisualization::Ptr visualization_;
  std::shared_ptr<TimeBenchmark> time_benchmark_; // Measures and stores CPU/Wall runtime

private: 

  /* Timer callbacks */

  /**
   * @brief Timer callback to publish current FSM State and benchmark
   * @param e 
   */
  void pubStateTimerCB(const ros::TimerEvent &e);

  /**
   * @brief Timer callback to tick State Machine
   * This callback should only ever handle transitions between states,
   * It should not attempt to call any execution function, this 
   * should be left to the execStateTimerCB callback.
   * @param e 
   */
  void tickStateTimerCB(const ros::TimerEvent &e);

  /**
   * @brief Timer callback to execute looping commands depending on the current state 
   * 
   * @param e 
   */
  void execStateTimerCB(const ros::TimerEvent &e);

  /**
   * @brief Emergency stop the drone
   * 
   * @param stop_pos 
   * @return true 
   * @return false 
   */
  bool callEmergencyStop(Eigen::Vector3d stop_pos);

  /**
   * @brief Plans a path using planner_manager_ methods
   * 
   * @param flag_use_poly_init Intialize new polynomial if true
   * @param flag_randomPolyTraj Randomize inner points if true
   * @return true 
   * @return false 
   */
  bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj);

  /**
   * @brief Used when no local trajectory is present, calls callReboundReplan
   * 
   * @param trial_times 
   * @return true 
   * @return false 
   */
  bool planFromGlobalTraj(const int trial_times = 1);

  /**
   * @brief Used when a local trajectory has already being planned, calls callReboundReplan
   * 
   * @param trial_times 
   * @return true 
   * @return false 
   */
  bool planFromLocalTraj(const int trial_times = 1);

  /**
   * @brief Plan from current odom position to next waypoint given the direction vector of next and prev wp
   * This is done by calling planGlobalTrajWaypoints
   * And publishing this to "global_list"
   * 
   * @param next_wp 
   * @param previous_wp 
   */
  void planNextWaypoint(const Eigen::Vector3d previous_wp, const Eigen::Vector3d next_wp);

  /**
   * @brief Check if goal is reached and replanning is required
   * 
   * @return std::pair<bool,bool> 
   */
  std::pair<bool,bool> isGoalReachedAndReplanNeeded();

  /**
   * @brief Perform the following checks
   * 1. Ground height
   * 2. Sensor data timeout
   * 3. From local trajectory, clearance between agent trajectories 
   * 
   * @return ServerEvent 
   */
  ServerEvent safetyChecks();

  bool checkSensorTimeout();
  bool checkGroundHeight();
  bool checkTrajectoryClearance();

  /* Subscriber callbacks */

  /**
   * @brief Individual waypoint callback
   * 
   * @param msg 
   */
  void waypointCallback(const geometry_msgs::PoseStampedPtr &msg);

  /**
   * @brief Multiple waypoints callback
   * 
   * @param msg 
   */
  void waypointsCB(const trajectory_server_msgs::WaypointsPtr &msg);

  void mandatoryStopCallback(const std_msgs::Empty &msg);
  void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
  void triggerCallback(const geometry_msgs::PoseStampedPtr &msg);
  void RecvBroadcastMINCOTrajCallback(const traj_utils::MINCOTrajConstPtr &msg);
  
  /* Helper methods */
  void polyTraj2ROSMsg(traj_utils::PolyTraj &poly_msg, traj_utils::MINCOTraj &MINCO_msg);
  bool measureGroundHeight(double &height);

  // Transform the trajectory from UAV frame to world frame
  void transformMINCOTrajectoryToWorld(traj_utils::MINCOTraj & MINCO_msg){

    // Account for offset from UAV origin to world frame
    for (int i = 0; i < MINCO_msg.duration.size() - 1; i++)
    {
      MINCO_msg.inner_x[i] += uav_origin_to_world_tf_.position.x;
      MINCO_msg.inner_y[i] += uav_origin_to_world_tf_.position.y;
      MINCO_msg.inner_z[i] += uav_origin_to_world_tf_.position.z;
    }
    MINCO_msg.start_p[0] += uav_origin_to_world_tf_.position.x;
    MINCO_msg.start_p[1] += uav_origin_to_world_tf_.position.y;
    MINCO_msg.start_p[2] += uav_origin_to_world_tf_.position.z;

    MINCO_msg.end_p[0] += uav_origin_to_world_tf_.position.x;
    MINCO_msg.end_p[1] += uav_origin_to_world_tf_.position.y;
    MINCO_msg.end_p[2] += uav_origin_to_world_tf_.position.z;
  }

  // Transform the trajectory from world frame to UAV frame
  void transformMINCOTrajectoryToUAVOrigin(traj_utils::MINCOTraj & MINCO_msg){
    
    // Account for offset from UAV origin to world frame
    for (int i = 0; i < MINCO_msg.duration.size() - 1; i++)
    {
      MINCO_msg.inner_x[i] += world_to_uav_origin_tf_.position.x;
      MINCO_msg.inner_y[i] += world_to_uav_origin_tf_.position.y;
      MINCO_msg.inner_z[i] += world_to_uav_origin_tf_.position.z;
    }
    MINCO_msg.start_p[0] += world_to_uav_origin_tf_.position.x;
    MINCO_msg.start_p[1] += world_to_uav_origin_tf_.position.y;
    MINCO_msg.start_p[2] += world_to_uav_origin_tf_.position.z;

    MINCO_msg.end_p[0] += world_to_uav_origin_tf_.position.x;
    MINCO_msg.end_p[1] += world_to_uav_origin_tf_.position.y;
    MINCO_msg.end_p[2] += world_to_uav_origin_tf_.position.z;
  }
  
  /* State Machine handling methods */
  
  // TODO: Refactor this
  std::pair<int, EGOReplanFSM::ServerState> timesOfConsecutiveStateCalls();

  void printFSMExecState();

  ServerState getServerState(){
    return current_state_;
  }

  void setServerState(ServerState des_state){
    // logInfo(string_format("Transitioning server state: %s -> %s", 
    //   StateToString(getServerState()).c_str(), StateToString(des_state).c_str()));

    if (des_state == getServerState())
      continously_called_times_++;
    else
      continously_called_times_ = 1;

    current_state_ = des_state;
  }

  void setServerEvent(ServerEvent event)
  {
    // logInfo(string_format("Set server event: %s", EventToString(event).c_str()));
    server_event_ = event;
  }

  ServerEvent getServerEvent()
  {
    // logInfo(string_format("Retrieved server event: %s", EventToString(server_event_).c_str()));
    ServerEvent event = server_event_;
    server_event_ = ServerEvent::EMPTY_E; // Reset to empty

    return event;
  }

private:

  /* Logging functions */
  void logInfo(const std::string& str){
    ROS_INFO_NAMED(node_name_, "UAV_%i: %s", 
      planner_manager_->pp_.drone_id, str.c_str());
  }

  void logWarn(const std::string& str){
    ROS_WARN_NAMED(node_name_, "UAV_%i: %s", 
      planner_manager_->pp_.drone_id, str.c_str());
  }

  void logError(const std::string& str){
    ROS_WARN_NAMED(node_name_, "UAV_%i: %s", 
      planner_manager_->pp_.drone_id, str.c_str());
  }

  void logFatal(const std::string& str){
    ROS_FATAL_NAMED(node_name_, "UAV_%i: %s", 
      planner_manager_->pp_.drone_id, str.c_str());
  }

  void logInfoThrottled(const std::string& str, double period){
    ROS_INFO_THROTTLE_NAMED(period, node_name_, "UAV_%i: %s", 
      planner_manager_->pp_.drone_id, str.c_str());
  }

  void logWarnThrottled(const std::string& str, double period){
    ROS_WARN_THROTTLE_NAMED(period, node_name_, "UAV_%i: %s", 
      planner_manager_->pp_.drone_id, str.c_str());
  }

  void logErrorThrottled(const std::string& str, double period){
    ROS_ERROR_THROTTLE_NAMED(period, node_name_, "UAV_%i: %s", 
      planner_manager_->pp_.drone_id, str.c_str());
  }

  void logFatalThrottled(const std::string& str, double period){
    ROS_FATAL_THROTTLE_NAMED(period, node_name_, "UAV_%i: %s", 
      planner_manager_->pp_.drone_id, str.c_str());
  }

};

} // namespace ego_planner

#endif // _EGO_PLANNER_FSM_H_