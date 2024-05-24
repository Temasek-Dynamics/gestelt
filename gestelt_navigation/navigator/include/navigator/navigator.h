#ifndef _NAVIGATOR_H_
#define _NAVIGATOR_H_

#include <navigator/navigator_helper.h>

#include <algorithm>
#include <unordered_map>

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>

/* Planning messages */
#include <gestelt_msgs/Command.h>
#include <gestelt_msgs/Goals.h>
#include <gestelt_msgs/SphericalSFCTrajectory.h>
#include <traj_utils/PolyTraj.h>
#include <traj_utils/MINCOTraj.h>

/* Visualization */
#include <decomp_ros_msgs/PolyhedronArray.h>
#include <traj_utils/planning_visualization.h>
#include <visualization_msgs/Marker.h>

/* Debugging */
#include <gestelt_debug_msgs/BackEndTrajectoryDebug.h>
#include <gestelt_debug_msgs/SFCTrajectory.h>
#include <gestelt_debug_msgs/SFCSegment.h>
#include <logger/timer.h>

/* Trajectory representation */
#include <traj_utils/plan_container.hpp>
#include <optimizer/poly_traj_utils.hpp>

/* Front-end Planner  */
#include <grid_map/grid_map.h> // Map representation
#include <global_planner/a_star.h>
#include <global_planner/jps_wrapper.h>

/* SFC */
#include <sfc_generation/spherical_sfc.h>
#include <sfc_generation/polytope_sfc.h>

/* Back-end optimization */
#include <optimizer/poly_traj_optimizer.h>
#include <optimizer/polyhedron_sfc_optimizer.h>
#include <optimizer/spherical_sfc_optimizer.h>
#include <ego_planner_fsm/ego_planner_manager.h>

class Navigator
{
enum FrontEndType{
  ASTAR,        
  JPS_AND_DMP
};

enum SFCType{
  SPHERICAL,  
  POLYTOPE    
};

enum BackEndType{
  SSFC, // Spherical safe flight corridor
  EGO,   // ESDF-Free Gradient planner
  POLY // Polytope safe flight corridor
};

public:  
  Navigator() {}
  ~Navigator() {}

  void init(ros::NodeHandle &nh, ros::NodeHandle &pnh);

private: /* Initialization methods */
  void initParams(ros::NodeHandle &nh, ros::NodeHandle &pnh);
  void initSubscribers(ros::NodeHandle &nh, ros::NodeHandle &pnh);
  void initPublishers(ros::NodeHandle &nh, ros::NodeHandle &pnh);

private:  
  /* Timer callbacks */

  /**
   * @brief Timer callback to generate plan
   * @param e 
   */
  void planFrontEndTimerCB(const ros::TimerEvent &e);

  /**
   * @brief Get receding horizon planning goal
   * 
   * @param global_goal 
   * @param start_pos 
   * @param rhp_dist 
   * @param rhp_goal 
   * @return true 
   * @return false 
   */
  bool getRHPGoal(
    const Eigen::Vector3d& global_goal, const Eigen::Vector3d& start_pos, 
    const double& rhp_dist, Eigen::Vector3d& rhp_goal) const;

  /**
   * @brief Timer for checking if
   * 1. Trajectory is safe (w.r.t inter-agent and static obstacle collision)
   * 2. Timeout for odom
   * 
   * @param e 
   */
  void safetyChecksTimerCB(const ros::TimerEvent &e);

  /**
   * @brief Timer for publishing heartbeat to indicate that planner is active
   * 
   * @param e 
   */
  void heartbeatTimerCB(const ros::TimerEvent &e);

  /* Planner methods */

  /**
   * @brief Stop all planning loops 
   * 
   */
  void stopAllPlanning();

  /**
   * @brief Request a back end plan and if successful, save it and broadcast to other agents.
   * 
   * @return true 
   * @return false 
   */
  bool requestBackEndPlan();

  /**
   * @brief Generate a front-end plan with safe flight corridor
   * 
   * @param start_pos 
   * @param goal_pos 
   * @param sfc_traj 
   * @return true 
   * @return false 
   */
  bool generateFrontEndPlan(
    const Eigen::Vector3d& start_pos, const Eigen::Vector3d& goal_pos,
    std::shared_ptr<SSFC::SFCTrajectory> sfc_traj);

  // EDSF Free optimization
  bool EGOOptimize(const Eigen::Matrix3d& startPVA, const Eigen::Matrix3d& endPVA, 
                  poly_traj::MinJerkOpt& mjo_opt);

  // Spherical safe flight corridor optimization
  bool SSFCOptimize(const Eigen::Matrix3d& startPVA, const Eigen::Matrix3d& endPVA, 
                    const double& req_plan_time,
                    std::shared_ptr<SSFC::SFCTrajectory> ssfc_ptr,
                    poly_traj::MinJerkOpt& mjo_opt);

  bool PolySFCOptimize(const Eigen::Matrix3d& startPVA, const Eigen::Matrix3d& endPVA, 
                    const double& req_plan_time,
                    const std::vector<Eigen::Matrix3Xd>& poly_sfc,
                    poly_traj::MinJerkOpt& mjo_opt);

  /* Subscriber callbacks */

  /**
   * @brief Callback for odometry message
   * 
   * @param msg 
   */
  void odometryCB(const nav_msgs::OdometryConstPtr &msg);

  /**
   * @brief Callback for single goal
   * 
   * @param msg 
   */
  void singleGoalCB(const geometry_msgs::PoseStampedConstPtr& msg)
  {
    logInfo(str_fmt("Received debug goal (%f, %f, %f). Note: position.z is set to default of 1.0m", 
          msg->pose.position.x,
          msg->pose.position.y,
          1.0));
    
    Eigen::Vector3d goal_pos = Eigen::Vector3d{msg->pose.position.x, msg->pose.position.y, 1.0};
    
    // 1. One shot planning
    // plan(cur_pos_, goal_pos);

    // 2. Continuous re-planning
    waypoints_.reset();
    waypoints_.addWP(goal_pos);
  }


  /**
   * @brief callback to multiple user-defined goal waypoints 
   * 
   * @param msg 
   */
  void goalsCB(const gestelt_msgs::GoalsConstPtr &msg)
  {
    if (msg->transforms.size() <= 0)
    {
      logError("Received empty waypoints");
      return;
    }
    if (msg->header.frame_id != "world" && msg->header.frame_id != "map" )
    {
      logError("Only waypoint goals in 'world' or 'map' frame are accepted, ignoring waypoints.");
      return;
    }

    std::vector<Eigen::Vector3d> wp_vec;

    for (auto& pos : msg->transforms) {
      // Transform received waypoints from world to UAV origin frame
      wp_vec.push_back(Eigen::Vector3d{pos.translation.x, pos.translation.y, pos.translation.z});
    }

    waypoints_.addMultipleWP(wp_vec);
  }


  /**
   * @brief Callback for swarm MINCO trajectories
   * 
   * @param msg 
   */
  void swarmMincoTrajCB(const traj_utils::MINCOTrajConstPtr &msg);

  /**
   * @brief callback to debug topic for setting starting point of plan
   * 
   * @param msg 
   */
  void debugStartCB(const geometry_msgs::PoseConstPtr &msg)
  {
    logInfo(str_fmt("Received debug start (%f, %f, %f)", 
          msg->position.x,
          msg->position.y,
          msg->position.z));
    cur_pos_ = Eigen::Vector3d{
          msg->position.x,
          msg->position.y,
          msg->position.z};
  }

  /**
   * @brief callback to debug goal topic for setting starting point of plan
   * 
   * @param msg 
   */
  void debugGoalCB(const geometry_msgs::PoseConstPtr &msg)
  {
    logInfo(str_fmt("Received debug goal (%f, %f, %f)", 
          msg->position.x,
          msg->position.y,
          msg->position.z));
    waypoints_.reset();
    waypoints_.addWP(Eigen::Vector3d{
          msg->position.x,
          msg->position.y,
          msg->position.z});
  }

  /**
   * @brief callback to debug goal topic for setting starting point of plan
   * 
   * @param msg 
   */
  void planOnDemandCB(const std_msgs::EmptyConstPtr &msg)
  {
    logInfo(str_fmt("Planning on demand triggered! from (%f, %f, %f) to (%f, %f, %f)",
      cur_pos_(0), cur_pos_(1), cur_pos_(2),
      waypoints_.nextWP()(0), waypoints_.nextWP()(1), waypoints_.nextWP()(2))
    );

    generateFrontEndPlan(cur_pos_, waypoints_.nextWP(), ssfc_);
  }

  /* Checks */

  /**
   * @brief Check if current position is within goal tolerance
   * 
   * @return true 
   * @return false 
   */
  bool isGoalReached(const Eigen::Vector3d& pos, const Eigen::Vector3d& goal)
  {
    return (pos - goal).squaredNorm() < squared_goal_tol_;
  }

  /**
   * @brief Check if time has timed out
   * 
   * @param last_state_time 
   * @return true 
   * @return false 
   */
  bool isTimeout(const double& last_state_time, const double& threshold)
  {
    return (ros::Time::now().toSec() - last_state_time) >= threshold;
  } 

  bool isTrajectorySafe(
    const std::vector<ego_planner::LocalTrajData>& swarm_local_trajs, 
    bool& e_stop, bool& must_replan);

  bool isTrajectoryDynFeasible(ego_planner::LocalTrajData* traj, bool& is_feasible);

  /* Helper methods */

  /**
   * @brief Sample the back end trajectory 
   * 
   * @param local_traj Back end trajectory
   * @param time_samp time at which back end trajectory is sampled
   * @param pos sampled position to assign value to
   * @return true 
   * @return false 
   */
  bool sampleBackEndTrajectory(const ego_planner::LocalTrajData& local_traj, 
    const double& time_samp, Eigen::Vector3d& pos, Eigen::Vector3d& vel, Eigen::Vector3d& acc);

  /**
   * @brief Convert received MINCO msgs from other agents to ego_planner::LocalTrajData type
   * 
   * @param msg 
   * @param traj 
   */
  void mincoMsgToTraj(const traj_utils::MINCOTraj &msg, ego_planner::LocalTrajData& traj);

  /**
   * @brief Convert MJO trajectory to MINCO message
   * 
   * @param mjo 
   * @param traj_start_time 
   * @param poly_msg 
   * @param MINCO_msg 
   */
  void mjoToMsg(const poly_traj::MinJerkOpt& mjo, const double& traj_start_time,
                traj_utils::PolyTraj &poly_msg, traj_utils::MINCOTraj &MINCO_msg);

  /**
   * @brief Get local trajectory from poly_traj::MinJerkOpt
   * 
   * @param mjo 
   * @param traj_start_time 
   * @param num_cp 
   * @param traj_id 
   * @param drone_id 
   * @return ego_planner::LocalTrajData 
   */
  ego_planner::LocalTrajData getLocalTraj(
    const poly_traj::MinJerkOpt& mjo, const double& traj_start_time, 
    const int& num_cp, const int& traj_id, const int& drone_id);

  /**
   * @brief Get optimized segment durations
   * 
   * @param local_traj 
   * @param time_dur 
   * @return true If local trajectory is valid
   * @return false 
   */
  bool getOptSegDur(const ego_planner::LocalTrajData& local_traj, std::vector<double>& time_dur);

  /**
   * @brief Publish command to trajectory server
   * 
   * @param cmd 
   */
  void pubTrajServerCmd(const int& cmd);

private: /* ROS subs, pubs and timers*/
  ros::NodeHandle node_;

  /* Navigator */
  ros::Subscriber odom_sub_;                // Subscriber to drone odometry
  ros::Subscriber goal_sub_;                // Subscriber to user-defined goals
  ros::Subscriber single_goal_sub_;         // Subscriber to single goals
  ros::Publisher traj_server_command_pub_;  // Publish command to trajectory server
  ros::Publisher heartbeat_pub_;            // Publish heartbeat to planner adaptor
  ros::Publisher rhp_goal_pub_;             // Publish receding horizon planning goals

  /* Front end  */
  ros::Subscriber debug_start_sub_; // DEBUG: Subscriber to user-defined start point
  ros::Subscriber debug_goal_sub_; // DEBUG: Subscriber to user-defined goal point
  ros::Subscriber plan_on_demand_sub_; // DEBUG: Subscriber to trigger planning on demand

  /* SFC Publishers */
  // ros::Publisher spherical_ssfc_pub_; // Publish safe flight corridor spherical trajectory

  /* Back-end */
  ros::Publisher debug_traj_pub_; // back-end trajectory for debugging
  ros::Publisher be_traj_pub_; // Publish back end trajectory 
  ros::Publisher swarm_minco_traj_pub_; // publisher for MINCO Trajectory
  ros::Subscriber swarm_minco_traj_sub_; // Subscriber to MINCO trajectory for inter-agent collision avoidance


  /* Visualization for Front End Planner*/
  ros::Publisher front_end_plan_pub_; // Publish front end plan to back-end optimizer
  ros::Publisher front_end_plan_viz_pub_; // Visualization of front end plan
  ros::Publisher closed_list_viz_pub_; // Visualization of closed list

  /* Visualization for SFC*/
  std::unordered_map<std::string, ros::Publisher> front_end_publisher_map_;
  std::unordered_map<std::string, ros::Publisher> sfc_publisher_map_; // Map of publisher topic to ROS publisher
  // ros::Publisher sfc_spherical_viz_pub_; // Visualization of SFC spheres
  // ros::Publisher sfc_p_cand_viz_pub_; // Visualization of SFC sampling points
  // ros::Publisher sfc_dist_viz_pub_; // Visualization of SFC sampling distribution
  // ros::Publisher samp_dir_vec_pub_; // Visualization of SFC sampling direction vectors
  // ros::Publisher sfc_waypoints_viz_pub_; // Visualization of SFC waypoints
  // ros::Publisher intxn_spheres_pub_; // Visualization of intersecting spheres

  /* Debugging */
  ros::Publisher dbg_ssfc_pub_; // Publishers to trajectory inspector for debugging 
  
  /* Timers */
  ros::Timer fe_plan_timer_;           // Front-end planning loop
  ros::Timer safety_checks_timer_;  // Safety checks
  ros::Timer hb_timer_;             // Heartbeat timer

private: /* Planner members */
  /* Mapping */
  std::shared_ptr<GridMap> map_;

  /* Visualization and debugging */
  std::shared_ptr<ego_planner::PlanningVisualization> visualization_; // For publishing visualizations

  /* Planner */
  // std::unique_ptr<AStarPlanner> front_end_planner_; // Front-end planner
  std::unique_ptr<PlannerBase> front_end_planner_; // Front-end planner
  std::unique_ptr<SFCBase> sfc_generation_; // Spherical safe flight corridor generator
  std::unique_ptr<PolytopeSFC> poly_sfc_gen_; // Polytope safe flight corridor generator

  std::unique_ptr<back_end::PolyhedronSFCOptimizer> polyhedron_sfc_optimizer_; // Polynomial trajectory optimizer
  std::unique_ptr<back_end::SphericalSFCOptimizer> ssfc_optimizer_; // Spherical SFC optimizer
  std::unique_ptr<ego_planner::EGOPlannerManager> ego_optimizer_; // Back-end planner

  /* Data structs */
  Waypoint waypoints_; // Goal waypoint handler object
  Eigen::Vector3d cur_pos_, cur_vel_;   // current state
  Eigen::Vector3d rhp_goal_pos_; // Receding horizon planning goal
  std::shared_ptr<std::vector<ego_planner::LocalTrajData>> swarm_local_trajs_; // Swarm MINCO trajectories, maps drone_id to local trajectory data

  std::shared_ptr<SSFC::SFCTrajectory> ssfc_{nullptr};   // Safe flight corridor trajectory

  // Polyhedron sfc data 
  std::vector<Polyhedron3D, Eigen::aligned_allocator<Polyhedron3D>> poly_sfc_hyp_;
  std::vector<Eigen::Matrix3Xd> poly_sfc_vtx_;

  /* Timestamps for detecting timeouts*/
  ros::Time last_state_output_t_; // Last time stamp at which UAV odom is received

private: /* Params */
  int traj_id_{0}; // Trajectory id that increments with every planning cycle

  /* Front end planner parameters */
  JPSWrapper::JPSParams jps_params_; 
  AStarPlanner::AStarParams astar_params_; 

  /* SFC parameters */
  SphericalSFC::SphericalSFCParams sph_sfc_params_; 
  PolytopeSFC::PolytopeSFCParams ply_sfc_params_;   

  /* Back-end optimizer parameters */
  ego_planner::EGOPlannerParams ego_params_;

  /* Coordinator params */
  std::string node_name_{"Navigator"};
  int drone_id_{-1};
  int max_drones_{100};        // Maximum number of drones. Used to preallocate swarm trajectory data structure
  
  double hb_freq_{20.0};       // Default at 20 Hz
  
  bool debug_planning_{false};       // IF true, then debug mode is activated
  bool verbose_planning_{false};     // Print debug info during planning
  
  double squared_goal_tol_{-1.0};   // Squared goal tolerance, if within this tolerance, goal is considered to have been reached

  double fe_planner_freq_{-1.0};       // Front end planner timer frequency
  double be_planner_freq_{-1.0};       // Back end planner timer frequency
  double safety_check_freq_{-1.0};  // Planner timer frequency
  double rhp_dist_{-1.0};           // Receding horizon planning dist
  double rhp_buffer_{-1.0};         // Buffer to put goal away from obstacles (in addition to inflation)

  double rhc_dist_{0.0}; // Receding horizon corridor: Keep safe flight corridors up to this distance

  SFCType sfc_type_{SFCType::SPHERICAL}; // Indicates the SFC generation (e.g. polytope or spherical)
  FrontEndType front_end_type_{FrontEndType::JPS_AND_DMP}; // Indicates the Front end planner (e.g. a* or JPS)
  BackEndType back_end_type_{BackEndType::SSFC}; // Indicates the Front end planner (e.g. a* or JPS)

  /* Back-end params */
  int optimizer_num_retries_{-1}; // Number of retries for back-end optimizer
  int num_cstr_pts_per_seg_{-1}; // Number of constraint points per segment

  /* Collision checking params*/
  double swarm_clearance_{-1.0}; // Required clearance between swarm agents
  double time_to_col_threshold_{-1.0}; // Threshold for time to collision before emergency measures are activated

  /* Stopwatch for profiling performance */
  Timer tm_front_end_plan_{"front_end_plan"};
  Timer tm_sfc_plan_{"sfc_plan"};
  Timer tm_back_end_plan_{"back_end_plan"};

  /* Logic Flags (EGO PLANNER ONLY) */
  bool init_new_poly_traj_{true};   // (EGO PLANNER ONLY) If true: initialize new polynomial. Else: start from previous polynomial
  bool touch_goal_{false};   // (EGO PLANNER ONLY) If true:  Local target is global target

  /* Logic Flags */
  bool enable_rhc_plan_{false}; // True if RHC plan is enabled

private: /* Logging functions */
  
  void logInfo(const std::string& str){
    ROS_INFO_NAMED(node_name_, "UAV_%i: %s", 
      drone_id_, str.c_str());
  }

  void logWarn(const std::string& str){
    ROS_WARN_NAMED(node_name_, "UAV_%i: %s", 
      drone_id_, str.c_str());
  }

  void logError(const std::string& str){
    ROS_ERROR_NAMED(node_name_, "UAV_%i: %s", 
      drone_id_, str.c_str());
  }

  void logFatal(const std::string& str){
    ROS_FATAL_NAMED(node_name_, "UAV_%i: %s", 
      drone_id_, str.c_str());
  }

  void logInfoThrottled(const std::string& str, double period){
    ROS_INFO_THROTTLE_NAMED(period, node_name_, "UAV_%i: %s", 
      drone_id_, str.c_str());
  }

  void logWarnThrottled(const std::string& str, double period){
    ROS_WARN_THROTTLE_NAMED(period, node_name_, "UAV_%i: %s", 
      drone_id_, str.c_str());
  }

  void logErrorThrottled(const std::string& str, double period){
    ROS_ERROR_THROTTLE_NAMED(period, node_name_, "UAV_%i: %s", 
      drone_id_, str.c_str());
  }

  void logFatalThrottled(const std::string& str, double period){
    ROS_FATAL_THROTTLE_NAMED(period, node_name_, "UAV_%i: %s", 
      drone_id_, str.c_str());
  }

}; // class Navigator

#endif // _NAVIGATOR_H_