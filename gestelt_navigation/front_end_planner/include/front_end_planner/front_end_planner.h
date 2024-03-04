#ifndef _FRONT_END_PLANNER_H_
#define _FRONT_END_PLANNER_H_

#include <front_end_planner/front_end_helper.h>

#include <algorithm>

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

#include <gestelt_msgs/Goals.h>
#include <gestelt_msgs/SphericalSFCTrajectory.h>

#include <gestelt_debug_msgs/SFCTrajectory.h>
#include <gestelt_debug_msgs/SFCSegment.h>

#include <optimizer/poly_traj_utils.hpp>
#include <traj_utils/PolyTraj.h>

#include <global_planner/a_star.h>
#include <sfc_generation/spherical_sfc.h>

class FrontEndPlanner
{
public:
  FrontEndPlanner() {}
  ~FrontEndPlanner() {}

  void init(ros::NodeHandle &nh, ros::NodeHandle &pnh);

private:
  ros::NodeHandle node_;
  std::string node_name_{"FrontEndPlanner"};

  ros::Timer plan_timer_; // Timer for planning loop

  // Subscribers and publishers
  ros::Subscriber odom_sub_; // Subscriber to drone odometry
  ros::Subscriber goal_sub_; // Subscriber to user-defined goals
  ros::Subscriber single_goal_sub_; // Subscriber to single goals

  ros::Subscriber plan_traj_sub_; // Subscriber to back end planner trajectory

  ros::Subscriber debug_start_sub_; // DEBUG: Subscriber to user-defined start point
  ros::Subscriber debug_goal_sub_; // DEBUG: Subscriber to user-defined goal point
  ros::Subscriber plan_on_demand_sub_; // DEBUG: Subscriber to trigger planning on demand

  ros::Publisher spherical_sfc_traj_pub_; // Publish safe flight corridor spherical trajectory

  /* Visualization for Front End Planner*/
  ros::Publisher front_end_plan_pub_; // Publish front end plan to back-end optimizer
  ros::Publisher front_end_plan_viz_pub_; // Visualization of front end plan
  ros::Publisher closed_list_viz_pub_; // Visualization of closed list

  /* Visualization for SFC*/
  ros::Publisher sfc_spherical_viz_pub_; // Visualization of SFC spheres

  ros::Publisher sfc_p_cand_viz_pub_; // Visualization of SFC sampling points
  ros::Publisher sfc_dist_viz_pub_; // Visualization of SFC sampling distribution
  ros::Publisher samp_dir_vec_pub_; // Visualization of SFC sampling direction vectors
  ros::Publisher sfc_waypoints_viz_pub_; // Visualization of SFC waypoints

  /* Debugging */
  ros::Publisher dbg_sfc_traj_pub_;
  
  /* parameters */
  int drone_id_{-1};

  bool debug_planning_; // IF true, then debug mode is activated

  double squared_goal_tol_; // Squared goal tolerance
  bool within_goal_tol_; // Within a specified tolerance of the goal

  double avg_vel_, max_vel_; // average and maximum velocity 

  /* Mapping */
  std::shared_ptr<GridMap> map_;

  /* Planner */
  std::unique_ptr<AStarPlanner> front_end_planner_; // Front-end planner
  std::unique_ptr<SphericalSFC> sfc_generation_; // Safe flight corridor generator

  /* Data structs */
  Waypoint waypoints_; // Waypoint handler object
  Eigen::Vector3d cur_pos_, cur_vel_;   // current state
  Eigen::Vector3d start_pos_;   // start state

  boost::shared_ptr<poly_traj::Trajectory> be_traj_; //received back end trajectory

private: 

  /* Timer callbacks */

  /**
   * @brief Timer callback to generate plan
   * @param e 
   */
  void planTimerCB(const ros::TimerEvent &e);

  /* Subscriber callbacks */

  /**
   * @brief callback to multiple user-defined goal waypoints 
   * 
   * @param msg 
   */
  void goalsCB(const gestelt_msgs::GoalsConstPtr &msg);

  /**
   * @brief Callback for single goal
   * 
   * @param msg 
   */
  void singleGoalCB(const geometry_msgs::PoseStampedConstPtr& msg);

  /**
   * @brief Callback for odometry message
   * 
   * @param msg 
   */
  void odometryCB(const nav_msgs::OdometryConstPtr &msg);

  /**
   * @brief Callback for back end trajectory
   * 
   * @param msg 
   */
  void backEndTrajCB(const traj_utils::PolyTrajPtr msg);

  /**
   * @brief Sample the back end trajectory 
   * 
   * @param time_samp time at which back end trajectory is sampled
   * @param pos sampled position to assign value to
   * @return true 
   * @return false 
   */
  bool sampleBackEndTrajectory(const double& time_samp, Eigen::Vector3d& pos);

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
    generatePlan(cur_pos_, waypoints_.nextWP());
  }

  /* Planner methods */

  /**
   * @brief Generate a plan
   * 
   */
  bool generatePlan(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& goal_pos);

  /* Checks */

  /**
   * @brief Check if current position is within goal tolerance
   * 
   * @return true 
   * @return false 
   */
  bool isGoalReached(const Eigen::Vector3d& pos, const Eigen::Vector3d& goal);

  /**
   * @brief Check feasibility of plan
   * 
   * @return true 
   * @return false 
   */
  bool isPlanFeasible(const Eigen::Vector3d& waypoints);

private:

  /* Logging functions */
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

}; // class FrontEndPlanner

#endif // _FRONT_END_PLANNER_H_