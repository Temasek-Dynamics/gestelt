#ifndef _BACK_END_PLANNER_H_
#define _BACK_END_PLANNER_H_

#include <algorithm>

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Empty.h>

#include <visualization_msgs/Marker.h>

#include <gestelt_msgs/SphericalSFCTrajectory.h>

#include <traj_utils/PolyTraj.h>
#include <traj_utils/MINCOTraj.h>
#include <ego_planner_fsm/ego_planner_manager.h>

class BackEndPlanner
{
public:
  BackEndPlanner() {}
  ~BackEndPlanner() {}

  void init(ros::NodeHandle &nh, ros::NodeHandle &pnh);

private:
  ros::NodeHandle node_;
  std::string node_name_{"BackEndPlanner"};

  // Subscribers and publishers
  ros::Subscriber debug_start_sub_; // DEBUG: Subscriber to user-defined start point
  ros::Subscriber debug_goal_sub_; // DEBUG: Subscriber to user-defined goal point
  ros::Subscriber sfc_traj_sub_; // Sub to Safe Flight Corridor trajectory

  ros::Subscriber plan_on_demand_esdf_free_sub_; // Subscriber to plan on demand for ESDF Free
  
  ros::Publisher plan_traj_pub_; // Pub planning trajectory (polynomial trajectory)

  /* parameters */
  int drone_id_{-1};
  double planning_horizon_;
  int num_replan_retries_;

  /* Mapping */
  // std::shared_ptr<GridMap> map_;

  /* Planner */
  std::unique_ptr<ego_planner::EGOPlannerManager> back_end_planner_; // Back-end planner
  std::shared_ptr<ego_planner::PlanningVisualization> visualization_; // For publishing visualizations
  
  /* Data structs */
  Eigen::Vector3d start_pos_, start_vel_;
  Eigen::Vector3d goal_pos_;
  

private: 

  /* Subscriber callbacks */

  void debugStartCB(const geometry_msgs::PoseConstPtr &msg);

  void debugGoalCB(const geometry_msgs::PoseConstPtr &msg);

  void planOnDemandESDFFree(const std_msgs::EmptyConstPtr &msg);

  /**
   * @brief Callback for safe flight corridor trajectory
   * 
   * @param msg 
   */
  void sfcTrajectoryCB(const gestelt_msgs::SphericalSFCTrajectoryConstPtr& msg);

  /* Plan generation methods */

  /**
   * @brief 
   * 
   * @param start_pos   Start position vector [x, y, z]
   * @param start_vel   Start velocity vector [dx, dy, dz]
   * @param inner_wps   Vector of inner waypoints [ [x_1, y_1, z_1], [x_2, y_2, z_2], ..., [x_M-1, y_M-1, z_M-1]]
   * @param segs_t_dur  Vector of time durations (s) of each segment (t_1, t_2, ..., t_M)
   * @param goal_pos    Local goal position vector [x, y, z]
   * @param num_opt_retries Number of times to retry optimization 
   * @return true 
   * @return false 
   */
  bool generatePlanSFC(  const Eigen::Vector3d& start_pos, const Eigen::Vector3d& start_vel, 
                      const std::vector<Eigen::Vector3d>& inner_wps, const Eigen::VectorXd& segs_t_dur,
                      const Eigen::Vector3d& goal_pos, const int& num_opt_retries,
                      const std::vector<double>& spheres_radius,
                      const std::vector<Eigen::Vector3d>& spheres_center,
                      poly_traj::MinJerkOpt& optimized_mjo
                      );

  /**
   * @brief Generate and optimize a given plan using ESDF free method
   * 
   * @param start_pos 
   * @param start_vel 
   * @param goal_pos 
   * @return true 
   * @return false 
   */
  bool generatePlanESDFFree(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& start_vel, 
    const Eigen::Vector3d& goal_pos, const int& num_opt_retries);

  /* Checks */

  /**
   * @brief Check feasibility of plan
   * 
   * @return true 
   * @return false 
   */
  bool isPlanFeasible(const Eigen::Vector3d& waypoints);

  /* Helper functions */

  /**
   * @brief Convert minimum jerk trajectory to message 
   * 
   * @param mjo minimum jerk trajectory
   * @param poly_msg polynomial trajectory message
   */
  void mjoToMsg(const poly_traj::MinJerkOpt& mjo, traj_utils::PolyTraj &poly_msg);

private:
  template<typename ... Args>
  std::string str_fmt( const std::string& format, Args ... args )
  {
    int size_s = std::snprintf( nullptr, 0, format.c_str(), args ... ) + 1; // Extra space for '\0'
    if( size_s <= 0 ){ throw std::runtime_error( "Error during formatting." ); }
    auto size = static_cast<size_t>( size_s );
    std::unique_ptr<char[]> buf( new char[ size ] );
    std::snprintf( buf.get(), size, format.c_str(), args ... );
    return std::string( buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
  }


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
    ROS_WARN_NAMED(node_name_, "UAV_%i: %s", 
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

}; // class BackEndPlanner

#endif // _BACK_END_PLANNER_H_