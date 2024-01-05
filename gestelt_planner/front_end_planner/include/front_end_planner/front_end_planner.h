#ifndef _EGO_PLANNER_FSM_H_
#define _EGO_PLANNER_FSM_H_

#include <front_end_planner/front_end_helper.h>

#include <algorithm>

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <gestelt_msgs/Goals.h>

#include <visualization_msgs/Marker.h>

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

  // Subscribers and publishers
  ros::Subscriber odom_sub_; // Subscriber to drone odometry
  ros::Subscriber goal_sub_; // Subscriber to user-defined goals

  ros::Subscriber debug_start_sub_; // DEBUG: Subscriber to user-defined start point
  ros::Subscriber debug_goal_sub_; // DEBUG: Subscriber to user-defined goal point
  ros::Subscriber plan_on_demand_sub_; // DEBUG: Subscriber to trigger planning on demand

  ros::Publisher front_end_plan_pub_; // Publish front end plan to back-end optimizer
  ros::Publisher front_end_plan_viz_pub_; // Visualization of front end plan
  ros::Publisher closed_list_viz_pub_; // Visualization of closed list

  ros::Timer plan_timer_; // Timer for planning loop
  
  /* parameters */
  int drone_id_;
  double squared_goal_tol_; // Squared goal tolerance
  bool within_goal_tol_; // Within a specified tolerance of the goal


  /* Mapping */
  std::shared_ptr<GridMap> map_;

  /* Planner */
  std::unique_ptr<AStarPlanner> front_end_planner_; // Front-end planner
  std::unique_ptr<SphericalSFC> sfc_generation_; // Safe flight corridor generator

  /* Data structs */
  Waypoint waypoints_; // Waypoint handler object
  Eigen::Vector3d cur_pos_, cur_vel_, cur_acc_;   // current state
  Eigen::Vector3d start_pos_, start_vel_, start_acc_;   // start state
  Eigen::Vector3d goal_pos_, goal_vel_, goal_acc_; // goal state

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
   * @brief Callback for odometry message
   * 
   * @param msg 
   */
  void odometryCB(const nav_msgs::OdometryConstPtr &msg);

  /**
   * @brief callback to debug topic for setting starting point of plan
   * 
   * @param msg 
   */
  void debugStartCB(const geometry_msgs::PoseConstPtr &msg);

  /**
   * @brief callback to debug goal topic for setting starting point of plan
   * 
   * @param msg 
   */
  void debugGoalCB(const geometry_msgs::PoseConstPtr &msg);

  /**
   * @brief callback to debug goal topic for setting starting point of plan
   * 
   * @param msg 
   */
  void planOnDemandCB(const std_msgs::EmptyConstPtr &msg);

  /**
   * @brief Generate a plan
   * 
   */
  void generatePlan();

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

  /* Visualization methods */

  /**
   * @brief Publish closed list for visualization
   * 
   * @param closed_list 
   */
  void publishVizClosedList(const std::vector<Eigen::Vector3d>& closed_list, const std::string& frame_id) {
    visualization_msgs::Marker closed_nodes;

    closed_nodes.header.frame_id = frame_id;
    closed_nodes.header.stamp = ros::Time::now();
    closed_nodes.type = visualization_msgs::Marker::CUBE_LIST;
    closed_nodes.action = visualization_msgs::Marker::ADD;
    closed_nodes.id = 0; 
    closed_nodes.pose.orientation.w = 1.0;

    closed_nodes.color.r = 1.0;
    closed_nodes.color.g = 1.0;
    closed_nodes.color.b = 1.0;
    closed_nodes.color.a = 0.6;

    closed_nodes.scale.x = 0.1;
    closed_nodes.scale.y = 0.1;
    closed_nodes.scale.z = 0.1;

    geometry_msgs::Point pt;
    for (auto pos : closed_list) {
      pt.x = pos(0);
      pt.y = pos(1);
      pt.z = pos(2);
      closed_nodes.points.push_back(pt);
    }

    closed_list_viz_pub_.publish(closed_nodes);
  }

  /**
   * @brief Publish path for visualization
   * 
   * @param path 
   */
  void publishVizPath(const std::vector<Eigen::Vector3d>& path, const std::string& frame_id) {
    visualization_msgs::Marker path_spheres, start_sphere, goal_sphere, path_line_strip;

    start_sphere.header.frame_id = goal_sphere.header.frame_id = frame_id;
    start_sphere.header.stamp = goal_sphere.header.stamp = ros::Time::now();
    start_sphere.type = goal_sphere.type = visualization_msgs::Marker::SPHERE;
    start_sphere.action = goal_sphere.action = visualization_msgs::Marker::ADD;
    start_sphere.id = 1;
    goal_sphere.id = 2; 
    start_sphere.pose.orientation.w = goal_sphere.pose.orientation.w = 1.0;

    start_sphere.color.r = goal_sphere.color.r = 0.0;
    start_sphere.color.g = goal_sphere.color.g = 0.0;
    start_sphere.color.b = goal_sphere.color.b = 1.0;
    start_sphere.color.a = goal_sphere.color.a = 0.6;

    start_sphere.scale.x = goal_sphere.scale.x = 0.2;
    start_sphere.scale.y = goal_sphere.scale.y = 0.2;
    start_sphere.scale.z = goal_sphere.scale.z = 0.2;

    path_line_strip.header.frame_id = frame_id;
    path_line_strip.header.stamp = ros::Time::now();
    path_line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    path_line_strip.action = visualization_msgs::Marker::ADD;
    path_line_strip.id = 1000;
    path_line_strip.pose.orientation.w = 1.0;

    path_line_strip.color.r = 0.0;
    path_line_strip.color.g = 0.0;
    path_line_strip.color.b = 1.0;
    path_line_strip.color.a = 0.6;

    path_line_strip.scale.x = 0.1;

    path_spheres.header.frame_id = frame_id;
    path_spheres.header.stamp = ros::Time::now();
    path_spheres.type = visualization_msgs::Marker::SPHERE_LIST;
    path_spheres.action = visualization_msgs::Marker::ADD;
    path_spheres.ns = "front_end_plan_spheres"; 
    path_spheres.id = 995; 
    path_spheres.pose.orientation.w = 1.0;

    path_spheres.color.r = 0.0;
    path_spheres.color.g = 0.0;
    path_spheres.color.b = 1.0;
    path_spheres.color.a = 0.6;

    path_spheres.scale.x = 0.1;
    path_spheres.scale.y = 0.1;
    path_spheres.scale.z = 0.1;

    start_sphere.pose.position.x = path[0](0);
    start_sphere.pose.position.y = path[0](1);
    start_sphere.pose.position.z = path[0](2);

    geometry_msgs::Point pt;
    for (int i = 1; i < path.size() - 1; i++){
      pt.x = path[i](0);
      pt.y = path[i](1);
      pt.z = path[i](2);

      path_spheres.points.push_back(pt);
      // path_line_strip.points.push_back(pt);
    }

    goal_sphere.pose.position.x = path.back()(0);
    goal_sphere.pose.position.y = path.back()(1);
    goal_sphere.pose.position.z = path.back()(2);

    // front_end_plan_viz_pub_.publish(start_sphere);
    // front_end_plan_viz_pub_.publish(goal_sphere);
    front_end_plan_viz_pub_.publish(path_spheres);
    // front_end_plan_viz_pub_.publish(path_line_strip);
  }

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

};

#endif // _EGO_PLANNER_FSM_H_