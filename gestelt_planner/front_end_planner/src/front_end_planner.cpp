#include <front_end_planner/front_end_planner.h>

void FrontEndPlanner::init(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{ 
  ROS_INFO("Initialized front end planner");
  double planner_freq;
  pnh.param("front_end/planner_frequency", planner_freq, -1.0);
  pnh.param("front_end/goal_tolerance", squared_goal_tol_, -1.0);
  squared_goal_tol_ *= squared_goal_tol_; 
  
  odom_sub_ = nh.subscribe("odom", 5, &FrontEndPlanner::odometryCB, this);
  goal_sub_ = nh.subscribe("planner/goals", 5, &FrontEndPlanner::goalsCB, this);

  debug_start_sub_ = nh.subscribe("debug/plan_start", 5, &FrontEndPlanner::debugStartCB, this);
  debug_goal_sub_ = nh.subscribe("debug/plan_goal", 5, &FrontEndPlanner::debugGoalCB, this);

  plan_on_demand_sub_ = nh.subscribe("plan_on_demand", 5, &FrontEndPlanner::planOnDemandCB, this);

  front_end_plan_viz_pub_ = nh.advertise<visualization_msgs::Marker>("plan_viz", 10);
  closed_list_viz_pub_ = nh.advertise<visualization_msgs::Marker>("closed_list_viz", 10);

  // plan_timer_ = nh.createTimer(ros::Duration(1/planner_freq), &FrontEndPlanner::planTimerCB, this);

  // Initialize map
  map_.reset(new GridMap);
  map_->initMap(nh, pnh);

  // Initialize front end planner 
  front_end_planner_ = std::make_unique<AStarPlanner>(map_);
}

/**
 * Timer Callbacks
*/

void FrontEndPlanner::planTimerCB(const ros::TimerEvent &e)
{
  generatePlan();
}

void FrontEndPlanner::generatePlan(){
  
  // Check if waypoint queue is empty
  if (waypoints_.empty()){
    return;
  }

  if (isGoalReached(cur_pos_, waypoints_.nextWP())){
    waypoints_.popWP();
    return;
  }

  // Generate a plan
  start_pos_ = cur_pos_;
  goal_pos_ = waypoints_.nextWP();

  ros::Time plan_start_time = ros::Time::now();

  if (!front_end_planner_->generatePlan(start_pos_, goal_pos_)){
    ROS_ERROR("[FrontEndPlanner] Path generation failed!");
    publishVizClosedList(front_end_planner_->getClosedList(), "world");
    return;
  }
  double plan_time_ms = (ros::Time::now() - plan_start_time).toSec() * 1000;
  ROS_INFO("[FrontEndPlanner]: Planning Time: %f ms", plan_time_ms);

  std::vector<Eigen::Vector3d> path = front_end_planner_->getPathPos();
  std::vector<Eigen::Vector3d> closed_list = front_end_planner_->getClosedList();

  // Generate Safe flight corridor
  // plan_refined = sfc_planner_->generateSFC(plan);
  ROS_INFO("[FrontEndPlanner]: Size of path: %ld", path.size());
  ROS_INFO("[FrontEndPlanner]: Size of closed list (expanded nodes): %ld", closed_list.size());

  // Publish front end plan
  publishVizPath(path, "world");
  publishVizClosedList(closed_list, "world");

}

/**
 * Subscriber Callbacks
*/

/**
 * @brief Callback to generate plan on demand
 * 
 * @param msg 
 */
void FrontEndPlanner::planOnDemandCB(const std_msgs::EmptyConstPtr& msg){
  ROS_INFO("[FrontEndPlanner]: Planning on demand triggered! from (%f, %f, %f) to (%f, %f, %f)",
    cur_pos_(0), cur_pos_(1), cur_pos_(2),
    waypoints_.nextWP()(0), waypoints_.nextWP()(1), waypoints_.nextWP()(2)
  );
  generatePlan();
}

void FrontEndPlanner::odometryCB(const nav_msgs::OdometryConstPtr &msg)
{
  // TODO Add mutex 
  cur_pos_= Eigen::Vector3d{msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z};
  cur_vel_= Eigen::Vector3d{msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z};
}

void FrontEndPlanner::goalsCB(const gestelt_msgs::GoalsConstPtr &msg)
{
  if (msg->transforms.size() <= 0)
  {
    logError("[FrontEndPlanner]: Received empty waypoints");
    return;
  }
  if (msg->header.frame_id != "world" && msg->header.frame_id != "map" )
  {
    logError("[FrontEndPlanner]: Only waypoint goals in 'world' or 'map' frame are accepted, ignoring waypoints.");
    return;
  }

  std::vector<Eigen::Vector3d> wp_vec;

  for (auto& pos : msg->transforms) {
    // Transform received waypoints from world to UAV origin frame
    wp_vec.push_back(Eigen::Vector3d{pos.translation.x, pos.translation.y, pos.translation.z});
  }

  waypoints_.addMultipleWP(wp_vec);

  // for (auto wp : waypoints_.getQueue())
  // {
  //   visualization_->displayGoalPoint(wp, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
  //   ros::Duration(0.001).sleep();
  // }

}

void FrontEndPlanner::debugStartCB(const geometry_msgs::PoseConstPtr &msg)
{
  ROS_INFO("[FrontEndPlanner]: Received debug start (%f, %f, %f)", 
        msg->position.x,
        msg->position.y,
        msg->position.z);
  cur_pos_ = Eigen::Vector3d{
        msg->position.x,
        msg->position.y,
        msg->position.z};
}

void FrontEndPlanner::debugGoalCB(const geometry_msgs::PoseConstPtr &msg)
{
  ROS_INFO("[FrontEndPlanner]: Received debug goal (%f, %f, %f)", 
        msg->position.x,
        msg->position.y,
        msg->position.z);
  waypoints_.reset();
  waypoints_.addWP(Eigen::Vector3d{
        msg->position.x,
        msg->position.y,
        msg->position.z});
}

/* Checking methods */

bool FrontEndPlanner::isGoalReached(const Eigen::Vector3d& pos, const Eigen::Vector3d& goal){
  if ((pos - goal).squaredNorm() < squared_goal_tol_){
    return true;
  }

  return false;
}

bool FrontEndPlanner::isPlanFeasible(const Eigen::Vector3d& waypoints){
  // Check occupancy of every waypoint
  return true;
}