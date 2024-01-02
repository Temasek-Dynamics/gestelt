
#include <front_end_planner/front_end_planner.h>

void FrontEndPlanner::init(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{

  double planner_freq;
  pnh.param("planner_frequency", planner_freq, -1.0);
  pnh.param("goal_tolerance", squared_goal_tol_, -1.0);
  squared_goal_tol_ *= squared_goal_tol_; 
  
  odom_sub_ = nh.subscribe("odom", 1, &FrontEndPlanner::odometryCB, this);
  goal_sub_ = nh.subscribe("planner/goals", 1, &FrontEndPlanner::goalsCB, this);

  debug_start_sub_ = nh.subscribe("debug/plan_start", 1, &FrontEndPlanner::debugStartCB, this);
  debug_goal_sub_ = nh.subscribe("debug/plan_goal", 1, &FrontEndPlanner::debugGoalCB, this);

  plan_timer_ = nh.createTimer(ros::Duration(1/planner_freq), &FrontEndPlanner::planTimerCB, this);
}

/**
 * Timer Callbacks
*/

void FrontEndPlanner::planTimerCB(const ros::TimerEvent &e)
{
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
  // plan = front_end_planner_->plan(start_pos_, goal_pos_);

  // Refine plan
  // plan_refined = sfc_planner_->refine(plan);

  // Publish plan
  // front_end_plan_pub_.publish(plan_refined);
}

/**
 * Subscriber Callbacks
*/

void FrontEndPlanner::odometryCB(const nav_msgs::OdometryConstPtr &msg)
{
  // TODO Add mutex 
  cur_pos_= Eigen::Vector3d{msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z};
  cur_vel_= Eigen::Vector3d{msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z};
}

void FrontEndPlanner::goalsCB(const gestelt_msgs::GoalsPtr &msg)
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

  // for (auto wp : waypoints_.getQueue())
  // {
  //   visualization_->displayGoalPoint(wp, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
  //   ros::Duration(0.001).sleep();
  // }

}

void FrontEndPlanner::debugStartCB(const geometry_msgs::PoseStampedPtr &msg)
{
  cur_pos_ = Eigen::Vector3d{
        msg->pose.position.x,
        msg->pose.position.y,
        msg->pose.position.z};
}

void FrontEndPlanner::debugGoalCB(const geometry_msgs::PoseStampedPtr &msg)
{
  waypoints_.reset();
  waypoints_.addWP(Eigen::Vector3d{
        msg->pose.position.x,
        msg->pose.position.y,
        msg->pose.position.z});
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

}

