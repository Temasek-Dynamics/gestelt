#include <front_end_planner/front_end_planner.h>

void FrontEndPlanner::init(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{ 
  logInfo("Initialized front end planner");

  pnh.param("drone_id", drone_id_, -1);
  pnh.param("front_end/debug_planning", debug_planning_, false);

  double planner_freq;
  pnh.param("front_end/planner_frequency", planner_freq, -1.0);
  pnh.param("front_end/goal_tolerance", squared_goal_tol_, -1.0);
  squared_goal_tol_ *= squared_goal_tol_; 

  AStarPlanner::AStarParams astar_params; 
  pnh.param("front_end/max_iterations", astar_params.max_iterations, -1);
  pnh.param("front_end/tie_breaker", astar_params.tie_breaker, -1.0);
  pnh.param("front_end/debug_viz", astar_params.debug_viz, false);
  pnh.param("front_end/cost_function_type", astar_params.cost_function_type, 2);
  
  SphericalSFC::SphericalSFCParams sfc_params; 
  pnh.param("sfc/max_iterations", sfc_params.max_itr, -1);
  pnh.param("sfc/debug_viz", sfc_params.debug_viz, false);

  pnh.param("sfc/max_sample_points", sfc_params.max_sample_points, -1);
  pnh.param("sfc/mult_stddev_x", sfc_params.mult_stddev_x, -1.0);
  pnh.param("sfc/mult_stddev_y", sfc_params.mult_stddev_y, -1.0);
  pnh.param("sfc/mult_stddev_z", sfc_params.mult_stddev_z, -1.0);
  pnh.param("sfc/W_cand_vol", sfc_params.W_cand_vol, -1.0);
  pnh.param("sfc/W_intersect_vol", sfc_params.W_intersect_vol, -1.0);

  pnh.param("sfc/min_sphere_vol", sfc_params.min_sphere_vol, -1.0);
  pnh.param("sfc/max_sphere_vol", sfc_params.max_sphere_vol, -1.0);

  pnh.param("sfc/max_vel", sfc_params.max_vel, 3.0);
  pnh.param("sfc/max_acc", sfc_params.max_acc, 10.0);

  pnh.param("sfc/spherical_buffer", sfc_params.spherical_buffer, 0.0);
  
  odom_sub_ = nh.subscribe("odom", 5, &FrontEndPlanner::odometryCB, this);
  goal_sub_ = nh.subscribe("planner/goals", 5, &FrontEndPlanner::goalsCB, this);

  plan_traj_sub_ = nh.subscribe("back_end/trajectory", 5, &FrontEndPlanner::backEndTrajCB, this);

  spherical_sfc_traj_pub_ = nh.advertise<gestelt_msgs::SphericalSFCTrajectory>("front_end/sfc_trajectory", 10);

  front_end_plan_viz_pub_ = nh.advertise<visualization_msgs::Marker>("plan_viz", 10);
  closed_list_viz_pub_ = nh.advertise<visualization_msgs::Marker>("closed_list_viz", 10);

  sfc_p_cand_viz_pub_ = nh.advertise<visualization_msgs::Marker>("sfc_cand_points", 10);
  sfc_dist_viz_pub_ = nh.advertise<visualization_msgs::MarkerArray>("sfc_dist", 10);
  sfc_spherical_viz_pub_ = nh.advertise<visualization_msgs::MarkerArray>("sfc_spherical", 10);
  sfc_waypoints_viz_pub_ = nh.advertise<visualization_msgs::Marker>("sfc_waypoints", 10);
  samp_dir_vec_pub_ = nh.advertise<visualization_msgs::MarkerArray>("sfc_samp_dir_vec", 10);

  if (debug_planning_){
    debug_start_sub_ = pnh.subscribe("debug/plan_start", 5, &FrontEndPlanner::debugStartCB, this);
    debug_goal_sub_ = pnh.subscribe("debug/plan_goal", 5, &FrontEndPlanner::debugGoalCB, this);
    plan_on_demand_sub_ = pnh.subscribe("plan_on_demand", 5, &FrontEndPlanner::planOnDemandCB, this);
  }
  else {
    plan_timer_ = nh.createTimer(ros::Duration(1/planner_freq), &FrontEndPlanner::planTimerCB, this);
  }

  // Initialize map
  map_.reset(new GridMap);
  map_->initMapROS(nh, pnh);

  // Initialize front end planner 
  front_end_planner_ = std::make_unique<AStarPlanner>(map_, astar_params);
  front_end_planner_->addVizPublishers(closed_list_viz_pub_);

  sfc_generation_ = std::make_unique<SphericalSFC>(map_, sfc_params);

  sfc_generation_->addVizPublishers(
    sfc_p_cand_viz_pub_, 
    sfc_dist_viz_pub_, 
    sfc_spherical_viz_pub_, 
    sfc_waypoints_viz_pub_,
    samp_dir_vec_pub_
  );
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

  // Sample the starting position from the back end trajectory
  if (!sampleBackEndTrajectory(ros::Time::now().toSec() ,start_pos_)){
    // If we are unable to sample the back end trajectory, we set the starting position as the quadrotor's current position
    start_pos_ = cur_pos_;
  }

  // Generate a front-end path
  generatePlan(start_pos_, waypoints_.nextWP());
}

/**
 * Timer Callbacks
*/

bool FrontEndPlanner::generatePlan(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& goal_pos){
  // logInfo(str_fmt("generatePlan() from (%f, %f, %f) to (%f, %f, %f)",
  //   start_pos(0), start_pos(1), start_pos(2),
  //   goal_pos(0), goal_pos(1), goal_pos(2))
  // );

  ros::Time front_end_plan_start_time = ros::Time::now();

  if (!front_end_planner_->generatePlan(start_pos, goal_pos)){
    logError("Path generation failed!");
    viz_helper::publishVizCubes(front_end_planner_->getClosedList(), "world", closed_list_viz_pub_);
    return false;
  }

  double front_end_plan_time_ms = (ros::Time::now() - front_end_plan_start_time).toSec() * 1000;

  std::vector<Eigen::Vector3d> front_end_path = front_end_planner_->getPathPos();
  std::vector<Eigen::Vector3d> closed_list = front_end_planner_->getClosedList();

  // Publish front end plan
  viz_helper::publishVizSpheres(front_end_path, "world", front_end_plan_viz_pub_) ;
  viz_helper::publishVizCubes(closed_list, "world", closed_list_viz_pub_);

  ros::Time sfc_plan_start_time = ros::Time::now();

  // Generate safe flight corridor from front end path
  if (!sfc_generation_->generateSFC(front_end_path)){
    return false;
  }
  double sfc_plan_time_ms = (ros::Time::now() - sfc_plan_start_time).toSec() * 1000;

  SphericalSFC::SFCTrajectory sfc_traj = sfc_generation_->getSFCTrajectory();

  gestelt_msgs::SphericalSFCTrajectory sfc_traj_msg;

  for (auto sphere : sfc_traj.spheres){
    gestelt_msgs::Sphere sphere_msg;
    sphere_msg.radius = sphere.radius;
    sphere_msg.center.x = sphere.center(0);
    sphere_msg.center.y = sphere.center(1);
    sphere_msg.center.z = sphere.center(2);
    sfc_traj_msg.spheres.push_back(sphere_msg);
  }

  for (auto wp : sfc_traj.waypoints)
  {
    geometry_msgs::Point wp_msg;
    wp_msg.x = wp(0);
    wp_msg.y = wp(1);
    wp_msg.z = wp(2);
    sfc_traj_msg.waypoints.push_back(wp_msg);
  }

  sfc_traj_msg.segments_time_duration = sfc_traj.segs_t_dur;
  spherical_sfc_traj_pub_.publish(sfc_traj_msg);

  // logInfo(str_fmt("Front-end Planning Time: %f ms", front_end_plan_time_ms));
  // logInfo(str_fmt("SFC Planning Time: %f ms", sfc_plan_time_ms));
  // logInfo(str_fmt("Number of waypoints in front-end path: %ld", front_end_path.size()));
  // logInfo(str_fmt("Size of closed list (expanded nodes): %ld", closed_list.size()));
  // logInfo(str_fmt("[SFC] Number of spheres in SFC Spherical corridor: %ld", sfc_traj.spheres.size()));
  // logInfo(str_fmt("[SFC] Number of waypoints: %ld", sfc_traj.waypoints.size()));
  // logInfo(str_fmt("[SFC] Number of time segment durations: %ld", sfc_traj.segs_t_dur.size()));

  return true;
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

void FrontEndPlanner::backEndTrajCB(const traj_utils::PolyTrajPtr msg)
{
    if (msg->order != 5)
    {
      // Only support trajectory order equals 5 now!
      return;
    }
    if (msg->duration.size() * (msg->order + 1) != msg->coef_x.size())
    {
      // WRONG trajectory parameters
      return;
    }

    // The chunk of code below is just converting the received 
    // trajectories into poly_traj::Trajectory type and storing it

    // piece_nums is the number of Pieces in the trajectory 
    int piece_nums = msg->duration.size();
    std::vector<double> dura(piece_nums);
    std::vector<poly_traj::CoefficientMat> cMats(piece_nums);
    for (int i = 0; i < piece_nums; ++i)
    {
      int i6 = i * 6;
      cMats[i].row(0) <<  msg->coef_x[i6 + 0], msg->coef_x[i6 + 1], msg->coef_x[i6 + 2],
                          msg->coef_x[i6 + 3], msg->coef_x[i6 + 4], msg->coef_x[i6 + 5];
      cMats[i].row(1) <<  msg->coef_y[i6 + 0], msg->coef_y[i6 + 1], msg->coef_y[i6 + 2],
                          msg->coef_y[i6 + 3], msg->coef_y[i6 + 4], msg->coef_y[i6 + 5];
      cMats[i].row(2) <<  msg->coef_z[i6 + 0], msg->coef_z[i6 + 1], msg->coef_z[i6 + 2],
                          msg->coef_z[i6 + 3], msg->coef_z[i6 + 4], msg->coef_z[i6 + 5];

      dura[i] = msg->duration[i];
    }

    be_traj_.reset(new poly_traj::Trajectory(dura, cMats));
    be_traj_->setGlobalStartTime(msg->start_time.toSec());
}

bool FrontEndPlanner::sampleBackEndTrajectory(
  const double& time_samp, 
  Eigen::Vector3d& pos)
{
  if (!be_traj_)
  {
    // ROS_INFO_THROTTLE(5.0, "[EGO Planner Adaptor] No trajectory received!");
    return false;
  }

  double t = time_samp - be_traj_->getGlobalStartTime();

  if (t >= 0.0 && t < be_traj_->getTotalDuration())
  {
    pos = be_traj_->getPos(t);
  }
  else {
    return false;
  }

  return true;
}

void FrontEndPlanner::goalsCB(const gestelt_msgs::GoalsConstPtr &msg)
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

/* Planning helper methods */

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