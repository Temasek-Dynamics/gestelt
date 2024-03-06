#include <navigator/navigator.h>

/* Initialization methods */

void Navigator::init(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{ 
  logInfo("Initialized Navigator");

  initParams(nh, pnh);
  initPublishers(nh, pnh);
  initSubscribers(nh, pnh);

  // Initialize map
  map_.reset(new GridMap);
  map_->initMapROS(nh, pnh);

  // Initialize visualizer 
  visualization_ = std::make_shared<ego_planner::PlanningVisualization>(pnh);

  // Initialize front end planner 
  front_end_planner_ = std::make_unique<AStarPlanner>(map_, astar_params_);
  front_end_planner_->addVizPublishers(closed_list_viz_pub_);

  // Initialize safe flight corridor generation
  sfc_generation_ = std::make_unique<SphericalSFC>(map_, sfc_params_);
  sfc_generation_->addVizPublishers(
    sfc_p_cand_viz_pub_, 
    sfc_dist_viz_pub_, 
    sfc_spherical_viz_pub_, 
    sfc_waypoints_viz_pub_,
    samp_dir_vec_pub_
  );

  // Initialize own trajectory
  swarm_minco_trajs_ = std::make_shared<std::unordered_map<int, ego_planner::LocalTrajData>>();
  (*swarm_minco_trajs_)[drone_id_] = ego_planner::LocalTrajData();

  // Initialize back-end planner
  back_end_optimizer_ = std::make_unique<PolyTrajOptimizer>();
  back_end_optimizer_->setParam(pnh);
  back_end_optimizer_->setEnvironment(grid_map_);

  back_end_optimizer_->setVisualizer(visualization_);
  back_end_optimizer_->assignSwarmTrajs(swarm_minco_trajs_);

  /* Initialize Timer */
  if (!debug_planning_){
    plan_timer_ = nh.createTimer(ros::Duration(1.0/planner_freq_), &Navigator::planTimerCB, this);
  }

  safetyChecksTimerCB = nh.createTimer(ros::Duration(1.0/safety_check_freq_), &Navigator::safetyChecksTimerCB, this);
}

void Navigator::initParams(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
  /* Navigator params*/
  pnh.param("drone_id", drone_id_, -1);
  pnh.param("goal_tolerance", squared_goal_tol_, -1.0);
  squared_goal_tol_ = squared_goal_tol_*squared_goal_tol_; 
  pnh.param("planner_frequency", planner_freq_, -1.0);
  pnh.param("safety_check_frequency", safety_check_freq_, -1.0);
  pnh.param("debug_planning", debug_planning_, false);

  /* Front end params */
  pnh.param("front_end/max_iterations", astar_params_.max_iterations, -1);
  pnh.param("front_end/tie_breaker", astar_params_.tie_breaker, -1.0);
  pnh.param("front_end/debug_viz", astar_params_.debug_viz, false);
  pnh.param("front_end/cost_function_type", astar_params_.cost_function_type, 2);
  
  /* Front SFC params */
  pnh.param("sfc/max_iterations", sfc_params_.max_itr, -1);
  pnh.param("sfc/debug_viz", sfc_params_.debug_viz, false);

  pnh.param("sfc/max_sample_points", sfc_params_.max_sample_points, -1);
  pnh.param("sfc/mult_stddev_x", sfc_params_.mult_stddev_x, -1.0);
  pnh.param("sfc/mult_stddev_y", sfc_params_.mult_stddev_y, -1.0);
  pnh.param("sfc/mult_stddev_z", sfc_params_.mult_stddev_z, -1.0);

  pnh.param("sfc/W_cand_vol", sfc_params_.W_cand_vol, -1.0);
  pnh.param("sfc/W_intersect_vol", sfc_params_.W_intersect_vol, -1.0);
  pnh.param("sfc/W_progress", sfc_params_.W_progress, -1.0);

  pnh.param("sfc/min_sphere_vol", sfc_params_.min_sphere_vol, -1.0);
  pnh.param("sfc/max_sphere_vol", sfc_params_.max_sphere_vol, -1.0);

  pnh.param("sfc/max_vel", sfc_params_.max_vel, 3.0);
  pnh.param("sfc/max_acc", sfc_params_.max_acc, 10.0);

  pnh.param("sfc/spherical_buffer", sfc_params_.spherical_buffer, 0.0);

  /* Back-end params */
  pnh.param("back_end/num_replan_retries", optimizer_num_retries_, -1);
}

void Navigator::initSubscribers(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
  /* Quadrotor state */
  odom_sub_ = nh.subscribe("odom", 5, &Navigator::odometryCB, this);

  /* Goals */
  goal_sub_ = nh.subscribe("planner/goals", 5, &Navigator::goalsCB, this);
  single_goal_sub_ = nh.subscribe("planner/single_goal", 5, &Navigator::singleGoalCB, this);

  /* Swarm trajectories */
  swarm_minco_traj_sub_ = nh.subscribe("/swarm/global/minco", 100,
                                        &Navigator::swarmMincoTrajCB,
                                        this,
                                        ros::TransportHints().tcpNoDelay());
}

void Navigator::initPublishers(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
  /* Front-end */
  front_end_plan_viz_pub_ = nh.advertise<visualization_msgs::Marker>("plan_viz", 10);
  closed_list_viz_pub_ = nh.advertise<visualization_msgs::Marker>("closed_list_viz", 10);

  /* SFC */
  spherical_sfc_traj_pub_ = nh.advertise<gestelt_msgs::SphericalSFCTrajectory>("front_end/sfc_trajectory", 10);

  sfc_p_cand_viz_pub_ = nh.advertise<visualization_msgs::Marker>("sfc_cand_points", 10);
  sfc_dist_viz_pub_ = nh.advertise<visualization_msgs::MarkerArray>("sfc_dist", 10);
  sfc_spherical_viz_pub_ = nh.advertise<visualization_msgs::MarkerArray>("sfc_spherical", 10);
  sfc_waypoints_viz_pub_ = nh.advertise<visualization_msgs::Marker>("sfc_waypoints", 10);
  samp_dir_vec_pub_ = nh.advertise<visualization_msgs::MarkerArray>("sfc_samp_dir_vec", 10);

  dbg_sfc_traj_pub_ = nh.advertise<gestelt_debug_msgs::SFCTrajectory>("sfc/debug_trajectory", 10, true);

  if (debug_planning_){
    debug_start_sub_ = pnh.subscribe("debug/plan_start", 5, &Navigator::debugStartCB, this);
    debug_goal_sub_ = pnh.subscribe("debug/plan_goal", 5, &Navigator::debugGoalCB, this);
    plan_on_demand_sub_ = pnh.subscribe("plan_on_demand", 5, &Navigator::planOnDemandCB, this);
  }

  /* Back-end */
  debug_traj_pub_ = nh.advertise<gestelt_debug_msgs::BackEndTrajectoryDebug>(
    "back_end/debug_trajectory", 10, true); 
  plan_traj_pub_ = nh.advertise<traj_utils::PolyTraj>("back_end/trajectory", 10); 
  swarm_minco_traj_pub_ = nh.advertise<traj_utils::MINCOTraj>("/swarm/global/minco", 10);

}

/* Timer callbacks */

void Navigator::planTimerCB(const ros::TimerEvent &e)
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

  // Generate a front-end plan
  SphericalSFC::SFCTrajectory sfc_traj;
  if (!generateFrontEndPlan(start_pos_, waypoints_.nextWP(), sfc_traj)){
    logError("Failed to generate front end plan!");
    return;
  }

  // Generate a back-end trajectory from front-end plan
  poly_traj::MinJerkOpt optimized_mjo; // Optimized minimum jerk trajectory
  if (!generateBackEndPlan( start_pos_, cur_vel_, waypoints_.nextWP(), 
                            sfc_traj, 
                            optimized_mjo
                            5)){
    logError("Failed to generate back end plan!");
    return;
  }

  be_traj_ = std::make_shared<poly_traj::Trajectory>(optimized_mjo.getTraj());


  traj_utils::PolyTraj poly_msg; 
  traj_utils::MINCOTraj MINCO_msg; 

  mjoToMsg(optimized_mjo_, poly_msg, MINCO_msg);
  plan_traj_pub_.publish(poly_msg); // (In drone origin frame) Publish to corresponding drone for execution
  swarm_minco_traj_pub_.publish(MINCO_msg); // (In world frame) Broadcast to all other drones for replanning to optimize in avoiding swarm collision

  // Update optimized trajectory 
  mincoMsgToTraj(MINCO_msg, (*swarm_minco_trajs_)[drone_id_]);
}

void Navigator::safetyChecksTimerCB(const ros::TimerEvent &e)
{
  bool obs_col{true}, swarm_col{true};

  isTrajectorySafe(&((*swarm_minco_trajs_)[drone_id_]), obs_col, swarm_col);

  if (obs_col || swarm_col){
    commandTrajectoryServer(ESTOP);
  }
  
}

/**
 * Planner methods
*/

bool Navigator::generateFrontEndPlan(
  const Eigen::Vector3d& start_pos, const Eigen::Vector3d& goal_pos,
  SphericalSFC::SFCTrajectory& sfc_traj)
{
  logInfo(str_fmt("generateFrontEndPlan() from (%f, %f, %f) to (%f, %f, %f)",
    start_pos(0), start_pos(1), start_pos(2),
    goal_pos(0), goal_pos(1), goal_pos(2))
  );

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

  sfc_traj = sfc_generation_->getSFCTrajectory();

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
  
  // Publish debug SFC trajectory
  std::vector<std::vector<SphericalSFC::Sphere>> sfc_sampled_spheres;
  std::vector<Eigen::Vector3d> samp_dir_vec, guide_points_vec;

  sfc_generation_->getSFCTrajectoryDebug(
    sfc_sampled_spheres, samp_dir_vec, guide_points_vec);

  if (sfc_sampled_spheres.size() != samp_dir_vec.size() 
      || samp_dir_vec.size() != guide_points_vec.size() 
      || sfc_sampled_spheres.size() != guide_points_vec.size()){
    ROS_ERROR("ERROR, SFC Debug trajectory fields do not all have the same size (same number of semgments)!");
  }

  gestelt_debug_msgs::SFCTrajectory dbg_sfc_traj_msg;
  for (size_t i = 0; i < guide_points_vec.size(); i++)
  {
    gestelt_debug_msgs::SFCSegment segment;
    for (auto& sphere : sfc_sampled_spheres[i])
    {
      gestelt_msgs::Sphere sphere_msg;
      sphere_msg.radius = sphere.radius;
      sphere_msg.center.x = sphere.center(0);
      sphere_msg.center.y = sphere.center(1);
      sphere_msg.center.z = sphere.center(2);

      segment.sampled_spheres.push_back(sphere_msg);
    }

    segment.guide_point.x = guide_points_vec[i](0);
    segment.guide_point.y = guide_points_vec[i](1);
    segment.guide_point.z = guide_points_vec[i](2);

    segment.sampling_vector.x = samp_dir_vec[i](0);
    segment.sampling_vector.y = samp_dir_vec[i](1);
    segment.sampling_vector.z = samp_dir_vec[i](2);

    dbg_sfc_traj_msg.segments.push_back(segment);
  }

  for (size_t i = 0; i < front_end_path.size(); i++)
  {
    geometry_msgs::Point front_end_pt;
    front_end_pt.x = front_end_path[i](0);
    front_end_pt.y = front_end_path[i](1);
    front_end_pt.z = front_end_path[i](2);

    dbg_sfc_traj_msg.front_end_path.push_back(front_end_pt);
  }

  dbg_sfc_traj_msg.sfc_spheres = sfc_traj_msg.spheres;
  dbg_sfc_traj_msg.sfc_waypoints = sfc_traj_msg.waypoints;
  dbg_sfc_traj_pub_.publish(dbg_sfc_traj_msg);

  // logInfo(str_fmt("Front-end Planning Time: %f ms", front_end_plan_time_ms));
  // logInfo(str_fmt("SFC Planning Time: %f ms", sfc_plan_time_ms));
  // logInfo(str_fmt("Number of waypoints in front-end path: %ld", front_end_path.size()));
  // logInfo(str_fmt("Size of closed list (expanded nodes): %ld", closed_list.size()));
  // logInfo(str_fmt("[SFC] Number of spheres in SFC Spherical corridor: %ld", sfc_traj.spheres.size()));
  // logInfo(str_fmt("[SFC] Number of waypoints: %ld", sfc_traj.waypoints.size()));
  // logInfo(str_fmt("[SFC] Number of time segment durations: %ld", sfc_traj.segs_t_dur.size()));

  return true;
}

bool Navigator::generateBackEndPlan( 
  const Eigen::Vector3d& start_pos, const Eigen::Vector3d& start_vel, 
  const Eigen::Vector3d& goal_pos, 
  SphericalSFC::SFCTrajectory& sfc_traj,
  poly_traj::MinJerkOpt& optimized_mjo,
  const int& num_retries)
{
  logInfo(str_fmt("generateBackEndPlan() from (%f, %f, %f) to (%f, %f, %f)", 
    start_pos(0), start_pos(1), start_pos(2), 
    goal_pos(0), goal_pos(1), goal_pos(2)));
  
  bool plan_success = false;

  Eigen::Vector3d start_acc;
  Eigen::Vector3d goal_vel;
  Eigen::Vector3d goal_acc;

  start_acc.setZero();
  goal_vel.setZero();
  goal_acc.setZero();

  int num_cstr_pts = back_end_optimizer_->get_cps_num_perPiece_();

  for (int i = 0; i < num_retries; i++)
  {
    /***************************/
    /*3:  Get minimum jerk trajectory */
    /***************************/

    int M = sfc_traj.getNumSegments(); // Number of path segments

    Eigen::Matrix3d startPVA; // Boundary start condition: Matrix consisting of 3d (position, velocity acceleration) 
    Eigen::Matrix3d endPVA;   // Boundary end condition: Matrix consisting of 3d (position, velocity acceleration) 
    startPVA << start_pos, start_vel, start_acc;            // Start (position, velocity, acceleration)
    endPVA << goal_pos, goal_vel, goal_acc;  // Goal (P)

    initial_mjo.reset(startPVA, endPVA, M);
    initial_mjo.generate(sfc_traj.getInnerWaypoints(), segs_t_dur);

    Eigen::MatrixXd init_cstr_pts = initial_mjo.getInitConstraintPoints(num_cstr_pts);

    std::vector<Eigen::Vector3d> initial_mjo_viz; // Visualization of the initial minimum jerk trajectory
    for (int i = 0; i < init_cstr_pts.cols(); ++i){
      initial_mjo_viz.push_back(init_cstr_pts.col(i));
    }
    visualization_->displayInitialMJO(initial_mjo_viz, 0.075, 0);
    visualization_->displayInitialCtrlPts(sfc_traj.getInnerWaypoints());

    // Display initial control points in xi coordinates
    Eigen::MatrixXd init_inner_ctrl_pts_xi = back_end_optimizer_->f_BInv_ctrl_pts(
                                              sfc_traj.getInnerWaypoints(), 
                                              spheres_center,
                                              spheres_radius);
    visualization_->displayInitialCtrlPts_xi(init_inner_ctrl_pts_xi);

    Eigen::MatrixXd init_inner_ctrl_pts_q = back_end_optimizer_->f_B_ctrl_pts(
                                              init_inner_ctrl_pts_xi, 
                                              spheres_center,
                                              spheres_radius);
    visualization_->displayInitialCtrlPts_q(init_inner_ctrl_pts_q);

    // Display initial constraint points in xi coordinates
    // Eigen::MatrixXd cstr_pts_xi = 
    //   back_end_optimizer_->f_BInv_cstr_pts(init_cstr_pts, 
    //                                           initial_mjo.getNumSegs(),
    //                                           num_cstr_pts,
    //                                           spheres_center,
    //                                           spheres_radius);
    // visualization_->displayInitialMJO_xi(cstr_pts_xi, 0); 

    // Eigen::MatrixXd cstr_pts_q = 
    //   back_end_optimizer_->f_B_cstr_pts(cstr_pts, 
    //                                           initial_mjo.getNumSegs(),
    //                                           num_cstr_pts,
    //                                           spheres_center,
    //                                           spheres_radius);
    // visualization_->displayInitialMJO_q(cstr_pts_q, 0); 

    /***************************/
    /*4:  Optimize plan
    /***************************/

    // Optimize trajectory!
    double final_cost; 
    plan_success = back_end_optimizer_->optimizeTrajectorySFC( 
          startPVA, endPVA,                   // Start and end (pos, vel, acc)
          sfc_traj.getInnerWaypoints(),       // Inner control points
          sfc_traj.getSegmentTimeDurations(), // Time durations of each segment
          sfc_traj.getSpheresRadii(),   
          sfc_traj.getSpheresCenter(),   
          final_cost);                      

    // Optimized minimum jerk trajectory
    optimized_mjo = back_end_optimizer_->getMinJerkOpt();
    Eigen::MatrixXd cstr_pts_optimized_mjo = optimized_mjo.getInitConstraintPoints(num_cstr_pts);

    /***************************/
    /* Print and display results for debugging
    /***************************/

    /* Publish all intermediate paths */
    visualization_->displayIntermediateMJO_xi(
      back_end_optimizer_->intermediate_cstr_pts_xi_);

    visualization_->displayIntermediateMJO_q(
      back_end_optimizer_->intermediate_cstr_pts_q_);

    // Print results for benchmarking
    double traj_length = getTrajectoryLength(optimized_mjo);
    double traj_jerk_cost = optimized_mjo.getTrajJerkCost();
    double trajectory_duration = getTrajectoryDuration(optimized_mjo);

    // logInfo(str_fmt("Trajectory: Length(%f), Jerk Cost(%f), Duration(%f)", 
    //   traj_length, traj_jerk_cost, trajectory_duration));

    /* Publish back end trajectory for debugging with trajectory inspector */
    gestelt_debug_msgs::BackEndTrajectoryDebug debug_traj_msg;
    for (int i = 0; i < init_cstr_pts.cols(); ++i){
      geometry_msgs::Point pt;
      pt.x = init_cstr_pts.col(i)(0);
      pt.y = init_cstr_pts.col(i)(1);
      pt.z = init_cstr_pts.col(i)(2);

      debug_traj_msg.initial_mjo.push_back(pt);
    }

    debug_traj_msg.num_cp = num_cstr_pts;
    debug_traj_msg.num_segs = initial_mjo.getNumSegs();
    debug_traj_pub_.publish(debug_traj_msg);

    /* Visualize optimized mjo trajectories */
    back_end_optimizer_->opt_costs_.printAll();

    logInfo(str_fmt("Final cost: %f", final_cost));

    if (plan_success)
    {
      logInfo("Planning successful!");
      visualization_->displayOptimalMJO(cstr_pts_optimized_mjo, 0);
      break;
    }
    else{
      logError(str_fmt("Trajectory optimization unsuccessful! Number retries left: %d", 
        num_retries - i));
      visualization_->displayFailedList(cstr_pts_optimized_mjo, 0);
    }
  }

  return plan_success;
}

/**
 * Subscriber Callbacks
*/

void Navigator::odometryCB(const nav_msgs::OdometryConstPtr &msg)
{
  odom_mutex_.lock();
  cur_pos_= Eigen::Vector3d{msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z};
  cur_vel_= Eigen::Vector3d{msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z};
  odom_mutex_.unlock();
}

void Navigator::singleGoalCB(const geometry_msgs::PoseStampedConstPtr& msg)
{
  logInfo(str_fmt("Received debug goal (%f, %f, %f). Note: position.z is set to default of 1.5m", 
        msg->pose.position.x,
        msg->pose.position.y,
        1.0));
  waypoints_.reset();
  waypoints_.addWP(Eigen::Vector3d{
        msg->pose.position.x,
        msg->pose.position.y,
        1.0});

  generateFrontEndPlan(cur_pos_, waypoints_.nextWP());
}

void Navigator::goalsCB(const gestelt_msgs::GoalsConstPtr &msg)
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

void Navigator::swarmMincoTrajCB(const traj_utils::MINCOTrajConstPtr &msg)
{
  if (msg->drone_id == drone_id_){
    // Self-published trajectory, ignore
    return; 
  }

  ros::Time t_now = ros::Time::now();
  if (abs((t_now - msg->start_time).toSec()) > 0.25)
  {
    if (abs((t_now - msg->start_time).toSec()) < 10.0) // 10 seconds offset, more likely to be caused by unsynced system time.
    {
      logWarn(str_fmt("Time stamp diff: Local - Remote Agent %d = %fs",
                msg->drone_id, (t_now - msg->start_time).toSec()));
    }
    else
    {
      logError(str_fmt("Time stamp diff: Local - Remote Agent %d = %fs, swarm time seems not synchronized, abandon!",
                msg->drone_id, (t_now - msg->start_time).toSec()));
      return;
    }
  }

  mincoMsgToTraj(*msg, (*swarm_minco_trajs_)[msg->drone_id]);
}

/* Checking methods */

bool Navigator::isGoalReached(const Eigen::Vector3d& pos, const Eigen::Vector3d& goal){
  if ((pos - goal).squaredNorm() < squared_goal_tol_){
    return true;
  }

  return false;
}

bool Navigator::isInCollisionSwarm(const Eigen::Vector3d& waypoints)

bool Navigator::isTrajectorySafe(LocalTrajData* traj, bool& obs_col, bool& swarm_col)
{
  obs_col = false;
  swarm_col = false;

  LocalTrajData *traj = &((*swarm_minco_trajs_)[drone_id_]);
  
  if (traj->traj_id <= 0){ // Return if no local trajectory yet
    return false;
  }

  double traj_t_cur = ros::Time::now().toSec() - traj->start_time;

  // pts_chk: Vector of constraint points. Within each piece is a vector of (timestamp, position).
  std::vector<std::vector<std::pair<double, Eigen::Vector3d>>> pts_chk = traj->pts_chk; 
  int num_segs = pts_chk.size(); // Number of segments

  // pair of (seg_idx, t_in_segment / dur_segment)
  std::pair<int, double> idx_time_ratio_pair = traj->traj.getIdxTimeRatioAtTime(traj_t_cur);
  
  // idx_seg is starting segment index, computed by multiplying segment index and it's fraction with the number of constraints per segment
  size_t idx_seg = floor((idx_time_ratio_pair.first + idx_time_ratio_pair.second) * back_end_optimizer_->get_cps_num_perPiece_());

  if (idx_seg >= num_segs) 
  {
      // Starting segment index exceeds total number of segments
      return false;
  }

  size_t idx_pt = 0; // idx of point within constraint piece
  for (; idx_seg < num_segs; idx_seg++) // iterate through each segment
  {
    for (idx_pt = 0; idx_pt < pts_chk[idx_seg].size(); ++idx_pt) // Iterate through each point in segment
    {
      // If time of point being checked exceeds current time, check for potential collision from that particular index onwards
      if (pts_chk[idx_seg][idx_pt].first > traj_t_cur)
      {
        goto find_ij_start;
      }
    }
  }
  
  find_ij_start:;
    for (size_t i = idx_seg; i < num_segs; ++i) // Iterate through each piece index
    {
      for (size_t j = idx_pt; j < pts_chk[i].size(); ++j) // Iterate through each point within piece index
      {
        double t = pts_chk[i][j].first;             // time
        Eigen::Vector3d pos = pts_chk[i][j].second; //position

        bool in_obs_grid = map_->getInflateOccupancy(pos); // Indicates if occupancy grid is occupied

        if (!in_obs_grid){ // If free from obstacle 
          
          for (size_t k = 0; k < (*swarm_minco_trajs_).size(); k++) // Iterate through trajectories of other agents
          {
              // Check that it's not a null pointer
              if (!(*swarm_minco_trajs_)[k]){
                  logError(str_fmt("Swarm agent %d has empty trajectory", k))
                  continue;
              }
              LocalTrajData *agent_k_traj = &((*swarm_minco_trajs_)[k]);

              if (agent_k_traj->drone_id == drone_id_)
              {
                  // Own trajectory
                  continue;
              }

              if (agent_k_traj->drone_id != (int)k)
              {
                  logError(str_fmt("Swarm agent %d's trajectory does not match it's prescribed ID %d", k, agent_k_traj->drone_id))
                  continue;
              }

              // Calculate time for other drone
              
              double t_X = t - (traj->start_time - agent_k_traj->start_time);
              // If time t_X is within the duration of the k-th agent trajectory
              if (t_X > 0 && t_X < agent_k_traj->duration) 
              {
                  Eigen::Vector3d agent_predicted_pos = agent_k_traj->traj.getPos(t_X);
                  double inter_agent_dist = (pos - agent_predicted_pos).norm();

                  if (inter_agent_dist < swarm_clearance_)
                  {
                  logWarn(str_fmt("Clearance between drones %d and %d is %f, too close!",
                          drone_id_, (int)id, inter_agent_dist));
                  
                  // swarm_col: indicates that a collision is imminent in "time_to_col_threshold_" seconds
                  swarm_col = (t - traj_t_cur) < time_to_col_threshold_;
                  return true;
                  }
              }
          }
        }
        else {
          obs_col = true;
          return true;
        }

      }
      idx_pt = 0; // reset to start of path
    }

    return false;
}

/* Helper methods */

bool Navigator::sampleBackEndTrajectory(
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

void Navigator::mincoMsgToTraj(const traj_utils::MINCOTraj &msg, ego_planner::LocalTrajData& traj)
{
  /* Store data */
  traj.drone_id = msg.drone_id;
  traj.traj_id = msg.traj_id;
  traj.start_time = msg.start_time.toSec();

  int piece_nums = msg.duration.size();
  Eigen::Matrix<double, 3, 3> headState, tailState;

  headState <<  msg.start_p[0], 
                  msg.start_v[0], msg.start_a[0],
                msg.start_p[1], 
                  msg.start_v[1], msg.start_a[1],
                msg.start_p[2], 
                  msg.start_v[2], msg.start_a[2];
  tailState <<  msg.end_p[0], 
                  msg.end_v[0], msg.end_a[0],
                msg.end_p[1], 
                  msg.end_v[1], msg.end_a[1],
                msg.end_p[2], 
                  msg.end_v[2], msg.end_a[2];

  Eigen::MatrixXd innerPts(3, piece_nums - 1);
  Eigen::VectorXd durations(piece_nums);

  for (int i = 0; i < piece_nums - 1; i++){
    innerPts.col(i) <<  msg.inner_x[i], 
                        msg.inner_y[i], 
                        msg.inner_z[i];
  }
  for (int i = 0; i < piece_nums; i++){
    durations(i) = msg.duration[i];
  }

  // Recreate trajectory using closed-form min jerk
  poly_traj::MinJerkOpt MJO;
  MJO.reset(headState, tailState, piece_nums);
  MJO.generate(innerPts, durations);

  poly_traj::Trajectory trajectory = MJO.getTraj();

  traj.traj = trajectory;
  traj.duration = trajectory.getTotalDuration();
  traj.start_pos = trajectory.getPos(0.0);
}

void Navigator::mjoToMsg(const poly_traj::MinJerkOpt& mjo, 
                              traj_utils::PolyTraj &poly_msg, traj_utils::MINCOTraj &MINCO_msg)
{
  static int traj_id = 0;
  traj_id++;

  poly_traj::Trajectory traj = mjo.getTraj();

  Eigen::VectorXd durs = traj.getDurations();
  int piece_num = traj.getPieceSize();
  poly_msg.drone_id = drone_id_;
  poly_msg.traj_id = traj_id;
  poly_msg.start_time = ros::Time::now();
  poly_msg.order = 5; 
  poly_msg.duration.resize(piece_num);
  poly_msg.coef_x.resize(6 * piece_num);
  poly_msg.coef_y.resize(6 * piece_num);
  poly_msg.coef_z.resize(6 * piece_num);

  // For each segment
  for (int i = 0; i < piece_num; ++i)
  {
    // Assign timestamp
    poly_msg.duration[i] = durs(i);

    // Assign coefficient matrix values
    poly_traj::CoefficientMat cMat = traj.getPiece(i).getCoeffMat();
    int i6 = i * 6;
    for (int j = 0; j < 6; j++)
    {
      poly_msg.coef_x[i6 + j] = cMat(0, j);
      poly_msg.coef_y[i6 + j] = cMat(1, j);
      poly_msg.coef_z[i6 + j] = cMat(2, j);
    }
  }

  MINCO_msg.drone_id = drone_id_;
  MINCO_msg.traj_id = traj_id;
  MINCO_msg.start_time = ros::Time::now();
  MINCO_msg.order = 5; 
  MINCO_msg.duration.resize(piece_num);

  Eigen::Vector3d vec; // Vector representing x,y,z values or their derivatives
  // Start Position
  vec = traj.getPos(0);
  MINCO_msg.start_p[0] = vec(0), MINCO_msg.start_p[1] = vec(1), MINCO_msg.start_p[2] = vec(2);
  // Start Velocity
  vec = traj.getVel(0);
  MINCO_msg.start_v[0] = vec(0), MINCO_msg.start_v[1] = vec(1), MINCO_msg.start_v[2] = vec(2);
  // Start Acceleration
  vec = traj.getAcc(0);
  MINCO_msg.start_a[0] = vec(0), MINCO_msg.start_a[1] = vec(1), MINCO_msg.start_a[2] = vec(2);
  // End position
  vec = traj.getPos(traj.getTotalDuration());
  MINCO_msg.end_p[0] = vec(0), MINCO_msg.end_p[1] = vec(1), MINCO_msg.end_p[2] = vec(2);
  // End velocity
  vec = traj.getVel(traj.getTotalDuration());
  MINCO_msg.end_v[0] = vec(0), MINCO_msg.end_v[1] = vec(1), MINCO_msg.end_v[2] = vec(2);
  // End Acceleration
  vec = traj.getAcc(traj.getTotalDuration());
  MINCO_msg.end_a[0] = vec(0), MINCO_msg.end_a[1] = vec(1), MINCO_msg.end_a[2] = vec(2);

  // Assign inner points
  MINCO_msg.inner_x.resize(piece_num - 1);
  MINCO_msg.inner_y.resize(piece_num - 1);
  MINCO_msg.inner_z.resize(piece_num - 1);
  Eigen::MatrixXd pos = traj.getPositions();
  for (int i = 0; i < piece_num - 1; i++)
  {
    MINCO_msg.inner_x[i] = pos(0, i + 1);
    MINCO_msg.inner_y[i] = pos(1, i + 1);
    MINCO_msg.inner_z[i] = pos(2, i + 1);
  }
  for (int i = 0; i < piece_num; i++){
    MINCO_msg.duration[i] = durs[i];
  }

}

void Navigator::commandTrajectoryServer()
{

}

// void Navigator::polyTrajMsgToPolyTraj(const traj_utils::PolyTrajPtr msg)
// {
//     if (msg->order != 5)
//     {
//       // Only support trajectory order equals 5 now!
//       return;
//     }
//     if (msg->duration.size() * (msg->order + 1) != msg->coef_x.size())
//     {
//       // WRONG trajectory parameters
//       return;
//     }

//     // The chunk of code below is just converting the received 
//     // trajectories into poly_traj::Trajectory type and storing it

//     // piece_nums is the number of Pieces in the trajectory 
//     int piece_nums = msg->duration.size();
//     std::vector<double> dura(piece_nums);
//     std::vector<poly_traj::CoefficientMat> cMats(piece_nums);
//     for (int i = 0; i < piece_nums; ++i)
//     {
//       int i6 = i * 6;
//       cMats[i].row(0) <<  msg->coef_x[i6 + 0], msg->coef_x[i6 + 1], msg->coef_x[i6 + 2],
//                           msg->coef_x[i6 + 3], msg->coef_x[i6 + 4], msg->coef_x[i6 + 5];
//       cMats[i].row(1) <<  msg->coef_y[i6 + 0], msg->coef_y[i6 + 1], msg->coef_y[i6 + 2],
//                           msg->coef_y[i6 + 3], msg->coef_y[i6 + 4], msg->coef_y[i6 + 5];
//       cMats[i].row(2) <<  msg->coef_z[i6 + 0], msg->coef_z[i6 + 1], msg->coef_z[i6 + 2],
//                           msg->coef_z[i6 + 3], msg->coef_z[i6 + 4], msg->coef_z[i6 + 5];

//       dura[i] = msg->duration[i];
//     }

//     be_traj_.reset(new poly_traj::Trajectory(dura, cMats));
//     be_traj_->setGlobalStartTime(msg->start_time.toSec());
// }