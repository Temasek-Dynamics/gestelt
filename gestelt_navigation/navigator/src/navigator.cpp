#include <navigator/navigator.h>

/* Initialization methods */

void Navigator::init(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{ 
  // Reset all data used for checking later
  last_state_output_t_ = ros::Time::now();

  initParams(nh, pnh);
  initPublishers(nh, pnh);
  initSubscribers(nh, pnh);

  // Initialize map
  map_.reset(new GridMap);
  map_->initMapROS(nh, pnh);

  // Initialize visualizer 
  visualization_ = std::make_shared<ego_planner::PlanningVisualization>(nh);

  // Initialize front end planner 
  if (front_end_type_ == FrontEndType::JPS_AND_DMP){
    front_end_planner_ = std::make_unique<JPSWrapper>(map_, jps_params_);
  }
  else if (front_end_type_ == FrontEndType::ASTAR){
    front_end_planner_ = std::make_unique<AStarPlanner>(map_, astar_params_);
  }
  front_end_planner_->addPublishers(front_end_publisher_map_);

  // Initialize safe flight corridor generation
  if (sfc_type_ == SFCType::POLYTOPE){
    sfc_generation_ = std::make_unique<PolytopeSFC>(map_, ply_sfc_params_);

  }
  else if (sfc_type_ == SFCType::SPHERICAL) {
    sfc_generation_ = std::make_unique<SphericalSFC>(map_, sph_sfc_params_);
  }
  sfc_generation_->addPublishers(sfc_publisher_map_);

  // Initialize own trajectory
  swarm_local_trajs_ = std::make_shared<std::vector<ego_planner::LocalTrajData>>();
  for (int i = 0; i < max_drones_; i++){
    (*swarm_local_trajs_).push_back(ego_planner::LocalTrajData());
  }

  // Initialize back-end planner
  if (back_end_type_ == BackEndType::EGO){
    ego_optimizer_ = std::make_unique<ego_planner::EGOPlannerManager>(ego_params_);
    ego_optimizer_->initPlanModules(nh, pnh, map_, visualization_);
    ego_optimizer_->setSwarmTrajectories(swarm_local_trajs_);
  }
  else if (back_end_type_ == BackEndType::SSFC) {
    back_end_optimizer_ = std::make_unique<ego_planner::PolyTrajOptimizer>();
    back_end_optimizer_->setParam(pnh);
    back_end_optimizer_->setEnvironment(map_);

    back_end_optimizer_->setVisualizer(visualization_);
    back_end_optimizer_->assignSwarmTrajs(swarm_local_trajs_);
  }

  /* Initialize Timer */
  if (!debug_planning_){
    plan_timer_ = nh.createTimer(ros::Duration(1.0/planner_freq_), &Navigator::planTimerCB, this);
  }

  safety_checks_timer_ = nh.createTimer(ros::Duration(1.0/safety_check_freq_), &Navigator::safetyChecksTimerCB, this);
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
  pnh.param("time_to_col_threshold", time_to_col_threshold_, 0.8);
  pnh.param("receding_horizon_planning_dist", rhp_dist_, 7.5);

  /* Planner type*/
  int fe_type, sfc_type, be_type;
  pnh.param("front_end/planner_type", fe_type, 0);
  pnh.param("sfc/planner_type", sfc_type, 0);
  pnh.param("back_end/planner_type", be_type, 0);

  switch (static_cast<FrontEndType>(fe_type)) {
    case FrontEndType::JPS_AND_DMP:
      front_end_type_ = FrontEndType::JPS_AND_DMP;
      break;
    case FrontEndType::ASTAR:
      front_end_type_ = FrontEndType::ASTAR;
      break;
    default:
      throw std::runtime_error("Invalid Front-End Planner type!");
  } 

  switch (static_cast<SFCType>(sfc_type)) {
    case SFCType::SPHERICAL:
      sfc_type_ = SFCType::SPHERICAL;
      break;
    case SFCType::POLYTOPE:
      sfc_type_ = SFCType::POLYTOPE;
      break;
    default:
      throw std::runtime_error("Invalid SFC type!");
  } 

  switch (static_cast<BackEndType>(be_type)) {
    case BackEndType::SSFC:
      back_end_type_ = BackEndType::SSFC;
      break;
    case BackEndType::EGO:
      back_end_type_ = BackEndType::EGO;
      break;
    default:
      throw std::runtime_error("Invalid Back-End Planner type!");
  } 

  /* Front end params */
  if (front_end_type_ == FrontEndType::JPS_AND_DMP){
    pnh.param("front_end/jps/planner_verbose",      jps_params_.planner_verbose, false);
    pnh.param("front_end/jps/interpolate",          jps_params_.interpolate, false);
    pnh.param("front_end/jps/use_dmp",              jps_params_.use_dmp, false);
    
    pnh.param("front_end/jps/dmp_search_radius",    jps_params_.dmp_search_rad, 0.5);
    pnh.param("front_end/jps/dmp_potential_radius", jps_params_.dmp_pot_rad, 1.0);
    pnh.param("front_end/jps/dmp_collision_weight", jps_params_.dmp_col_weight, 0.1);
    pnh.param("front_end/jps/dmp_heuristic_weight", jps_params_.dmp_heuristic_weight, 0.0);
    pnh.param("front_end/jps/dmp_pow", jps_params_.dmp_pow, 1);
  }
  else if (front_end_type_ == FrontEndType::ASTAR){
    pnh.param("front_end/a_star/max_iterations", astar_params_.max_iterations, -1);
    pnh.param("front_end/a_star/debug_viz", astar_params_.debug_viz, false);

    pnh.param("front_end/a_star/tie_breaker", astar_params_.tie_breaker, -1.0);
    pnh.param("front_end/a_star/cost_function_type", astar_params_.cost_function_type, 2);
  }

  /* SFC params */
  if (sfc_type_ == SFCType::POLYTOPE){
    pnh.param("sfc/poly/max_iterations", ply_sfc_params_.max_itr, -1);
    pnh.param("sfc/poly/debug_viz",      ply_sfc_params_.debug_viz, false);

    pnh.param("sfc/poly/poly_max", ply_sfc_params_.poly_hor, 10);
    int cvx_decomp_type;
    pnh.param("sfc/poly/cvx_decomp_type", cvx_decomp_type, 2);
    ply_sfc_params_.cvx_decomp_type = static_cast<PolytopeSFC::CVXDecompType>(cvx_decomp_type);

    pnh.param("sfc/poly/num_expansion_itr", ply_sfc_params_.n_it_decomp, 60);

    rhp_buffer_ = 0.05;
  }
  else if (sfc_type_ == SFCType::SPHERICAL) {
    pnh.param("sfc/spherical/max_iterations", sph_sfc_params_.max_itr, -1);
    pnh.param("sfc/spherical/debug_viz",      sph_sfc_params_.debug_viz, false);

    pnh.param("sfc/spherical/max_sample_points", sph_sfc_params_.max_sample_points, -1);
    pnh.param("sfc/spherical/mult_stddev_x", sph_sfc_params_.mult_stddev_x, -1.0);
    pnh.param("sfc/spherical/mult_stddev_y", sph_sfc_params_.mult_stddev_y, -1.0);
    pnh.param("sfc/spherical/mult_stddev_z", sph_sfc_params_.mult_stddev_z, -1.0);

    pnh.param("sfc/spherical/W_cand_vol",       sph_sfc_params_.W_cand_vol, -1.0);
    pnh.param("sfc/spherical/W_intersect_vol",  sph_sfc_params_.W_intersect_vol, -1.0);
    pnh.param("sfc/spherical/W_progress",       sph_sfc_params_.W_progress, -1.0);

    pnh.param("sfc/spherical/min_sphere_vol", sph_sfc_params_.min_sphere_vol, -1.0);
    pnh.param("sfc/spherical/max_sphere_vol", sph_sfc_params_.max_sphere_vol, -1.0);

    pnh.param("sfc/spherical/spherical_buffer", sph_sfc_params_.spherical_buffer, 0.0);

    pnh.param("sfc/spherical/max_vel", sph_sfc_params_.max_vel, 3.0);
    pnh.param("sfc/spherical/max_acc", sph_sfc_params_.max_acc, 10.0);

    pnh.param("sfc/spherical/time_allocation_type", sph_sfc_params_.time_allocation_type, 0);


    rhp_buffer_ = sph_sfc_params_.spherical_buffer;
  }

  /* Back-end params */
  pnh.param("back_end/num_replan_retries", optimizer_num_retries_, -1);
  pnh.param("optimization/num_cstr_pts_per_seg", num_cstr_pts_per_seg_, -1);

  /* Back end params */
  if (back_end_type_ == BackEndType::EGO) {
    pnh.param("drone_id", ego_params_.drone_id, -1);
    pnh.param("optimization/max_vel", ego_params_.max_vel, -1.0);
    pnh.param("optimization/max_acc", ego_params_.max_acc, -1.0);
    pnh.param("back_end/ego/segment_length", ego_params_.seg_length, -1.0);
  }
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
  /* navigator */
  heartbeat_pub_ = nh.advertise<std_msgs::Empty>("planner/heartbeat", 5); 
  // To trajectory server
  traj_server_command_pub_ = nh.advertise<gestelt_msgs::Command>("traj_server/command", 5); 
  rhp_goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>("navigator/rhp_goal", 5); 

  /* Front-end */
  front_end_plan_viz_pub_ = nh.advertise<visualization_msgs::Marker>("plan_viz", 10);
  front_end_publisher_map_["front_end/closed_list"] = nh.advertise<visualization_msgs::Marker>("closed_list_viz", 10);

  /* SFC */
  if (sfc_type_ == SFCType::POLYTOPE){
    sfc_publisher_map_["sfc/poly"] =  nh.advertise<decomp_ros_msgs::PolyhedronArray>("sfc/poly", 10);
  }
  else if (sfc_type_ == SFCType::SPHERICAL) {
    sfc_publisher_map_["sfc_cand_points"] = nh.advertise<visualization_msgs::Marker>("sfc_cand_points", 10);
    sfc_publisher_map_["sfc_dist"] =  nh.advertise<visualization_msgs::MarkerArray>("sfc_dist", 10);
    sfc_publisher_map_["sfc_spherical"] = nh.advertise<visualization_msgs::MarkerArray>("sfc_spherical", 10);
    sfc_publisher_map_["sfc_waypoints"] = nh.advertise<visualization_msgs::Marker>("sfc_waypoints", 10);
    sfc_publisher_map_["sfc_samp_dir_vec"] = nh.advertise<visualization_msgs::MarkerArray>("sfc_samp_dir_vec", 10);
    sfc_publisher_map_["sfc_intxn_spheres"] =  nh.advertise<visualization_msgs::MarkerArray>("sfc_intxn_spheres", 10);

    spherical_sfc_traj_pub_ = nh.advertise<gestelt_msgs::SphericalSFCTrajectory>("front_end/sfc_trajectory", 10);
    // dbg_sfc_traj_pub_ = nh.advertise<gestelt_debug_msgs::SFCTrajectory>("sfc/debug_trajectory", 10, true);
  }

  if (debug_planning_){
    debug_start_sub_ = pnh.subscribe("debug/plan_start", 5, &Navigator::debugStartCB, this);
    debug_goal_sub_ = pnh.subscribe("debug/plan_goal", 5, &Navigator::debugGoalCB, this);
    plan_on_demand_sub_ = pnh.subscribe("plan_on_demand", 5, &Navigator::planOnDemandCB, this);
  }

  /* Back-end */
  debug_traj_pub_ = nh.advertise<gestelt_debug_msgs::BackEndTrajectoryDebug>(
    "back_end/debug_trajectory", 10, true); 
  be_traj_pub_ = nh.advertise<traj_utils::PolyTraj>("back_end/trajectory", 10); 
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

  double req_plan_time = ros::Time::now().toSec(); // Time at which plan was requested

  // Sample the starting position from the back end trajectory
  if (!sampleBackEndTrajectory(req_plan_time, start_pos_)){
    // If we are unable to sample the back end trajectory, we set the starting position as the quadrotor's current position
    start_pos_ = cur_pos_;
  }

  // Get Receding Horizon Planning goal 
  Eigen::Vector3d rhp_goal;
  if (!getRHPGoal(waypoints_.nextWP(), start_pos_, rhp_dist_, rhp_goal)){
    return;
  }

  // Publish RHP goal
  geometry_msgs::PoseStamped rhp_goal_msg;
  rhp_goal_msg.header.stamp = ros::Time::now();
  rhp_goal_msg.header.frame_id = "world";
  rhp_goal_msg.pose.position.x = rhp_goal(0);
  rhp_goal_msg.pose.position.y = rhp_goal(1);
  rhp_goal_msg.pose.position.z = rhp_goal(2);
  rhp_goal_pub_.publish(rhp_goal_msg);

  // Generate plan 
  plan(start_pos_, rhp_goal, req_plan_time);

  // Publish heartbeat
  std_msgs::Empty empty_msg;
  heartbeat_pub_.publish(empty_msg);
}

bool Navigator::getRHPGoal(
  const Eigen::Vector3d& global_goal, const Eigen::Vector3d& start_pos, 
  const double& rhp_dist, Eigen::Vector3d& rhp_goal) const
{
  if ((global_goal - start_pos).norm() <= rhp_dist){
    // If within distance of goal
    rhp_goal = global_goal;
    return true;
  }

  // Plan straight line to goal
  Eigen::Vector3d vec_to_goal = (global_goal - start_pos).normalized();
  rhp_goal = start_pos + (rhp_dist * vec_to_goal);
  
  // While RHP goal is in obstacle, backtrack.
  double dec = 0.15;
  double backtrack_dist = 0.0;
  while (map_->getInflateOccupancy(rhp_goal, rhp_buffer_ + map_->getInflation()))
  {
    rhp_goal -= dec * vec_to_goal;
    backtrack_dist += dec;
    if (backtrack_dist >= rhp_dist){
      return false;
    }
  }

  return true;
}

void Navigator::safetyChecksTimerCB(const ros::TimerEvent &e)
{
  bool e_stop{true}, must_replan{true};
  bool is_feasible{true};

  if (!isTrajectorySafe(swarm_local_trajs_, e_stop, must_replan)){
    if (e_stop){
      // logError("Activating emergency stop!");
      // pubTrajServerCmd(gestelt_msgs::Command::HOVER);
      // stopAllPlanning();
    }
    if (must_replan){
      // replan
    }
  }

  if (!isTrajectoryDynFeasible(&((*swarm_local_trajs_)[drone_id_]), is_feasible)){
    logError("Trajectory is infeasible!");
    if (!is_feasible){
      // replan
    }
  }

  if (isTimeout(last_state_output_t_.toSec(), 30.0)) 
  {
    logError("Time between UAV odom callback exceeded timeout of 0.5s, switching to HOVER!");
    pubTrajServerCmd(gestelt_msgs::Command::HOVER);
    stopAllPlanning();
  }

}

/**
 * Planner methods
*/

void Navigator::stopAllPlanning()
{
  plan_timer_.stop();
  safety_checks_timer_.stop();
}

bool Navigator::plan(
  const Eigen::Vector3d& start_pos, const Eigen::Vector3d& goal_pos, 
  const double& req_plan_time)
{
  SSFC::SFCTrajectory sfc_traj;

  /* Generate a front-end plan */
  if (!(back_end_type_ == BackEndType::EGO)){ // If not using EGO Planner
    if (!generateFrontEndPlan(start_pos, goal_pos, sfc_traj)){
      logError("Failed to generate front end plan!");
      return false;
    }
  }

  if (sfc_type_ == SFCType::POLYTOPE){
    return true;
  }

  /* Generate a back-end plan */
  poly_traj::MinJerkOpt optimized_mjo; // Optimized minimum jerk trajectory

  tm_back_end_plan_.start();
  // Generate a back-end trajectory from front-end plan
  if (!generateBackEndPlan( start_pos, cur_vel_, goal_pos, 
                            sfc_traj, 
                            optimized_mjo,
                            optimizer_num_retries_)){
    logError("Failed to generate back end plan!");
    return false;
  }
  tm_back_end_plan_.stop(verbose_planning_);

  /* Save and publish messages */
  be_traj_ = std::make_shared<poly_traj::Trajectory>(optimized_mjo.getTraj(req_plan_time));

  traj_utils::PolyTraj poly_msg; 
  traj_utils::MINCOTraj MINCO_msg; 

  mjoToMsg(optimized_mjo, req_plan_time, poly_msg, MINCO_msg);
  be_traj_pub_.publish(poly_msg); // (In drone origin frame) Publish to corresponding drone for execution
  swarm_minco_traj_pub_.publish(MINCO_msg); // (In world frame) Broadcast to all other drones for replanning to optimize in avoiding swarm collision

  // Update optimized trajectory 
  (*swarm_local_trajs_)[drone_id_] = getLocalTraj(
    optimized_mjo, req_plan_time, 
    num_cstr_pts_per_seg_, 
    traj_id_, drone_id_);

  traj_id_++; // Increment trajectory id

  return true;
}

bool Navigator::generateFrontEndPlan(
  const Eigen::Vector3d& start_pos, const Eigen::Vector3d& goal_pos,
  SSFC::SFCTrajectory& sfc_traj)
{
  // logInfo(str_fmt("generateFrontEndPlan() from (%f, %f, %f) to (%f, %f, %f)",
  //   start_pos(0), start_pos(1), start_pos(2),
  //   goal_pos(0), goal_pos(1), goal_pos(2))
  // );

  tm_front_end_plan_.start();

  if (!front_end_planner_->generatePlan(start_pos, goal_pos)){
    logError("Path generation failed!");
    // viz_helper::publishClosedList(front_end_planner_->getClosedList(), "world", closed_list_viz_pub_);
    return false;
  }

  tm_front_end_plan_.stop(verbose_planning_);

  std::vector<Eigen::Vector3d> front_end_path = front_end_planner_->getPathPosRaw();
  // std::vector<Eigen::Vector3d> dmp_search_region = front_end_planner_->getDMPSearchRegion();
  // std::vector<Eigen::Vector3d> closed_list = front_end_planner_->getClosedList();

  // Publish front end plan
  viz_helper::publishFrontEndPath(front_end_path, "world", front_end_plan_viz_pub_) ;
  // viz_helper::publishClosedList(dmp_search_region, "world", closed_list_viz_pub_);
  // viz_helper::publishClosedList(closed_list, "world", closed_list_viz_pub_);

  /* UNCOMMENT EVERYTHING FROM HERE*/

  tm_sfc_plan_.start();

  // Generate safe flight corridor from front end path
  if (!sfc_generation_->generateSFC(front_end_path)){
    logError("Failed to generate safe flight corridor!");
    return false;
  }

  tm_sfc_plan_.stop(verbose_planning_);

  if (sfc_type_ == SFCType::POLYTOPE){
    // poly_sfc = sfc_generation_->getPolySFC();
  }
  else if (sfc_type_ == SFCType::SPHERICAL) {
    sfc_traj = sfc_generation_->getSSFCTrajectory();

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
    // std::vector<std::vector<SSFC::Sphere>> sfc_sampled_spheres;
    // std::vector<Eigen::Vector3d> samp_dir_vec, guide_points_vec;

    // sfc_generation_->getSFCTrajectoryDebug(
    //   sfc_sampled_spheres, samp_dir_vec, guide_points_vec);

    // if (sfc_sampled_spheres.size() != samp_dir_vec.size() 
    //     || samp_dir_vec.size() != guide_points_vec.size() 
    //     || sfc_sampled_spheres.size() != guide_points_vec.size()){
    //   ROS_ERROR("ERROR, SFC Debug trajectory fields do not all have the same size (same number of semgments)!");
    // }

    // gestelt_debug_msgs::SFCTrajectory dbg_sfc_traj_msg;
    // for (size_t i = 0; i < guide_points_vec.size(); i++)
    // {
    //   gestelt_debug_msgs::SFCSegment segment;
    //   for (auto& sphere : sfc_sampled_spheres[i])
    //   {
    //     gestelt_msgs::Sphere sphere_msg;
    //     sphere_msg.radius = sphere.radius;
    //     sphere_msg.center.x = sphere.center(0);
    //     sphere_msg.center.y = sphere.center(1);
    //     sphere_msg.center.z = sphere.center(2);

    //     segment.sampled_spheres.push_back(sphere_msg);
    //   }

    //   segment.guide_point.x = guide_points_vec[i](0);
    //   segment.guide_point.y = guide_points_vec[i](1);
    //   segment.guide_point.z = guide_points_vec[i](2);

    //   segment.sampling_vector.x = samp_dir_vec[i](0);
    //   segment.sampling_vector.y = samp_dir_vec[i](1);
    //   segment.sampling_vector.z = samp_dir_vec[i](2);

    //   dbg_sfc_traj_msg.segments.push_back(segment);
    // }

    // for (size_t i = 0; i < front_end_path.size(); i++)
    // {
    //   geometry_msgs::Point front_end_pt;
    //   front_end_pt.x = front_end_path[i](0);
    //   front_end_pt.y = front_end_path[i](1);
    //   front_end_pt.z = front_end_path[i](2);

    //   dbg_sfc_traj_msg.front_end_path.push_back(front_end_pt);
    // }

    // dbg_sfc_traj_msg.sfc_spheres = sfc_traj_msg.spheres;
    // dbg_sfc_traj_msg.sfc_waypoints = sfc_traj_msg.waypoints;
    // dbg_sfc_traj_pub_.publish(dbg_sfc_traj_msg);

  }


  // logInfo(str_fmt("Number of waypoints in front-end path: %ld", front_end_path.size()));
  // logInfo(str_fmt("Size of closed list (expanded nodes): %ld", closed_list.size()));
  // logInfo(str_fmt("[SFC] Number of spheres in SFC Spherical corridor: %ld", sfc_traj.spheres.size()));
  // logInfo(str_fmt("[SFC] Number of waypoints: %ld", sfc_traj.waypoints.size()));
  // logInfo(str_fmt("[SFC] Number of time segment durations: %ld", sfc_traj.segs_t_dur.size()));

  return true;
}

bool Navigator::SSFCOptimize(const Eigen::Matrix3d& startPVA, const Eigen::Matrix3d& endPVA, 
                  SSFC::SFCTrajectory& sfc_traj,
                  poly_traj::MinJerkOpt& optimized_mjo){

  bool plan_success;

  int num_cstr_pts = back_end_optimizer_->getNumCstrPtsPerSeg();
  int num_segs = sfc_traj.getNumSegments(); // Number of path segments

  /***************************/
  /*3:  Get minimum jerk trajectory */
  /***************************/

  poly_traj::MinJerkOpt initial_mjo; // Initial minimum jerk trajectory
  initial_mjo.reset(startPVA, endPVA, num_segs);

  initial_mjo.generate(sfc_traj.getInnerWaypoints(), sfc_traj.getSegmentTimeDurations());

  Eigen::MatrixXd init_cstr_pts = initial_mjo.getInitConstraintPoints(num_cstr_pts);

  std::vector<Eigen::Vector3d> initial_mjo_viz; // Visualization of the initial minimum jerk trajectory
  for (int i = 0; i < init_cstr_pts.cols(); ++i){
    initial_mjo_viz.push_back(init_cstr_pts.col(i));
  }
  visualization_->displayInitialMJO(initial_mjo_viz, 0.075, 0);

  // Visualize initial control points in constrained q space
  Eigen::MatrixXd init_inner_ctrl_pts_q = back_end_optimizer_->f_B_ctrl_pts(
                                            sfc_traj.getInnerWaypoints(), 
                                            sfc_traj.getSpheresCenter(), sfc_traj.getSpheresRadii(),
                                            sfc_traj.getIntxnPlaneVec(), sfc_traj.getIntxnPlaneDist(),
                                            sfc_traj.getIntxnCenters(), sfc_traj.getIntxnCircleRadius());
  visualization_->displayInitialCtrlPts_q(init_inner_ctrl_pts_q);

  // Display intersection sphere north vectors
  visualization_->displaySphereIntxnVec(sfc_traj.getIntxnCenters(), sfc_traj.getIntxnPlaneVec());

  // Visualize initial constraint points in constrained q space
  // Eigen::MatrixXd cstr_pts_q = 
  //   back_end_optimizer_->f_B_cstr_pts(init_cstr_pts, 
  //                                     num_segs,
  //                                     num_cstr_pts,
  //                                     sfc_traj.getSpheresCenter(),
  //                                     sfc_traj.getSpheresRadii());
  // visualization_->displayInitialMJO_q(cstr_pts_q, 0); 

  /***************************/
  /*4:  Optimize plan
  /***************************/

  // Optimize trajectory!
  double final_cost = 0; 
  Eigen::MatrixXd opt_inner_pts; //optimized inner points

  plan_success = back_end_optimizer_->optimizeTrajectorySFC( 
        startPVA, endPVA,                   // Start and end (pos, vel, acc)
        sfc_traj.getInnerWaypoints(),       // Inner control points
        sfc_traj.getSegmentTimeDurations(), // Time durations of each segment
        sfc_traj.getSpheresCenter(), sfc_traj.getSpheresRadii(),   
        sfc_traj.getIntxnPlaneVec(), sfc_traj.getIntxnPlaneDist(),
        sfc_traj.getIntxnCenters(), sfc_traj.getIntxnCircleRadius(),
        final_cost);                      

  // Optimized minimum jerk trajectory
  optimized_mjo = back_end_optimizer_->getOptimizedMJO();

  /***************************/
  /* Print and display results for debugging
  /***************************/

  /* Publish all intermediate paths */
  visualization_->displayIntermediateMJO_xi(
    back_end_optimizer_->intermediate_cstr_pts_xi_);

  visualization_->displayIntermediateMJO_q(
    back_end_optimizer_->intermediate_cstr_pts_q_);

  // Print results for benchmarking
  poly_traj::Trajectory optimized_traj = optimized_mjo.getTraj();
  double total_duration = optimized_traj.getDurations().sum();
  double traj_length = 0.0;
  double dt = 0.05;
  for (double t = 0; t < total_duration - dt; t += dt)
  {
    traj_length += (optimized_traj.getPos(t + dt) - optimized_traj.getPos(t)).norm();
  }

  double traj_jerk_cost = optimized_mjo.getTrajJerkCost();
  double trajectory_duration = optimized_mjo.getTraj().getDurations().sum();

  // logInfo(str_fmt("Trajectory: Length(%f), Jerk Cost(%f), Duration(%f)", 
  //   traj_length, traj_jerk_cost, trajectory_duration));

  /* Publish back end trajectory for debugging with trajectory inspector */
  // gestelt_debug_msgs::BackEndTrajectoryDebug debug_traj_msg;
  // for (int i = 0; i < init_cstr_pts.cols(); ++i){
  //   geometry_msgs::Point pt;
  //   pt.x = init_cstr_pts.col(i)(0);
  //   pt.y = init_cstr_pts.col(i)(1);
  //   pt.z = init_cstr_pts.col(i)(2);

  //   debug_traj_msg.initial_mjo.push_back(pt);
  // }

  // debug_traj_msg.num_cp = num_cstr_pts;
  // debug_traj_msg.num_segs = initial_mjo.getNumSegs();
  // debug_traj_pub_.publish(debug_traj_msg);

  /* Visualize optimized mjo trajectories */
  // back_end_optimizer_->opt_costs_.printAll();
  // logInfo(str_fmt("Final cost: %f", final_cost));

  return plan_success;
}

// ESDF-free Gradient optimization
bool Navigator::EGOOptimize(const Eigen::Matrix3d& startPVA, const Eigen::Matrix3d& endPVA, 
                  poly_traj::MinJerkOpt& optimized_mjo){

  std::vector<Eigen::Vector3d> waypoints;
  waypoints.push_back(endPVA.col(0));
  // Generate initial minimum jerk trajectory starting from agent's current position with 0 starting/ending acceleration and velocity.
  bool plan_success = ego_optimizer_->planGlobalTrajWaypoints(
      optimized_mjo,
      startPVA.col(0), startPVA.col(1), startPVA.col(2),
      waypoints, endPVA.col(1), endPVA.col(2));

  // Publishes to "goal_point"
  visualization_->displayGoalPoint(endPVA.col(0), Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

  if (!plan_success)
  {
    logError("Unable to generate initial global minimum jerk trajectory!");
    return false;
  }

  ego_optimizer_->traj_.setGlobalTraj(optimized_mjo.getTraj(), ros::Time::now().toSec());

  std::vector<Eigen::Vector3d> global_traj = ego_optimizer_->traj_.getGlobalTrajViz(0.1);

  // Publishes to "global_list"
  visualization_->displayGlobalPathList(global_traj, 0.1, 0);

  /*2:  Plan global trajectory */
  bool flag_randomPolyTraj = false; // Random polynomial coefficients

  Eigen::Vector3d local_target_pos;
  Eigen::Vector3d local_target_vel;

  // Get local target based on planning horizon
  ego_optimizer_->getLocalTarget(
      rhp_dist_, startPVA.col(0), endPVA.col(0),
      local_target_pos, local_target_vel,
      touch_goal_);

  // Optimizer plans to local target and goal
  plan_success = ego_optimizer_->reboundReplan(
      startPVA.col(0), startPVA.col(1), startPVA.col(2), 
      local_target_pos, local_target_vel, 
      init_new_poly_traj_, flag_randomPolyTraj, 
      touch_goal_);

  optimized_mjo = ego_optimizer_->ploy_traj_opt_->getOptimizedMJO_EGO();

  init_new_poly_traj_ = false;

  return plan_success;
}

bool Navigator::generateBackEndPlan( 
  const Eigen::Vector3d& start_pos, const Eigen::Vector3d& start_vel, 
  const Eigen::Vector3d& goal_pos, 
  SSFC::SFCTrajectory& sfc_traj,
  poly_traj::MinJerkOpt& optimized_mjo,
  const int& num_retries)
{
  // logInfo(str_fmt("generateBackEndPlan() from (%f, %f, %f) to (%f, %f, %f)", 
  //   start_pos(0), start_pos(1), start_pos(2), 
  //   goal_pos(0), goal_pos(1), goal_pos(2)));
  
  visualization_->displayGoalPoint(goal_pos, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

  bool plan_success = false;

  Eigen::Vector3d start_acc;
  Eigen::Vector3d goal_vel;
  Eigen::Vector3d goal_acc;

  start_acc.setZero();
  goal_vel.setZero();
  goal_acc.setZero();

  Eigen::Matrix3d startPVA, endPVA;   // Boundary start and end condition: Matrix consisting of 3d (position, velocity acceleration) 
  startPVA << start_pos, start_vel, start_acc;            // Start (position, velocity, acceleration)
  endPVA << goal_pos, goal_vel, goal_acc;  // Goal (P)

  for (int itr = 0; itr < num_retries; itr++)
  {
    if (back_end_type_ == BackEndType::EGO){
      plan_success = EGOOptimize(startPVA, endPVA, optimized_mjo);
    }
    else if (back_end_type_ == BackEndType::SSFC){
      plan_success = SSFCOptimize(startPVA, endPVA, sfc_traj, optimized_mjo);
    }

    Eigen::MatrixXd cstr_pts_optimized_mjo = 
      optimized_mjo.getInitConstraintPoints(num_cstr_pts_per_seg_);

    if (plan_success)
    {
      visualization_->displayOptimalMJO(cstr_pts_optimized_mjo, 0);
      // visualization_->displayOptimalCtrlPts_q(back_end_optimizer_->getOptimizedCtrlPts());
      break;
    }
    else{
      logError(str_fmt("Trajectory optimization unsuccessful! Number retries left: %d", 
        num_retries - itr));
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

  last_state_output_t_ = ros::Time::now();
}

void Navigator::singleGoalCB(const geometry_msgs::PoseStampedConstPtr& msg)
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
  // TODO: Enable again
  // if (abs((t_now - msg->start_time).toSec()) > 0.25)
  // {
  //   if (abs((t_now - msg->start_time).toSec()) < 10.0) // 10 seconds offset, more likely to be caused by unsynced system time.
  //   {
  //     logWarn(str_fmt("Time-stamp diff. with Agent %d = %fs",
  //               msg->drone_id, (t_now - msg->start_time).toSec()));
  //   }
  //   else
  //   {
  //     logError(str_fmt("Time-stamp diff. with Agent %d = %fs, swarm time seems not synchronized. Trajectory will not be saved!",
  //               msg->drone_id, (t_now - msg->start_time).toSec()));
  //     return;
  //   }
  // }

  ego_planner::LocalTrajData traj;
  mincoMsgToTraj(*msg, traj);
  
  (*swarm_local_trajs_)[msg->drone_id] = traj;
}

/* Checking methods */

bool Navigator::isGoalReached(const Eigen::Vector3d& pos, const Eigen::Vector3d& goal){
  init_new_poly_traj_ = true;

  return (pos - goal).squaredNorm() < squared_goal_tol_;
}

bool Navigator::isTimeout(const double& last_state_time, const double& threshold)
{
  return (ros::Time::now().toSec() - last_state_time) >= threshold;
} 

bool Navigator::isTrajectorySafe(
  std::shared_ptr<std::vector<ego_planner::LocalTrajData>> swarm_local_trajs, 
  bool& e_stop, bool& must_replan)
{
  e_stop = false;
  must_replan = false;

  ego_planner::LocalTrajData *traj = &((*swarm_local_trajs)[drone_id_]);
  
  if (traj->traj_id <= 0){ // Return if no local trajectory yet
    return true;
  }

  double traj_t_cur = ros::Time::now().toSec() - traj->start_time;

  if ( traj_t_cur >= traj->duration) // Time exceeded
  {
    return true;
  }

  // pts_chk: Vector of constraint points. Within each piece is a vector of (timestamp, position).
  std::vector<std::vector<std::pair<double, Eigen::Vector3d>>> pts_chk = traj->pts_chk; 
  int num_segs = pts_chk.size(); // Number of segments

  // pair of (seg_idx, t_in_segment / dur_segment)
  std::pair<int, double> idx_time_ratio_pair = traj->traj.getIdxTimeRatioAtTime(traj_t_cur);
  
  // idx_seg is starting segment index, computed by multiplying segment index and it's fraction with the number of constraints per segment
  size_t idx_seg = floor((idx_time_ratio_pair.first + idx_time_ratio_pair.second) * num_cstr_pts_per_seg_);

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

        bool in_collision = map_->getInflateOccupancy(pos, 0.13); // Indicates if occupancy grid is occupied

        if (!in_collision){ // If free from obstacle 
          
          for (auto& agent_traj : *swarm_local_trajs) // Iterate through trajectories of other agents
          {
            // Check that it's not a null pointer
            // if (!(*swarm_local_trajs)[k]){
            //   logError(str_fmt("Swarm agent %d has empty trajectory", k))
            //   continue;
            // }

            if (agent_traj.drone_id == drone_id_ || agent_traj.drone_id < 0)
            {
              // Own trajectory
              continue;
            }

            // Calculate time for other drone
            double t_X = t - (traj->start_time - agent_traj.start_time);
            if (t_X > 0 && t_X < agent_traj.duration) // If time t_X is within the duration of the k-th agent trajectory
            {
              Eigen::Vector3d agent_pos = agent_traj.traj.getPos(t_X);
              double inter_agent_dist = (pos - agent_pos).norm();

              if (inter_agent_dist < swarm_clearance_)
              {
                double t_to_col = t - traj_t_cur; // time to collision

                logWarn(str_fmt("Clearance between drones %d and %d is below threshold of %f, at %f seconds later",
                        drone_id_, agent_traj.drone_id, inter_agent_dist, t_to_col));
                
                // e_stop: indicates that a collision is imminent in "time_to_col_threshold_" seconds
                if (t_to_col < time_to_col_threshold_){
                  logError(str_fmt("EMERGENCY, time to inter-agent collision is %f, which is less than threshold of %f!", 
                    t_to_col, time_to_col_threshold_));
                  e_stop =  true;
                }

                must_replan = true;

                return false;
              }
            }
          }
        }
        else {
          double t_to_col = t - traj_t_cur; // time to collision

          logWarn(str_fmt("Trajectory is colliding with obstacle %f seconds later!", t_to_col));
          
          if (t_to_col < time_to_col_threshold_){
            logError(str_fmt("EMERGENCY, time to static obstacle collision is %f, which is less than threshold of %f!", 
              t_to_col, time_to_col_threshold_));
            e_stop =  true;
          }

          must_replan = true;
          return false;
        }

      }
      idx_pt = 0; // reset to start of path
    }

    return true;
}

bool Navigator::isTrajectoryDynFeasible(ego_planner::LocalTrajData* traj, bool& is_feasible)
{
  // Check time durations
  return true;
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

void Navigator::mjoToMsg(const poly_traj::MinJerkOpt& mjo, const double& req_plan_time,
                          traj_utils::PolyTraj &poly_msg, traj_utils::MINCOTraj &MINCO_msg)
{

  poly_traj::Trajectory traj = mjo.getTraj();

  Eigen::VectorXd durs = traj.getDurations();
  int piece_num = traj.getPieceSize();
  poly_msg.drone_id = drone_id_;
  poly_msg.traj_id = traj_id_;
  poly_msg.start_time = ros::Time(req_plan_time);
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
  MINCO_msg.traj_id = traj_id_;
  MINCO_msg.start_time = ros::Time(req_plan_time);
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

ego_planner::LocalTrajData Navigator::getLocalTraj(
  poly_traj::MinJerkOpt& mjo, const double& req_plan_time, 
  const int& num_cp, const int& traj_id, const int& drone_id)
{
  poly_traj::Trajectory traj = mjo.getTraj();

  ego_planner::LocalTrajData local_traj;

  local_traj.drone_id = drone_id;
  local_traj.traj_id = traj_id;
  local_traj.duration = traj.getTotalDuration();
  local_traj.start_pos = traj.getJuncPos(0);
  local_traj.start_time = req_plan_time;
  local_traj.traj = traj;
  local_traj.pts_chk = mjo.getTimePositionPairs(num_cp);

  return local_traj;
}

void Navigator::pubTrajServerCmd(const int& cmd)
{
  gestelt_msgs::Command cmd_msg;
  cmd_msg.header.stamp = ros::Time::now();
  cmd_msg.command = cmd;
  traj_server_command_pub_.publish(cmd_msg);
}