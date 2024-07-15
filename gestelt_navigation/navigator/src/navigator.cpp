#include <navigator/navigator.h>

/* Initialization methods */

void Navigator::init(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{ 
  while (ros::Time::now().toSec() <= 1 ){
    ROS_WARN("ros::Time is either simulated or not initialized");
  }

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
    poly_sfc_gen_ = std::make_unique<PolytopeSFC>(map_, ply_sfc_params_);
    poly_sfc_gen_->addPublishers(sfc_publisher_map_);
  }
  else if (sfc_type_ == SFCType::SPHERICAL) {
    sfc_generation_ = std::make_unique<SphericalSFC>(map_, sph_sfc_params_);
    sfc_generation_->addPublishers(sfc_publisher_map_);
  }

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
    ssfc_optimizer_ = std::make_unique<back_end::SphericalSFCOptimizer>();
    ssfc_optimizer_->setParam(pnh);
    ssfc_optimizer_->setEnvironment(map_);

    ssfc_optimizer_->setVisualizer(visualization_);
    ssfc_optimizer_->assignSwarmTrajs(swarm_local_trajs_);
  }
  else if (back_end_type_ == BackEndType::POLY) {
    polyhedron_sfc_optimizer_ = std::make_unique<back_end::PolyhedronSFCOptimizer>();
    polyhedron_sfc_optimizer_->setParam(pnh);
    polyhedron_sfc_optimizer_->setEnvironment(map_);

    polyhedron_sfc_optimizer_->setVisualizer(visualization_);
    polyhedron_sfc_optimizer_->assignSwarmTrajs(swarm_local_trajs_);
  }

  /* Initialize Timer */
  if (!debug_planning_){
    fe_plan_timer_ = nh.createTimer(ros::Duration(1.0/fe_planner_freq_), &Navigator::planFrontEndTimerCB, this);
  }

  hb_timer_ = nh.createTimer(ros::Duration(1.0/hb_freq_), &Navigator::heartbeatTimerCB, this);
  safety_checks_timer_ = nh.createTimer(ros::Duration(1.0/safety_check_freq_), &Navigator::safetyChecksTimerCB, this);
}

void Navigator::initParams(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
  /* Navigator params*/
  pnh.param("drone_id", drone_id_, -1);
  pnh.param("goal_tolerance", squared_goal_tol_, -1.0);
  squared_goal_tol_ = squared_goal_tol_*squared_goal_tol_; 
  pnh.param("safety_check_frequency", safety_check_freq_, -1.0);
  pnh.param("debug_planning", debug_planning_, false);
  pnh.param("time_to_col_threshold", time_to_col_threshold_, 0.8);
  pnh.param("receding_horizon_planning_dist", rhp_dist_, -1.0);
  pnh.param("receding_horizon_corridor_dist", rhc_dist_, -1.0);
  pnh.param("verbose_planning", verbose_planning_, false);
  
  pnh.param("heartbeat_frequency", hb_freq_, 20.0);

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
    case BackEndType::POLY:
      back_end_type_ = BackEndType::POLY;
      break;
    default:
      throw std::runtime_error("Invalid Back-End Planner type!");
  }

  /* Front end params */
  pnh.param("front_end/planner_frequency", fe_planner_freq_, -1.0);
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
    pnh.param("sfc/poly/debug_viz",      ply_sfc_params_.debug_viz, false);

    int cvx_decomp_type;
    pnh.param("sfc/poly/cvx_decomp_type", cvx_decomp_type, 2);
    ply_sfc_params_.cvx_decomp_type = static_cast<PolytopeSFC::CVXDecompType>(cvx_decomp_type);

    // Liu SFC Params
    pnh.param("sfc/poly/bbox_x", ply_sfc_params_.bbox_x, 1.0);
    pnh.param("sfc/poly/bbox_y", ply_sfc_params_.bbox_y, 2.0);
    pnh.param("sfc/poly/bbox_z", ply_sfc_params_.bbox_z, 1.0);

    // Toumieh SFC Params
    pnh.param("sfc/poly/poly_max", ply_sfc_params_.poly_hor, 10);
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
  // pnh.param("back_end/planner_frequency", be_planner_freq_, -1.0);
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
  front_end_plan_viz_pub_ = nh.advertise<visualization_msgs::Marker>("plan_viz", 1);
  front_end_publisher_map_["front_end/closed_list"] = nh.advertise<visualization_msgs::Marker>("closed_list_viz", 10);

  /* SFC */
  if (sfc_type_ == SFCType::POLYTOPE){
    sfc_publisher_map_["sfc/poly"] =  nh.advertise<decomp_ros_msgs::PolyhedronArray>("sfc/poly", 10);
    sfc_publisher_map_["sfc/ellipsoid"] =  nh.advertise<decomp_ros_msgs::EllipsoidArray>("sfc/ellipsoid", 10);
  }
  else if (sfc_type_ == SFCType::SPHERICAL) {
    sfc_publisher_map_["sfc_cand_points"] = nh.advertise<visualization_msgs::Marker>("sfc_cand_points", 10);
    sfc_publisher_map_["sfc_dist"] =  nh.advertise<visualization_msgs::MarkerArray>("sfc_dist", 10);
    sfc_publisher_map_["sfc_spherical"] = nh.advertise<visualization_msgs::MarkerArray>("sfc_spherical", 10);
    sfc_publisher_map_["sfc_waypoints"] = nh.advertise<visualization_msgs::Marker>("sfc_waypoints", 10);
    sfc_publisher_map_["sfc_samp_dir_vec"] = nh.advertise<visualization_msgs::MarkerArray>("sfc_samp_dir_vec", 10);
    sfc_publisher_map_["sfc_intxn_spheres"] =  nh.advertise<visualization_msgs::MarkerArray>("sfc_intxn_spheres", 10);

    // spherical_ssfc_pub_ = nh.advertise<gestelt_msgs::SphericalSFCTrajectory>("front_end/sfc_trajectory", 10);
  }

  if (debug_planning_){
    debug_start_sub_ = pnh.subscribe("debug/plan_start", 5, &Navigator::debugStartCB, this);
    debug_goal_sub_ = pnh.subscribe("debug/plan_goal", 5, &Navigator::debugGoalCB, this);
    plan_on_demand_sub_ = pnh.subscribe("plan_on_demand", 5, &Navigator::planOnDemandCB, this);

    dbg_ssfc_pub_ = nh.advertise<gestelt_debug_msgs::SFCTrajectory>("sfc/debug_trajectory", 10, true);

    debug_traj_pub_ = nh.advertise<gestelt_debug_msgs::BackEndTrajectoryDebug>(
      "back_end/debug_trajectory", 10, true); 
  }

  /* Back-end */
  be_traj_pub_ = nh.advertise<traj_utils::PolyTraj>("back_end/trajectory", 2); 
  swarm_minco_traj_pub_ = nh.advertise<traj_utils::MINCOTraj>("/swarm/global/minco", 10);
}

/* Timer callbacks */

void Navigator::heartbeatTimerCB(const ros::TimerEvent &e)
{
  // Publish heartbeat
  std_msgs::Empty empty_msg;
  heartbeat_pub_.publish(empty_msg);
}

void Navigator::planFrontEndTimerCB(const ros::TimerEvent &e)
{
  // Check if waypoint queue is empty
  if (waypoints_.empty()){
    return;
  }

  if (isGoalReached(cur_pos_, waypoints_.nextWP())){
    std::cout << " Goal reached at " << waypoints_.nextWP() << std::endl;
    // If goals is within a given tolerance, then pop this goal and plan next goal (if available)
    waypoints_.popWP();
    // Invalidate current sfc_traj
    // ssfc_ = nullptr;
    init_new_poly_traj_ = true; // Used in EGO Planner 

    global_traj_exists_ = false;
    local_traj_exists_ = false;

    return;
  }

  if (back_end_type_ == BackEndType::EGO){
    rhp_goal_pos_ = waypoints_.nextWP();
    requestBackEndPlan(rhp_goal_pos_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    return;
  }

  Eigen::Vector3d start_pos, start_vel, start_acc;

  double req_plan_t = ros::Time::now().toSec(); // time at which back end plan was requested
  
  // Sample the starting position from the back end trajectory
  if (!sampleBackEndTrajectory((*swarm_local_trajs_)[drone_id_], req_plan_t, start_pos, start_vel, start_acc)){
    // If we are unable to sample the back end trajectory, we set the starting position as the quadrotor's current position
    start_pos = cur_pos_;
    start_vel = cur_vel_;
    start_acc.setZero();
  }

  // // Get Receding Horizon Planning goal 
  // if (!getRHPGoal(waypoints_.nextWP(), start_pos, rhp_dist_, rhp_goal_pos_)){
  //   logError("Failed to get RHP goal");
  //   return;
  // }

  // Plan global naive trajectory using front-end path
  std::vector<Eigen::Vector3d> one_pt_wps;
  one_pt_wps.push_back(waypoints_.nextWP()); 
  planGlobalTrajWaypoints(start_pos, start_vel, start_acc,
                          one_pt_wps, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

  // Get RHP goal using global naive trajectory
  getRHPGoal(waypoints_.nextWP(), start_pos, rhp_dist_, rhp_goal_pos_, rhp_goal_vel_);

  // Display receding horizion goal point
  visualization_->displayGoalPoint(rhp_goal_pos_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

  if (!generateFrontEndPlan(start_pos, rhp_goal_pos_, ssfc_)){
    logError("Failed to generate front end plan!");
    return;
  }

  if (!requestBackEndPlan(rhp_goal_pos_, rhp_goal_vel_, Eigen::Vector3d::Zero())){
    logError("Request back-end plan failed!");
    return;
  }

}

void Navigator::safetyChecksTimerCB(const ros::TimerEvent &e)
{
  bool e_stop{true}, must_replan{true};
  // bool is_feasible{true};

  std::vector<ego_planner::LocalTrajData> swarm_local_trajs = *swarm_local_trajs_;

  if (!isTrajectorySafe(swarm_local_trajs, e_stop, must_replan)){
    if (e_stop){
      // logError("Activating emergency stop!");
      // pubTrajServerCmd(gestelt_msgs::Command::HOVER);
      // stopAllPlanning();
    }
    if (must_replan){
    }
  }

  // if (!isTrajectoryDynFeasible(&((*swarm_local_trajs_)[drone_id_]), is_feasible)){
  //   logError("Trajectory is infeasible!");
  //   if (!is_feasible){
  //   }
  // }

  // if (isTimeout(last_state_output_t_.toSec(), 0.5)) 
  // {
  //   logError("Time between UAV odom callback exceeded timeout of 0.5s, switching to HOVER!");
  //   pubTrajServerCmd(gestelt_msgs::Command::HOVER);
  //   stopAllPlanning();
  // }

}

/**
 * Planner methods
*/
void Navigator::planGlobalTrajWaypoints(
    const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc, 
      const std::vector<Eigen::Vector3d> &waypoints,
      const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
{
  poly_traj::MinJerkOpt globalMJO;
  Eigen::Matrix<double, 3, 3> headState, tailState;
  headState << start_pos, start_vel, start_acc;
  tailState << waypoints.back(), end_vel, end_acc;
  Eigen::MatrixXd innerPts;

  // Exclude the start waypoint
  if (waypoints.size() > 1)
  {

    innerPts.resize(3, waypoints.size());
    for (int i = 0; i < (int)waypoints.size()-1; ++i)
    {
      innerPts.col(i) = waypoints[i];
    }
  }

  globalMJO.reset(headState, tailState, waypoints.size());

  double des_vel = max_vel_;
  Eigen::VectorXd time_dur(waypoints.size()); // time duration vector

  for (int j = 0; j < 2; ++j) // re-try twice
  {
    for (size_t i = 0; i < waypoints.size(); ++i)
    {
      time_dur(i) = (i == 0) ? (waypoints[0] - start_pos).norm() / des_vel
                              : (waypoints[i] - waypoints[i - 1]).norm() / des_vel;
    }

    globalMJO.generate(innerPts, time_dur);

    if (globalMJO.getTraj().getMaxVelRate() < max_vel_ ||
        start_vel.norm() > max_vel_ ||
        end_vel.norm() > max_vel_)
    {
      break;
    }

    if (j == 2)
    {
      ROS_WARN("[Navigator::planGlobalTrajWaypoints] Global traj MaxVel = %f > set_max_vel", 
                globalMJO.getTraj().getMaxVelRate());
      std::cout << "headState=" << endl
                << headState << endl;
      std::cout << "tailState=" << endl
                << tailState << endl;
    }

    des_vel /= 1.5;
  }

  traj_.setGlobalTraj(globalMJO.getTraj(), ros::Time::now().toSec());

  global_traj_exists_ = true;

  return;
}

bool Navigator::generateFrontEndPlan(
  const Eigen::Vector3d& start_pos, const Eigen::Vector3d& goal_pos,
  std::shared_ptr<SSFC::SFCTrajectory> sfc_traj)
{
  logInfo(str_fmt("generateFrontEndPlan() from (%f, %f, %f) to (%f, %f, %f)",
    start_pos(0), start_pos(1), start_pos(2),
    goal_pos(0), goal_pos(1), goal_pos(2))
  );

  tm_front_end_plan_.start();

  if (!front_end_planner_->generatePlan(start_pos, goal_pos)){
    logInfo(str_fmt("FRONT END FAILED!!!! front_end_planner_->generatePlan() from (%f, %f, %f) to (%f, %f, %f)",
      start_pos(0), start_pos(1), start_pos(2),
      goal_pos(0), goal_pos(1), goal_pos(2))
    );

    // viz_helper::publishClosedList(front_end_planner_->getClosedList(), "world", closed_list_viz_pub_);
    return false;
  }

  tm_front_end_plan_.stop(verbose_planning_);

  front_end_path_ = front_end_planner_->getPathPosRaw();
  // std::vector<Eigen::Vector3d> dmp_search_region = front_end_planner_->getDMPSearchRegion();
  // std::vector<Eigen::Vector3d> closed_list = front_end_planner_->getClosedList();

  // Publish front end plan
  viz_helper::publishFrontEndPath(front_end_path_, "world", front_end_plan_viz_pub_) ;
  // viz_helper::publishClosedList(dmp_search_region, "world", closed_list_viz_pub_);
  // viz_helper::publishClosedList(closed_list, "world", closed_list_viz_pub_);

  tm_sfc_plan_.start();

  double req_plan_t = ros::Time::now().toSec(); // time at which back end plan was requested


  // Generate safe flight corridor from front end path
  if (sfc_type_ == SFCType::POLYTOPE){
    if (!poly_sfc_gen_->generateSFC(front_end_path_, 
                                    false, rhc_dist_, 
                                    start_pos, req_plan_t))
    {
      logError("Failed to generate safe flight corridor!");
      return false;
    }
  }
  else if (sfc_type_ == SFCType::SPHERICAL) {
    if (!sfc_generation_->generateSFC(front_end_path_, 
                                      false, rhc_dist_, 
                                      start_pos, req_plan_t))
    {
      logError("Failed to generate safe flight corridor!");
      return false;
    }

  }

  tm_sfc_plan_.stop(verbose_planning_);

  if (sfc_type_ == SFCType::POLYTOPE){
    h_poly_ = poly_sfc_gen_->getPolySFCHyperplanes();
    v_poly_ = poly_sfc_gen_->getPolySFCVertices();
  }
  else if (sfc_type_ == SFCType::SPHERICAL) {
    ssfc_ = std::make_shared<SSFC::SFCTrajectory>(sfc_generation_->getSSFCTrajectory(req_plan_t));
  }

  // logInfo(str_fmt("Number of waypoints in front-end path: %ld", front_end_path_.size()));
  // logInfo(str_fmt("Size of closed list (expanded nodes): %ld", closed_list.size()));
  // logInfo(str_fmt("[SFC] Number of spheres in SFC Spherical corridor: %ld", sfc_traj.spheres.size()));
  // logInfo(str_fmt("[SFC] Number of waypoints: %ld", sfc_traj.waypoints.size()));
  // logInfo(str_fmt("[SFC] Number of time segment durations: %ld", sfc_traj.segs_t_dur.size()));

  return true;
}

bool Navigator::requestBackEndPlan(
  const Eigen::Vector3d &goal_pos, const Eigen::Vector3d &goal_vel, const Eigen::Vector3d &goal_acc)
{
  // Check 1: if waypoint queue is empty
  if (waypoints_.empty()){
    return false;
  }

  // Check 2: if safe flight corridor trajectory exists
  if (sfc_type_ == SFCType::SPHERICAL && ssfc_ == nullptr)
  {
    return false;
  }


  bool plan_success = false;      // Indicates if back-end optimization is successful
  bool valid_mjo = false;         // Indicates if there is a valid MJO returned from the back-ends
  poly_traj::MinJerkOpt mjo_opt;  // Optimal MINCO trajectory 

  tm_back_end_plan_.start();
  for (int itr = 0; itr < optimizer_num_retries_; itr++) // For each back-end optimization retry
  {
    Eigen::Vector3d start_pos, start_vel, start_acc;

    double req_plan_t = ros::Time::now().toSec(); // time at which back end plan was requested

    if (!sampleBackEndTrajectory((*swarm_local_trajs_)[drone_id_], req_plan_t, start_pos, start_vel, start_acc))
    {
      start_pos = cur_pos_;
      start_vel = cur_vel_;
      start_acc.setZero();
    }

    // start_pos = cur_pos_;
    // start_vel = cur_vel_;
    // start_acc.setZero();


    Eigen::Matrix3d startPVA, endPVA;   // Boundary start and end condition: Matrix consisting of 3d (position, velocity acceleration) 
    startPVA << start_pos, start_vel, start_acc;            // Start (position, velocity, acceleration)
    endPVA << goal_pos, goal_vel, goal_acc;  // Goal (P)

  // logInfo(str_fmt("generateBackEndPlan() from (%f, %f, %f) to (%f, %f, %f)", 
  //   start_pos(0), start_pos(1), start_pos(2), 
  //   goal_pos(0), goal_pos(1), goal_pos(2)));

    if (back_end_type_ == BackEndType::EGO){
      plan_success = EGOOptimize(startPVA, endPVA, mjo_opt);
    }
    else if (back_end_type_ == BackEndType::SSFC){
      plan_success = SSFCOptimize(startPVA, endPVA, req_plan_t, ssfc_, mjo_opt);
    }
    else if (back_end_type_ == BackEndType::POLY){
      plan_success = PolySFCOptimize(startPVA, endPVA, req_plan_t, 
                                    v_poly_, h_poly_, mjo_opt, valid_mjo);
    }

    if (plan_success)
    {
      Eigen::MatrixXd cstr_pts_mjo_opt = mjo_opt.getInitConstraintPoints(num_cstr_pts_per_seg_);
      visualization_->displayOptimalMJO(cstr_pts_mjo_opt, 0);
      break;
    }
    else if (valid_mjo)
    {
      logError(str_fmt("Trajectory optimization unsuccessful! Number retries left: %d", 
        optimizer_num_retries_ - itr));
      Eigen::MatrixXd cstr_pts_mjo_opt = mjo_opt.getInitConstraintPoints(num_cstr_pts_per_seg_);
      visualization_->displayFailedList(cstr_pts_mjo_opt, 0);
    }
  }
  tm_back_end_plan_.stop(verbose_planning_);

  if (!plan_success){
    return false;
  }

  /* Save and publish messages */

  traj_utils::PolyTraj poly_msg; 
  traj_utils::MINCOTraj MINCO_msg; 

  mjoToMsg(mjo_opt, ros::Time::now().toSec(), poly_msg, MINCO_msg);
  be_traj_pub_.publish(poly_msg); // (In drone origin frame) Publish to other nodes on the current drone for trajectory execution
  swarm_minco_traj_pub_.publish(MINCO_msg); // (In world frame) Broadcast to all other drones for replanning to optimize in avoiding swarm collision

  // Update optimized trajectory 
  (*swarm_local_trajs_)[drone_id_] = getLocalTraj(
    mjo_opt, ros::Time::now().toSec(), 
    num_cstr_pts_per_seg_, 
    traj_id_, drone_id_);

  // set local trajectory
  traj_.local_traj = (*swarm_local_trajs_)[drone_id_];
  local_traj_exists_ = true;
  traj_id_++; // Increment trajectory id

  return true;
}

bool Navigator::PolySFCOptimize(const Eigen::Matrix3d& startPVA, const Eigen::Matrix3d& endPVA, 
                    const double& req_plan_time,
                    const std::vector<Eigen::Matrix3Xd>& v_poly, const std::vector<Eigen::MatrixX4d>& h_poly,
                    poly_traj::MinJerkOpt& mjo_opt, bool& valid_mjo)
{
  bool plan_success{false};

  // /***************************/
  /* 1. Generate initial path */
  // /***************************/
  Eigen::Matrix3Xd initial_path;

  // if (local_traj_exists_ && global_traj_exists_)
  // {
  //   std::cout << "==== Using previous trajectory as initial! ====" << std::endl;

  //   auto start_pos = startPVA.col(0);
  //   auto start_vel = startPVA.col(1);
  //   auto start_acc = startPVA.col(2);

  //   auto local_target_pos = endPVA.col(0);
  //   auto local_target_vel = endPVA.col(1);

  //   // et_local_traj_start: Time elapsed since local trajectory has started
  //   double et_local_traj_start = ros::Time::now().toSec() - traj_.local_traj.start_time;
  //   // t_to_local_traj_end: Time left to complete trajectory
  //   double t_to_local_traj_end = traj_.local_traj.duration - et_local_traj_start;

  //   if (traj_.global_traj.last_glb_t_of_lc_tgt < 0.0)
  //   {
  //     ROS_ERROR("[Navigator::PolySFCOptimize] You are initializing a trajectory from a previous optimal trajectory, but no previous trajectories up to now.");
  //     global_traj_exists_ = false;
  //     return false;
  //   }

  //   if (t_to_local_traj_end < 0 )
  //   { 
  //     logError(str_fmt("Drone %d: [EGOPlannerManager::computeInitState] Time elapsed since start of \
  //               local trajectory (%f) exceeds it's duration (%f). Terminating this planning instance", 
  //               drone_id_, et_local_traj_start, traj_.local_traj.duration));
  //     local_traj_exists_ = false;
  //     return false;
  //   }
  //   // t_to_local_tgt: Time to reach local target
  //   double t_to_local_tgt = t_to_local_traj_end + 
  //                         (traj_.global_traj.glb_t_of_lc_tgt - traj_.global_traj.last_glb_t_of_lc_tgt);

  //   // Number of pieces = remaining distance / piece length 
  //   int piece_nums = ceil((start_pos - local_target_pos).norm() / segment_length_);
  //   piece_nums = piece_nums < 2 ? 2: piece_nums; // ensure piece_nums is always minimally 2

  //   Eigen::Matrix3d headState, tailState;
  //   Eigen::MatrixXd innerPs(3, piece_nums - 1);
  //   // piece_dur_vec: Vector of time duration of each piece. 
  //   //    Each duration assumed to be equal, value = time to local target / piece_num 
  //   Eigen::VectorXd piece_dur_vec = Eigen::VectorXd::Constant(piece_nums, t_to_local_tgt / piece_nums);

  //   headState << start_pos, start_vel, start_acc;
  //   tailState << local_target_pos, local_target_vel, Eigen::Vector3d::Zero();

  //   /* Generate inner waypoints based on time per piece */
  //   double t = piece_dur_vec(0); // Set start time t to first piece  
  //   for (int i = 0; i < piece_nums - 1; ++i) // For each segment
  //   {
  //     // Timeline: t ----> t_to_local_traj_end ----> t_to_local_tgt

  //     // if not yet end of local trajectory
  //     if (t < t_to_local_traj_end) 
  //     {
  //       innerPs.col(i) = traj_.local_traj.traj.getPos(t + et_local_traj_start);
  //     }
  //     // if not yet reached local target but exceed local trajectory duration
  //     //    we need to use global trajectory 
  //     else if (t <= t_to_local_tgt) 
  //     {
  //       // glb_t: t - time to local trajectory end + (time last local target was set - start time of global trajectory)
  //       double glb_t = t - t_to_local_traj_end + traj_.global_traj.last_glb_t_of_lc_tgt - traj_.global_traj.global_start_time;
  //       innerPs.col(i) = traj_.global_traj.traj.getPos(glb_t);
  //     }
  //     else
  //     {
  //       // time t should not exceed t_to_local_Tgt
  //       ROS_ERROR("Drone %d: [Navigator::PolySFCOptimize] t=%.2f, t_to_local_traj_end=%.2f, t_to_local_tgt=%.2f", drone_id_, t, t_to_local_traj_end, t_to_local_tgt);
  //     }

  //     t += piece_dur_vec(i + 1);
  //   }
  //   poly_traj::MinJerkOpt initMJO;

  //   initMJO.reset(headState, tailState, piece_nums);
  //   initMJO.generate(innerPs, piece_dur_vec);

  //   initial_path = initMJO.getInitConstraintPoints(num_cstr_pts_per_seg_);

  //   // Publish as "back_end/dbg/initial_poly_path"
  //   visualization_->displayInitialPolyPath(initial_path, 0);
  // }
  // else {
  //   if (!polyhedron_sfc_optimizer_->genInitialSFCTrajectory(startPVA.col(0), endPVA.col(0),
  //                                                     v_poly, 0.01, initial_path))
  //   {
  //     logInfo("PolySFCOptimize: Failed to generate initial SFC trajectory");
  //     valid_mjo = false;
  //     return false;
  //   }

  //   // Publish as "back_end/dbg/initial_poly_path"
  //   visualization_->displayInitialPolyPath(initial_path, 0);
  // }

  if (!polyhedron_sfc_optimizer_->genInitialSFCTrajectory(startPVA.col(0), endPVA.col(0),
                                                    v_poly, 0.01, initial_path))
  {
    logInfo("PolySFCOptimize: Failed to generate initial SFC trajectory");
    valid_mjo = false;
    return false;
  }

  // Publish as "back_end/dbg/initial_poly_path"
  visualization_->displayInitialPolyPath(initial_path, 0);

  // /***************************/
  /* 2. Pre-processing for optimization */
  // /***************************/

  // DEFINITION: super-segment: The segments between control points (their length can vary and depends on the initial path generation)
  // DEFINITION: segment: The segments between control points (Length is fixed by user-defined parameter)

  int num_polyhedrons = h_poly.size();
  // cp_deltas: change in control points position
  Eigen::Matrix3Xd cp_deltas = initial_path.rightCols(num_polyhedrons) - initial_path.leftCols(num_polyhedrons);
  // segs_in_super_seg: Elements are indexed by super-segment index and the value is number of segments belonging to that super-segment 
  //                    = (diff between waypoints / length per piece)
  Eigen::VectorXi segs_in_super_seg = (cp_deltas.colwise().norm() / INFINITY).cast<int>().transpose();
  segs_in_super_seg.array() += 1;

  int num_segs = segs_in_super_seg.sum(); // Total number of segments
  int num_cstr_pts = polyhedron_sfc_optimizer_->getNumCstrPtsPerSeg();
  int num_decis_var_t = num_segs; // num_decis_var_t: Number of decision variables for Time duration 
  int num_decis_var_bary = 0; // num_decis_var_bary: Number of decision variables for barycentric coordinates

  Eigen::VectorXi vPolyIdx, hPolyIdx; // Elements are indexed by segment index and the value is the polyhedron index the segment belongs to 
  vPolyIdx.resize(num_segs - 1); 
  hPolyIdx.resize(num_segs);
  for (int i = 0, j = 0, k; i < num_polyhedrons; i++) // For each polygon i
  {
      // k: Number of segments per super-segment
      k = segs_in_super_seg(i); 
      for (int l = 0; l < k; l++, j++)  // For each segment l on the super-segment
      {
          if (l < k - 1) // If not the last segment of the piece
          {
              vPolyIdx(j) = 2 * i; // segment j belongs to the (2*i)-th polyhedron (CURRENT) 
              num_decis_var_bary += v_poly[2 * i].cols(); // Add Number of vertices 
          }
          else if (i < num_polyhedrons - 1) // Last segment of the super-segment
          {
              vPolyIdx(j) = 2 * i + 1; // segment j belongs to the (2*i+1)-th polyhedron (NEXT)
              num_decis_var_bary += v_poly[2 * i + 1].cols(); // Add Number of vertices
          }
          hPolyIdx(j) = i;
      }
  }

  // /***************************/
  /* 3. Generate inner control points and time allocation vector */
  // /***************************/

  Eigen::Matrix3Xd inner_ctrl_pts(3, num_segs - 1);
  Eigen::VectorXd init_seg_dur(num_segs);

  Eigen::Vector3d a, b, c;
  for (int i = 0, j = 0, k = 0, l; i < segs_in_super_seg.size(); i++) // For each super-segment
  {
    l = segs_in_super_seg(i);
    a = initial_path.col(i);
    b = initial_path.col(i + 1);
    c = (b - a) / l; // Length of each segment in current super-segment
    init_seg_dur.segment(j, l).setConstant(c.norm() / polyhedron_sfc_optimizer_->getMaxVel());
    j += l;
    for (int m = 0; m < l; m++) // for each segment
    {
      if (i > 0 || m > 0)
      {
        inner_ctrl_pts.col(k++) = a + c * m;
      }
    }
  }

  // /***************************/
  /* 4. Create initial MJO for visualization*/
  // /***************************/

  poly_traj::MinJerkOpt initial_mjo; // Initial minimum jerk trajectory
  initial_mjo.reset(startPVA, endPVA, num_segs);
  initial_mjo.generate(inner_ctrl_pts, init_seg_dur);

  Eigen::MatrixXd init_cstr_pts = initial_mjo.getInitConstraintPoints(num_cstr_pts);

  std::vector<Eigen::Vector3d> initial_mjo_viz; // Visualization of the initial minimum jerk trajectory
  for (int i = 0; i < init_cstr_pts.cols(); ++i){
    initial_mjo_viz.push_back(init_cstr_pts.col(i));
  }
  visualization_->displayInitialMJO(initial_mjo_viz, 0.075, 0);

  // /***************************/
  // /*5:  Optimize plan
  // /***************************/
  double final_cost = 0; 

  plan_success = polyhedron_sfc_optimizer_->optimizeTrajectory( 
        startPVA, endPVA,                   // Start and end (pos, vel, acc)
        inner_ctrl_pts, init_seg_dur,
        vPolyIdx, v_poly,
        hPolyIdx, h_poly,
        num_decis_var_t, num_decis_var_bary,
        final_cost);                      

  // Optimized minimum jerk trajectory
  mjo_opt = polyhedron_sfc_optimizer_->getMJO();
  valid_mjo = true;


  return plan_success;
}

bool Navigator::SSFCOptimize(const Eigen::Matrix3d& startPVA, const Eigen::Matrix3d& endPVA, 
                  const double& req_plan_time,
                  std::shared_ptr<SSFC::SFCTrajectory> ssfc_ptr,
                  poly_traj::MinJerkOpt& mjo_opt){

  bool plan_success{false};

  SSFC::SFCTrajectory sfc_traj = *ssfc_ptr;

  int num_segs_traversed = sfc_traj.getNumSegmentsTraversed(req_plan_time);
  sfc_traj.prune(0, num_segs_traversed);

  int num_cstr_pts = ssfc_optimizer_->getNumCstrPtsPerSeg();
  int num_segs = sfc_traj.getNumSegments(); // Number of path segments

  /***************************/
  /*3:  Display initial MJO */
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
  // Eigen::MatrixXd init_inner_ctrl_pts_q = ssfc_optimizer_->f_B_ctrl_pts(
  //                                           sfc_traj.getInnerWaypoints(), 
  //                                           sfc_traj.getSpheresCenter(), sfc_traj.getSpheresRadii(),
  //                                           sfc_traj.getIntxnPlaneVec(), sfc_traj.getIntxnPlaneDist(),
  //                                           sfc_traj.getIntxnCenters(), sfc_traj.getIntxnCircleRadius());
  // visualization_->displayInitialCtrlPts_q(init_inner_ctrl_pts_q);

  // Display intersection sphere north vectors
  // visualization_->displaySphereIntxnVec(sfc_traj.getIntxnCenters(), sfc_traj.getIntxnPlaneVec());

  // Visualize initial constraint points in constrained q space
  // Eigen::MatrixXd cstr_pts_q = 
  //   ssfc_optimizer_->f_B_cstr_pts(init_cstr_pts, 
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

  plan_success = ssfc_optimizer_->optimizeTrajectory( 
        startPVA, endPVA,                   // Start and end (pos, vel, acc)
        sfc_traj.getInnerWaypoints(),       // Inner control points
        sfc_traj.getSegmentTimeDurations(), // Time durations of each segment
        sfc_traj.getSpheresCenter(), sfc_traj.getSpheresRadii(),   
        sfc_traj.getIntxnPlaneVec(), sfc_traj.getIntxnPlaneDist(),
        sfc_traj.getIntxnCenters(), sfc_traj.getIntxnCircleRadius(),
        final_cost);                      

  // Optimized minimum jerk trajectory
  mjo_opt = ssfc_optimizer_->getOptimizedMJO();

  /***************************/
  /* Print and display results for debugging
  /***************************/

  // /* Publish all intermediate paths */
  // visualization_->displayIntermediateMJO_xi(
  //   ssfc_optimizer_->intermediate_cstr_pts_xi_);

  // visualization_->displayIntermediateMJO_q(
  //   ssfc_optimizer_->intermediate_cstr_pts_q_);

  // // Print results for benchmarking
  // poly_traj::Trajectory optimized_traj = mjo_opt.getTraj();
  // double total_duration = optimized_traj.getDurations().sum();
  // double traj_length = 0.0;
  // double dt = 0.05;
  // for (double t = 0; t < total_duration - dt; t += dt)
  // {
  //   traj_length += (optimized_traj.getPos(t + dt) - optimized_traj.getPos(t)).norm();
  // }

  // double traj_jerk_cost = mjo_opt.getTrajJerkCost();
  // double trajectory_duration = mjo_opt.getTraj().getDurations().sum();

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

  // ssfc_optimizer_->opt_costs_.printAll();
  // logInfo(str_fmt("Final cost: %f", final_cost));


  return plan_success;
}

// ESDF-free Gradient optimization
bool Navigator::EGOOptimize(const Eigen::Matrix3d& startPVA,  
                            const Eigen::Matrix3d& endPVA, 
                            poly_traj::MinJerkOpt& mjo_opt){

  bool plan_success = false;

  if (init_new_poly_traj_){

    std::vector<Eigen::Vector3d> waypoints;
    waypoints.push_back(endPVA.col(0));

    // Generate initial minimum jerk trajectory starting from agent's current position with 0 starting/ending acceleration and velocity.
    plan_success = ego_optimizer_->planGlobalTrajWaypoints(
        mjo_opt,
        startPVA.col(0), startPVA.col(1), startPVA.col(2),
        waypoints, endPVA.col(1), endPVA.col(2));

    if (!plan_success)
    {
      logError("Unable to generate initial global minimum jerk trajectory!");
      return false;
    }
  }

  // Publishes to "goal_point"
  visualization_->displayGoalPoint(endPVA.col(0), Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

  std::vector<Eigen::Vector3d> global_traj = ego_optimizer_->traj_.getGlobalTrajViz(0.1); // visualize with time step of 0.1s

  // Publishes to "global_list"
  visualization_->displayGlobalPathList(global_traj, 0.1, 0);

  /*2:  Plan global trajectory */
  bool flag_randomPolyTraj = false; // Random polynomial coefficients

  Eigen::Vector3d local_target_pos, local_target_vel;

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

  init_new_poly_traj_ = !plan_success;
  mjo_opt = ego_optimizer_->ploy_traj_opt_->getOptimizedMJO_EGO();

  // init_new_poly_traj_ = false;

  return plan_success;
}

void Navigator::stopAllPlanning()
{
  fe_plan_timer_.stop();
  safety_checks_timer_.stop();
  hb_timer_.stop();
}

/**
 * Subscriber Callbacks
*/

void Navigator::odometryCB(const nav_msgs::OdometryConstPtr &msg)
{
  cur_pos_= Eigen::Vector3d{msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z};
  cur_vel_= Eigen::Vector3d{msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z};

  last_state_output_t_ = ros::Time::now();
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

bool Navigator::isTrajectorySafe(
  const std::vector<ego_planner::LocalTrajData>& swarm_local_trajs, 
  bool& e_stop, bool& must_replan)
{
  e_stop = false;
  must_replan = false;

  ego_planner::LocalTrajData traj = (swarm_local_trajs)[drone_id_];
  
  if (traj.traj_id < 0){ // Return if no local trajectory yet
    return true;
  }

  double traj_t_cur = ros::Time::now().toSec() - traj.start_time;

  if ( traj_t_cur >= traj.duration) // Time exceeded trajectory duration
  {
    return true;
  }

  // pts_chk: Vector of constraint points. Within each piece is a vector of (timestamp, position).
  std::vector<std::vector<std::pair<double, Eigen::Vector3d>>> pts_chk = traj.pts_chk; 
  size_t num_segs = pts_chk.size(); // Number of segments

  // pair of (seg_idx, t_in_segment / dur_segment)
  std::pair<int, double> idx_time_ratio_pair = traj.traj.getIdxTimeRatioAtTime(traj_t_cur);
  
  // idx_seg is segment index at current time: computed by multiplying segment index and it's fraction with the number of constraints per segment
  size_t idx_seg = floor((idx_time_ratio_pair.first + idx_time_ratio_pair.second) * num_cstr_pts_per_seg_);

  size_t idx_pt = 0; // idx of point within constraint piece
  for (; idx_seg < num_segs; idx_seg++) // iterate from segment at current time to last segment
  {
    for (idx_pt = 0; idx_pt < pts_chk[idx_seg].size(); ++idx_pt) // Iterate through each point in segment
    {
      if (pts_chk[idx_seg][idx_pt].first > traj_t_cur) // If time of checked point exceeds current time, check for potential collision from that particular index onwards
      {
        goto find_ij_start;
      }
    }
  }
  
  find_ij_start:;
    for (size_t i = idx_seg; i < num_segs; ++i) // Iterate through each segment
    {
      for (size_t j = idx_pt; j < pts_chk[i].size(); ++j) // Iterate through each point within segement
      {
        double t = pts_chk[i][j].first;             // time
        Eigen::Vector3d pos = pts_chk[i][j].second; //position

        bool in_collision = map_->getInflateOccupancy(pos, 0.2); // Indicates if occupancy grid is occupied

        if (!in_collision){ // If current position not in collision
          
          for (auto agent_traj : swarm_local_trajs) // Iterate through trajectories of other agents
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
            double t_X = t - (traj.start_time - agent_traj.start_time);
            if (t_X > 0 && t_X < agent_traj.duration) // If time t_X is within the duration of the k-th agent trajectory
            {
              Eigen::Vector3d agent_pos = agent_traj.traj.getPos(t_X);
              double inter_agent_dist = (pos - agent_pos).norm();

              if (inter_agent_dist < swarm_clearance_)
              {
                must_replan = true;

                double t_to_col = t - traj_t_cur; // time to collision

                logWarn(str_fmt("Clearance between drones %d and %d is below threshold of %f, at %f seconds later",
                        drone_id_, agent_traj.drone_id, inter_agent_dist, t_to_col));
                
                // e_stop: indicates that a collision is imminent in "time_to_col_threshold_" seconds
                if (t_to_col < time_to_col_threshold_){
                  logError(str_fmt("EMERGENCY, time to inter-agent collision is %f, which is less than threshold of %f!", 
                    t_to_col, time_to_col_threshold_));
                  e_stop =  true;
                }
                return false;
              }
            }
          }
        }
        else {  // If current trajectory results in collision
          must_replan = true;

          double t_to_col = t - traj_t_cur; // time to collision

          logWarn(str_fmt("Trajectory is colliding with obstacle %f seconds later!", t_to_col));
          
          if (t_to_col < time_to_col_threshold_){
            logError(str_fmt("EMERGENCY, time to static obstacle collision is %f, which is less than threshold of %f!", 
              t_to_col, time_to_col_threshold_));
            e_stop =  true;
          }

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
  const ego_planner::LocalTrajData& local_traj,
  const double& time_samp, 
  Eigen::Vector3d& pos, Eigen::Vector3d& vel, Eigen::Vector3d& acc)
{
  if (local_traj.start_time <= 0.0){ // It means my first planning has not started
    // ROS_INFO_THROTTLE(5.0, "[EGO Planner Adaptor] No trajectory received!");
    return false;
  }

  double t = time_samp - local_traj.start_time; // Get time t relative to start of trajectory

  if (t >= 0.0 && t < local_traj.traj.getTotalDuration())
  {
    pos = local_traj.traj.getPos(t);
    vel = local_traj.traj.getVel(t);
    acc = local_traj.traj.getAcc(t);

    return true;
  }

  return false;
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

void Navigator::mjoToMsg(const poly_traj::MinJerkOpt& mjo, const double& traj_start_time,
                          traj_utils::PolyTraj &poly_msg, traj_utils::MINCOTraj &MINCO_msg)
{

  poly_traj::Trajectory traj = mjo.getTraj();

  Eigen::VectorXd durs = traj.getDurations();
  int piece_num = traj.getPieceSize();
  poly_msg.drone_id = drone_id_;
  poly_msg.traj_id = traj_id_;
  poly_msg.start_time = ros::Time(traj_start_time);
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
  MINCO_msg.start_time = ros::Time(traj_start_time);
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
  const poly_traj::MinJerkOpt& mjo, const double& traj_start_time, 
  const int& num_cp, const int& traj_id, const int& drone_id)
{
  poly_traj::Trajectory traj = mjo.getTraj();

  ego_planner::LocalTrajData local_traj;

  local_traj.drone_id = drone_id;
  local_traj.traj_id = traj_id;
  local_traj.duration = traj.getTotalDuration();
  local_traj.start_pos = traj.getJuncPos(0);
  local_traj.start_time = traj_start_time;
  local_traj.traj = traj;
  local_traj.pts_chk = mjo.getTimePositionPairs(num_cp);

  return local_traj;
}

bool Navigator::getOptSegDur(
  const ego_planner::LocalTrajData& local_traj, std::vector<double>& time_dur)
{
  if (local_traj.start_time <= 0.0){ // It means my first planning has not started
    // ROS_INFO_THROTTLE(5.0, "[EGO Planner Adaptor] No trajectory received!");
    return false;
  }

  time_dur = local_traj.traj.getSegDurationsDouble();
  return true;
}

void Navigator::pubTrajServerCmd(const int& cmd)
{
  gestelt_msgs::Command cmd_msg;
  cmd_msg.header.stamp = ros::Time::now();
  cmd_msg.command = cmd;
  traj_server_command_pub_.publish(cmd_msg);
}
