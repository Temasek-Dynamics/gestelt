#include <back_end_planner/back_end_planner.h>

void BackEndPlanner::init(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{ 
  logInfo("Initialized back end planner");
  // double planner_freq;
  // pnh.param("back_end/planner_frequency", planner_freq, -1.0);
  pnh.param("drone_id", drone_id_, -1);
  pnh.param("back_end/planning_horizon", planning_horizon_, -1.0);
  pnh.param("back_end/num_replan_retries", num_replan_retries_, -1);
  
  /* Subscribers */
  sfc_traj_sub_ = nh.subscribe("front_end/sfc_trajectory", 5, &BackEndPlanner::sfcTrajectoryCB, this);
  odom_sub_ = nh.subscribe("odom", 5, &BackEndPlanner::odometryCB, this);
  // swarm_minco_traj_sub_ = nh.subscribe("/swarm/global/minco", 100,
  //                                       &BackEndPlanner::swarmMincoTrajCB,
  //                                       this,
  //                                       ros::TransportHints().tcpNoDelay());

  /* Publishers */
  plan_traj_pub_ = nh.advertise<traj_utils::PolyTraj>("back_end/trajectory", 10); 
  swarm_minco_traj_pub_ = nh.advertise<traj_utils::MINCOTraj>("/swarm/global/minco", 10);

  // Debugging topics
  debug_start_sub_ = pnh.subscribe("debug/plan_start", 5, &BackEndPlanner::debugStartCB, this);
  debug_goal_sub_ = pnh.subscribe("debug/plan_goal", 5, &BackEndPlanner::debugGoalCB, this);
  plan_on_demand_esdf_free_sub_ = pnh.subscribe("plan_on_demand/esdf_free", 5, &BackEndPlanner::planOnDemandESDFFree, this);

  // Initialize map
  // map_.reset(new GridMap);
  // map_->initMapROS(nh, pnh);

  visualization_.reset(new ego_planner::PlanningVisualization(pnh));

  // Initialize back end planner 
  back_end_planner_.reset(new ego_planner::EGOPlannerManager());
  // back_end_planner_ = std::make_unique<ego_planner::EGOPlannerManager>();
  back_end_planner_->initPlanModules(nh, pnh, visualization_);

  
  // Initialize own trajectory
  swarm_minco_trajs_ = std::make_shared<std::unordered_map<int, ego_planner::LocalTrajData>>();
  (*swarm_minco_trajs_)[drone_id_] = ego_planner::LocalTrajData();

  back_end_planner_->setSwarmTrajectories(swarm_minco_trajs_);

}

/**
 * Subscriber Callbacks
*/

void BackEndPlanner::swarmMincoTrajCB(const traj_utils::MINCOTrajConstPtr &msg)
{
  if (msg->drone_id == drone_id_){
    return; 
  }

  logInfo(str_fmt("Received swarm MINCO trajectory from drone %d", msg->drone_id));

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

  ego_planner::LocalTrajData swarm_minco_traj;
  mincoMsgToTraj(*msg, swarm_minco_traj);

  (*swarm_minco_trajs_)[msg->drone_id] = swarm_minco_traj;
}

void BackEndPlanner::odometryCB(const nav_msgs::OdometryConstPtr &msg)
{
  odom_mutex_.lock();
  // TODO Add mutex 
  cur_pos_= Eigen::Vector3d{msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z};
  cur_vel_= Eigen::Vector3d{msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z};
  odom_mutex_.unlock();
}

void BackEndPlanner::sfcTrajectoryCB(const gestelt_msgs::SphericalSFCTrajectoryConstPtr& msg){
  // logInfo(str_fmt("Received callback to SFC Trajectory with %ld waypoints", msg->waypoints.size()));
  std::vector<double> spheres_radius;
  std::vector<Eigen::Vector3d> spheres_center;

  for (auto sphere : msg->spheres){
    spheres_radius.push_back(sphere.radius);
    spheres_center.push_back(Eigen::Vector3d{sphere.center.x, sphere.center.y, sphere.center.z});
  }
  Eigen::VectorXd segs_t_dur(msg->segments_time_duration.size());
  for (size_t i = 0; i < msg->segments_time_duration.size(); i++){
    segs_t_dur(i) = msg->segments_time_duration[i];
  }

  // Convert to data types used by optimizer
  Eigen::Vector3d start_pos{msg->waypoints[0].x, msg->waypoints[0].y, msg->waypoints[0].z};
  odom_mutex_.lock();
  Eigen::Vector3d start_vel{cur_vel_(0), cur_vel_(1), cur_vel_(2)};  
  odom_mutex_.unlock();
  
  std::vector<Eigen::Vector3d> inner_wps;  
  // Inner points contain all waypoints except start and goal
  for (int i = 1; i < msg->waypoints.size() - 1; i++){
    inner_wps.push_back(Eigen::Vector3d{msg->waypoints[i].x, msg->waypoints[i].y, msg->waypoints[i].z});
  }
  Eigen::Vector3d goal_pos{msg->waypoints.back().x, msg->waypoints.back().y, msg->waypoints.back().z};

  if (!generatePlanSFC(start_pos, start_vel, 
                  inner_wps, segs_t_dur,
                  goal_pos, num_replan_retries_,
                  spheres_radius, spheres_center,
                  optimized_mjo_))
  {
    logError("Failed to optimize SFC Trajectory!");
    return;
  }

  // Get data from optimized trajectory and store in PolyTraj and MINCOTraj 
  traj_utils::PolyTraj poly_msg; 
  traj_utils::MINCOTraj MINCO_msg; 

  mjoToMsg(optimized_mjo_, poly_msg, MINCO_msg);
  plan_traj_pub_.publish(poly_msg); // (In drone origin frame) Publish to corresponding drone for execution
  swarm_minco_traj_pub_.publish(MINCO_msg); // (In world frame) Broadcast to all other drones for replanning to optimize in avoiding swarm collision

  ego_planner::LocalTrajData swarm_minco_traj;
  mincoMsgToTraj(MINCO_msg, swarm_minco_traj);

  (*swarm_minco_trajs_)[drone_id_] = swarm_minco_traj;
}

/* Planning methods */
bool BackEndPlanner::generatePlanSFC( const Eigen::Vector3d& start_pos, const Eigen::Vector3d& start_vel, 
                                      const std::vector<Eigen::Vector3d>& inner_wps, const Eigen::VectorXd& segs_t_dur,
                                      const Eigen::Vector3d& goal_pos, const int& num_opt_retries,
                                      const std::vector<double>& spheres_radius, const std::vector<Eigen::Vector3d>& spheres_center,
                                      poly_traj::MinJerkOpt& optimized_mjo)
{
  // logInfo(str_fmt("Generating plan from SFC from (%f, %f, %f) to (%f, %f, %f)", 
  //   start_pos(0), start_pos(1), start_pos(2), 
  //   goal_pos(0), goal_pos(1), goal_pos(2)));
  
  bool plan_success = false;
  int num_segs = inner_wps.size()+ 1;// Number of path segments

  Eigen::Vector3d start_acc;
  Eigen::Vector3d goal_vel;
  Eigen::Vector3d goal_acc;

  start_acc.setZero();
  goal_vel.setZero();
  goal_acc.setZero();

  // 1) Use waypoints from bubble planner to form an initial minimum jerk trajectory
  // 2) Obtain (inner waypoints, start, end, durations, constraint points) from initial minimum jerk trajectory
  // 3) Optimize plan

  // for (int i = 0; i < inner_wps.size(); i++) {
    // std::cout << "Idx " << i  << ": "<< inner_wps[i].transpose() << std::endl;
  // }

  // Eigen::Vector3d local_target_pos, local_target_vel;

  // If this is the first time planning has been called, then initialize a random polynomial
  for (int i = 0; i < num_opt_retries; i++)
  {
    /***************************/
    /*2:  Set local target (For receding horizon planning) */
    /***************************/

    // Get local target based on planning horizon
    // back_end_planner_->getLocalTarget(
    //     planning_horizon_, start_pos, goal_pos,
    //     local_target_pos, local_target_vel,
    //     touch_goal);

    /***************************/
    /*3:  Get minimum jerk trajectory */
    /***************************/

    poly_traj::MinJerkOpt initial_mjo; // Initial minimum jerk trajectory optimizer

    back_end_planner_->generateMinJerkTraj( start_pos, start_vel, start_acc, // Start states 
                                            inner_wps, // Inner waypoints
                                            goal_pos, goal_vel, // Goal states
                                            segs_t_dur, // Time duration of segments
                                            initial_mjo); // Initial Minimum jerk trajectory

    int num_constr_pts = 5;
    Eigen::MatrixXd cstr_pts_mjo = initial_mjo.getInitConstraintPoints(num_constr_pts);

    // for (int i = 0; i < 2; i++) {
      // std::cout << "Idx " << i  << ": "<< initial_mjo.get_b().block<6, 3>(i * 6, 0) << std::endl;
    // }

    std::vector<Eigen::Vector3d> initial_mjo_viz; // Visualization of the initial minimum jerk trajectory
    for (int i = 0; i < cstr_pts_mjo.cols(); ++i){
      // std::cout << "Constraint_pt " << i << ": " << cstr_pts_mjo.col(i).transpose() << std::endl;
      initial_mjo_viz.push_back(cstr_pts_mjo.col(i));
    }
    visualization_->displayInitialMinJerkTraj(initial_mjo_viz, 0.075, 0);

    /***************************/
    /*4:  Optimize plan
    /***************************/

    plan_success = back_end_planner_->optimizeMJOTraj(initial_mjo, optimized_mjo, spheres_radius, spheres_center);

    // Print results for benchmarking
    double traj_length = back_end_planner_->getTrajectoryLength(optimized_mjo);
    double traj_jerk_cost = optimized_mjo.getTrajJerkCost();
    double trajectory_duration = back_end_planner_->getTrajectoryDuration(optimized_mjo);

    // logInfo(str_fmt("Trajectory: Length(%f), Jerk Cost(%f), Duration(%f)", 
    //   traj_length, traj_jerk_cost, trajectory_duration));

    Eigen::MatrixXd cstr_pts_optimized_mjo = optimized_mjo.getInitConstraintPoints(num_constr_pts);
    if (plan_success)
    {
      visualization_->displayOptimalList(cstr_pts_optimized_mjo, 0);
      break;
    }
    else{
      logError(str_fmt("Trajectory optimization unsuccessful! Number retries left: %d", 
        num_opt_retries - i));
      visualization_->displayFailedList(cstr_pts_optimized_mjo, 0);
    }
  }

  if (!plan_success)
  {
    logError("back_end_planner_->reboundReplan not successful");
    return false;
  }

  return true;
}

bool BackEndPlanner::generatePlanESDFFree(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& start_vel, 
                                          const Eigen::Vector3d& goal_pos, const int& num_opt_retries)
{
  logInfo("generatePlanESDFFree");
  Eigen::Vector3d start_acc;
  Eigen::Vector3d goal_vel;
  Eigen::Vector3d goal_acc;

  start_acc.setZero();
  goal_vel.setZero();
  goal_acc.setZero();

  /*1:  Plan initial minimum jerk trajectory */
  poly_traj::MinJerkOpt globalMJO; // Global minimum jerk trajectory


  std::vector<Eigen::Vector3d> waypoints;
  waypoints.push_back(goal_pos);
  // Generate initial minimum jerk trajectory starting from agent's current position with 0 starting/ending acceleration and velocity.
  bool plan_success = back_end_planner_->planGlobalTrajWaypoints(
      globalMJO,
      start_pos, start_vel, start_acc,
      waypoints, goal_vel, goal_acc);

  // Publishes to "goal_point"
  visualization_->displayGoalPoint(goal_pos, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

  if (!plan_success)
  {
    logError("Unable to generate initial global minimum jerk trajectory!");
    return false;
  }

  back_end_planner_->traj_.setGlobalTraj(globalMJO.getTraj(), ros::Time::now().toSec());


  std::vector<Eigen::Vector3d> global_traj = back_end_planner_->traj_.getGlobalTrajViz(0.1);

  // Publishes to "global_list"
  visualization_->displayGlobalPathList(global_traj, 0.1, 0);


  /*2:  Plan global trajectory */
  bool flag_polyInit = true; // Initialize new polynomial
  bool flag_randomPolyTraj = true; // Random polynomial coefficients
  bool touch_goal = false;

  Eigen::Vector3d local_target_pos;
  Eigen::Vector3d local_target_vel;

  // If this is the first time planning has been called, then initialize a random polynomial
  for (int i = 0; i < num_opt_retries; i++)
  {
    // Get local target based on planning horizon
    back_end_planner_->getLocalTarget(
        planning_horizon_, start_pos, goal_pos,
        local_target_pos, local_target_vel,
        touch_goal);


    // Optimizer plans to local target and goal
    plan_success = back_end_planner_->reboundReplan(
        start_pos, start_vel, start_acc, 
        local_target_pos, local_target_vel, 
        flag_polyInit, flag_randomPolyTraj, 
        touch_goal);

    if (plan_success)
    {
      // Print results for benchmarking
      poly_traj::MinJerkOpt optimized_mjo = back_end_planner_->getMinJerkOpt();
      
      double traj_length = back_end_planner_->getTrajectoryLength(optimized_mjo);
      double traj_jerk_cost = optimized_mjo.getTrajJerkCost();
      double trajectory_duration = back_end_planner_->getTrajectoryDuration(optimized_mjo);

      // logInfo(str_fmt("Trajectory: Length(%f), Jerk Cost(%f), Duration(%f)", 
      //   traj_length, traj_jerk_cost, trajectory_duration));

      // double max_speed =
      // double max_acc = 

      logInfo("Back-end planning successful!");
      break;
    }
  }

  if (!plan_success)
  {
    logError("back_end_planner_->reboundReplan not successful");
    return false;
  }

  return true;
}

void BackEndPlanner::mjoToMsg(const poly_traj::MinJerkOpt& mjo, 
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

void BackEndPlanner::mincoMsgToTraj(const traj_utils::MINCOTraj &msg, ego_planner::LocalTrajData& traj)
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

/* Checking methods */
bool BackEndPlanner::isPlanFeasible(const Eigen::Vector3d& waypoints){
  // Check occupancy of every waypoint
  return true;
}