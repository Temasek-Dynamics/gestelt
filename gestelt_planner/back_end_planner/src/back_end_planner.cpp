#include <back_end_planner/back_end_planner.h>

void BackEndPlanner::init(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{ 
  logInfo("Initialized back end planner");
  double planner_freq;
  pnh.param("back_end/planner_frequency", planner_freq, -1.0);
  pnh.param("back_end/planning_horizon", planning_horizon_, -1.0);
  pnh.param("back_end/num_replan_retries", num_replan_retries_, -1);
  
  /* Subscribers */
  // sfc_traj_sub_ = nh.subscribe("front_end_planner/sfc_trajectory", 5, &BackEndPlanner::sfcTrajectoryCB, this);
  debug_start_sub_ = pnh.subscribe("debug/plan_start", 5, &BackEndPlanner::debugStartCB, this);
  debug_goal_sub_ = pnh.subscribe("debug/plan_goal", 5, &BackEndPlanner::debugGoalCB, this);

  // Initialize map
  // map_.reset(new GridMap);
  // map_->initMap(nh, pnh);

  visualization_.reset(new ego_planner::PlanningVisualization(pnh));

  // Initialize back end planner 
  back_end_planner_.reset(new ego_planner::EGOPlannerManager());
  // back_end_planner_ = std::make_unique<ego_planner::EGOPlannerManager>();
  back_end_planner_->initPlanModules(nh, pnh, visualization_);
}

/**
 * Subscriber Callbacks
*/

void BackEndPlanner::debugStartCB(const geometry_msgs::PoseConstPtr &msg)
{
  logInfo(string_format("Received debug start (%f, %f, %f)", 
        msg->position.x,
        msg->position.y,
        msg->position.z));

  start_pos_(0) = msg->position.x ;
  start_pos_(1) = msg->position.y ;
  start_pos_(2) = msg->position.z ;

  start_vel_.setZero();
}

void BackEndPlanner::debugGoalCB(const geometry_msgs::PoseConstPtr &msg)
{
  logInfo(string_format("Received debug goal (%f, %f, %f)", 
        msg->position.x,
        msg->position.y,
        msg->position.z));

  goal_pos_ = Eigen::Vector3d{
        msg->position.x,
        msg->position.y,
        msg->position.z};

  if (!generatePlan(start_pos_, start_vel_, goal_pos_, num_replan_retries_)){
    logError("Unable to generate plan!");
  }
}

/**
 * @brief Callback to generate plan on demand
 * 
 * @param msg 
 */
// void BackEndPlanner::sfcTrajectoryCB(const gestelt_msgs::SphericalSFCConstPtr& msg){
//   msg->

//   generatePlan();
// }

/* Planning methods */

bool BackEndPlanner::generatePlan(
  const Eigen::Vector3d& start_pos, const Eigen::Vector3d& start_vel, 
  const Eigen::Vector3d& goal_pos, const int& num_retries)
{
  Eigen::Vector3d start_acc;
  start_acc.setZero();

  /*1:  Plan initial minimum jerk trajectory */

  std::vector<Eigen::Vector3d> waypoints;
  waypoints.push_back(goal_pos);
  // Generate initial minimum jerk trajectory starting from agent's current position with 0 starting/ending acceleration and velocity.
  bool plan_success = back_end_planner_->planGlobalTrajWaypoints(
      start_pos, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
      waypoints, 
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

  // Publishes to "goal_point"
  visualization_->displayGoalPoint(goal_pos, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

  if (!plan_success)
  {
    logError("Unable to generate global trajectory! Undefined actions!");
    return false;
  }
  constexpr double step_size_t = 0.1;
  int i_end = floor(back_end_planner_->traj_.global_traj.duration / step_size_t);
  // global_traj is only used for visualization
  vector<Eigen::Vector3d> global_traj(i_end);
  for (int i = 0; i < i_end; i++)
  {
    global_traj[i] = back_end_planner_->traj_.global_traj.traj.getPos(i * step_size_t);
  }
  
  // Publishes to "global_list"
  visualization_->displayGlobalPathList(global_traj, 0.1, 0);

  /*2:  Plan global trajectory */

  bool flag_polyInit = true; 
  bool flag_randomPolyTraj = true;
  bool touch_goal = false;

  Eigen::Vector3d local_target_pos;
  Eigen::Vector3d local_target_vel;

  // If this is the first time planning has been called, then initialize a random polynomial
  for (int i = 0; i < num_retries; i++)
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
      logInfo("Back-end planning successful!");
      break;
    }
  }

  if (!plan_success)
  {
    logError("back_end_planner_->reboundReplan not successful");
    return false;
  }

  // Get data from local trajectory and store in PolyTraj and MINCOTraj 
  traj_utils::PolyTraj poly_msg; 
  traj_utils::MINCOTraj MINCO_msg; 
  ego_planner::LocalTrajData* local_traj_data = &back_end_planner_->traj_.local_traj;

  polyTraj2ROSMsg(local_traj_data, poly_msg, MINCO_msg);

  // poly_traj_pub_.publish(poly_msg); // (In drone origin frame) Publish to corresponding drone for execution
  // broadcast_ploytraj_pub_.publish(MINCO_msg); // (In world frame) Broadcast to all other drones for replanning to optimize in avoiding swarm collision
  
  return true;
}

void BackEndPlanner::polyTraj2ROSMsg(ego_planner::LocalTrajData *data, traj_utils::PolyTraj &poly_msg, traj_utils::MINCOTraj &MINCO_msg)
{
  // struct LocalTrajData
  // {
  //   poly_traj::Trajectory traj;
  //   PtsChk_t pts_chk;
  //   int drone_id; // A negative value indicates no received trajectories.
  //   int traj_id;
  //   double duration;
  //   double start_time; // world time
  //   double end_time;   // world time
  //   Eigen::Vector3d start_pos;
  // };

  Eigen::VectorXd durs = data->traj.getDurations();
  int piece_num = data->traj.getPieceSize();
  poly_msg.drone_id = back_end_planner_->pp_.drone_id;
  poly_msg.traj_id = data->traj_id;
  poly_msg.start_time = ros::Time(data->start_time);
  poly_msg.order = 5; // todo, only support order = 5 now.
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
    poly_traj::CoefficientMat cMat = data->traj.getPiece(i).getCoeffMat();
    int i6 = i * 6;
    for (int j = 0; j < 6; j++)
    {
      poly_msg.coef_x[i6 + j] = cMat(0, j);
      poly_msg.coef_y[i6 + j] = cMat(1, j);
      poly_msg.coef_z[i6 + j] = cMat(2, j);
    }
  }

  MINCO_msg.drone_id = back_end_planner_->pp_.drone_id;
  MINCO_msg.traj_id = data->traj_id;
  MINCO_msg.start_time = ros::Time(data->start_time);
  MINCO_msg.order = 5; // todo, only support order = 5 now.
  MINCO_msg.duration.resize(piece_num);

  Eigen::Vector3d vec; // Vector representing x,y,z values or their derivatives
  // Start Position
  vec = data->traj.getPos(0);
  MINCO_msg.start_p[0] = vec(0), MINCO_msg.start_p[1] = vec(1), MINCO_msg.start_p[2] = vec(2);
  // Start Velocity
  vec = data->traj.getVel(0);
  MINCO_msg.start_v[0] = vec(0), MINCO_msg.start_v[1] = vec(1), MINCO_msg.start_v[2] = vec(2);
  // Start Acceleration
  vec = data->traj.getAcc(0);
  MINCO_msg.start_a[0] = vec(0), MINCO_msg.start_a[1] = vec(1), MINCO_msg.start_a[2] = vec(2);
  // End position
  vec = data->traj.getPos(data->duration);
  MINCO_msg.end_p[0] = vec(0), MINCO_msg.end_p[1] = vec(1), MINCO_msg.end_p[2] = vec(2);
  // End velocity
  vec = data->traj.getVel(data->duration);
  MINCO_msg.end_v[0] = vec(0), MINCO_msg.end_v[1] = vec(1), MINCO_msg.end_v[2] = vec(2);
  // End Acceleration
  vec = data->traj.getAcc(data->duration);
  MINCO_msg.end_a[0] = vec(0), MINCO_msg.end_a[1] = vec(1), MINCO_msg.end_a[2] = vec(2);

  // Assign inner points
  MINCO_msg.inner_x.resize(piece_num - 1);
  MINCO_msg.inner_y.resize(piece_num - 1);
  MINCO_msg.inner_z.resize(piece_num - 1);
  Eigen::MatrixXd pos = data->traj.getPositions();
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

/* Checking methods */

bool BackEndPlanner::isPlanFeasible(const Eigen::Vector3d& waypoints){
  // Check occupancy of every waypoint
  return true;
}