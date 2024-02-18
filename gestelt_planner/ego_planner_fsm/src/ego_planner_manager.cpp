#include <ego_planner_fsm/ego_planner_manager.h>

namespace ego_planner
{
  // SECTION interfaces for setup and query

  EGOPlannerManager::EGOPlannerManager() {}

  EGOPlannerManager::~EGOPlannerManager() { 
    std::cout << "destroyed EGOPlannerManager" << std::endl; 
  }

  void EGOPlannerManager::initPlanModules(ros::NodeHandle &nh, ros::NodeHandle &pnh, PlanningVisualization::Ptr vis)
  {
    pnh.param("drone_id", pp_.drone_id, -1);

    /* read algorithm parameters */
    pnh.param("manager/max_vel", pp_.max_vel_, -1.0);
    pnh.param("manager/max_acc", pp_.max_acc_, -1.0);
    pnh.param("manager/feasibility_tolerance", pp_.feasibility_tolerance_, 0.0);
    pnh.param("manager/polyTraj_piece_length", pp_.polyTraj_piece_length, -1.0);
    pnh.param("manager/planning_horizon", pp_.planning_horizon_, 5.0);
    pnh.param("manager/use_distinctive_trajs", pp_.use_distinctive_trajs, false);

    grid_map_.reset(new GridMap);
    grid_map_->initMapROS(nh, pnh);

    ploy_traj_opt_.reset(new PolyTrajOptimizer());
    ploy_traj_opt_->setParam(pnh);
    ploy_traj_opt_->setEnvironment(grid_map_);

    visualization_ = vis;

    ploy_traj_opt_->setVisualizer(visualization_);

    ploy_traj_opt_->setDroneId(pp_.drone_id);
  }

  bool EGOPlannerManager::generateMinJerkTraj(
      const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
      const std::vector<Eigen::Vector3d>& inner_wps,
      const Eigen::Vector3d &goal_pos, const Eigen::Vector3d &goal_vel,
      const Eigen::VectorXd &segs_t_dur,
      poly_traj::MinJerkOpt &mj_opt)
  {
    int num_segs = segs_t_dur.size();// Number of path segments

    Eigen::Matrix3d startPVA; // Boundary start condition: Matrix consisting of 3d (position, velocity acceleration) 
    Eigen::Matrix3d endPVA;   // Boundary end condition: Matrix consisting of 3d (position, velocity acceleration) 
    startPVA << start_pos, start_vel, start_acc;
    endPVA << goal_pos, goal_vel, Eigen::Vector3d::Zero();

    Eigen::MatrixXd inner_ctrl_pts(3, inner_wps.size()); // matrix of inner waypoints

    for (size_t i = 0; i < inner_wps.size(); i++){
      inner_ctrl_pts.col(i) = inner_wps[i];
    }

    mj_opt.reset(startPVA, endPVA, num_segs);
    mj_opt.generate(inner_ctrl_pts, segs_t_dur);

    return true;
  }

  bool EGOPlannerManager::optimizeMJOTraj(
    const poly_traj::MinJerkOpt &initial_mjo, poly_traj::MinJerkOpt& optimized_mjo,
    const std::vector<double>& spheres_radius, const std::vector<Eigen::Vector3d>& spheres_center)
  {
    bool plan_success = false;
    poly_traj::Trajectory initial_traj = initial_mjo.getTraj();

    int num_segs = initial_traj.getPieceSize();
    // Eigen::MatrixXd all_pos = initial_traj.getPositions();
    // Get inner_ctrl_pts, a block of size (3, num_segs-1) from column 1 onwards. This excludes the boundary points (start and goal).
    Eigen::MatrixXd inner_ctrl_pts = initial_traj.getPositions().block(0, 1, 3, num_segs - 1);
    Eigen::Matrix<double, 3, 3> headState, tailState;
    headState << initial_traj.getJuncPos(0),        initial_traj.getJuncVel(0),        initial_traj.getJuncAcc(0);
    tailState << initial_traj.getJuncPos(num_segs), initial_traj.getJuncVel(num_segs), initial_traj.getJuncAcc(num_segs);
    
    double final_cost; // Not used for now

    plan_success = ploy_traj_opt_->optimizeTrajectorySFC( 
      headState, tailState,
      inner_ctrl_pts, initial_traj.getDurations(), 
      spheres_radius, spheres_center,
      final_cost);

    // Optimized minimum jerk trajectory
    optimized_mjo = ploy_traj_opt_->getMinJerkOpt();

    return plan_success;
  }

  bool EGOPlannerManager::computeInitState(
      const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
      const Eigen::Vector3d &local_target_pos, const Eigen::Vector3d &local_target_vel,
      const bool flag_polyInit, const bool flag_randomPolyTraj, const double &t_seg_dur,
      poly_traj::MinJerkOpt &initMJO)
  {

    static bool flag_first_call = true;

    /*** case 1: polynomial initialization ***/
    if (flag_first_call || flag_polyInit) 
    {
      flag_first_call = false;

      /* basic params */
      Eigen::Matrix3d headState, tailState; //headstate and tailstate contains the start and end Position, Velocity and Acceleration
      headState << start_pos, start_vel, start_acc;
      tailState << local_target_pos, local_target_vel, Eigen::Vector3d::Zero(); // Assume local target has zero acceleration
      Eigen::MatrixXd innerPs;              // Inner points
      Eigen::VectorXd piece_dur_vec;        // Vector of piece/segment time durations
      int piece_nums;                       // Number of pieces/segments
      constexpr double init_of_init_totaldur = 2.0; // WHY is this 2.0?

      // IF: Non-random inner point 
      if (!flag_randomPolyTraj)
      {
        if (innerPs.cols() != 0)
        {
          ROS_ERROR("innerPs.cols() != 0");
        }

        piece_nums = 1;
        piece_dur_vec.resize(1);
        piece_dur_vec(0) = init_of_init_totaldur;
      }
      // ELSE: Random inner point
      else
      {
        Eigen::Vector3d lc_tgt_dir = start_pos - local_target_pos; // Direction vector from local target to start position
        // Get horizontal and vertical direction
        Eigen::Vector3d horizon_dir = (lc_tgt_dir.cross(Eigen::Vector3d(0, 0, 1))).normalized();
        Eigen::Vector3d vertical_dir = (lc_tgt_dir.cross(horizon_dir)).normalized();
        innerPs.resize(3, 1);
        // InnerP = midway position between start and target 
        //          + random dist along horizontal
        //          + random dist along vertical
        innerPs = (start_pos + local_target_pos) / 2 
                  + (((double)rand()) / RAND_MAX - 0.5) *  lc_tgt_dir.norm() *  horizon_dir * 0.8 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989) 
                  + (((double)rand()) / RAND_MAX - 0.5) *  lc_tgt_dir.norm() *  vertical_dir * 0.4 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989);

        piece_nums = 2;
        piece_dur_vec.resize(2);
        piece_dur_vec = Eigen::Vector2d(init_of_init_totaldur / 2, init_of_init_totaldur / 2);
      }

      /* generate the preliminary initial minimum jerk trajectory */
      // Why do we need this preliminary trajectory?
      initMJO.reset(headState, tailState, piece_nums);
      initMJO.generate(innerPs, piece_dur_vec);
      poly_traj::Trajectory initTraj = initMJO.getTraj();

      /* generate the actual init trajectory */
      // Get actual number of pieces/segments
      piece_nums = round((headState.col(0) - tailState.col(0)).norm() / pp_.polyTraj_piece_length); // num pieces/segments = (dist btw. start and goal) / (length of each piece/segment)
      piece_nums = piece_nums < 2 ? 2: piece_nums; // ensure piece_nums is at least 2

      // Segment duration assumed to be equal
      double piece_dur = init_of_init_totaldur / (double)piece_nums;
      piece_dur_vec.resize(piece_nums);
      piece_dur_vec = Eigen::VectorXd::Constant(piece_nums, t_seg_dur); 

      innerPs.resize(3, piece_nums - 1); // Resize inner waypoints 
      int idx = 0; 
      double t_s = piece_dur; // Starting time
      double t_e = init_of_init_totaldur - piece_dur / 2; // Ending time

      for (double t = t_s; t < t_e; t += piece_dur)
      {
        // set inner points to be that position from the initial trajectory
        innerPs.col(idx++) = initTraj.getPos(t);
      }

      if (idx != piece_nums - 1)
      {
        ROS_ERROR("Drone %d: [EGOPlannerManager::computeInitState] (idx != piece_nums - 1). Unexpected error", pp_.drone_id);
        return false;
      }
      
      initMJO.reset(headState, tailState, piece_nums);
      initMJO.generate(innerPs, piece_dur_vec);
    }
    /*** case 2: initialize from previous optimal trajectory ***/
    else 
    {
      // TODO: Better checking mechanism for whether global trajectory exists
      // last_glb_t_of_lc_tgt: The corresponding global trajectory time of the last local target
      if (traj_.global_traj.last_glb_t_of_lc_tgt < 0.0)
      {
        ROS_ERROR("Drone %d: [EGOPlannerManager::computeInitState] You are initializing a \\
                  trajectory from a previous optimal trajectory, but no \\
                  previous trajectories up to now.", pp_.drone_id);
        return false;
      }

      /* the trajectory time system is a little bit complicated... */

      // Time elapsed since local trajectory jas started
      double t_elapsed_lc_traj = ros::Time::now().toSec() - traj_.local_traj.start_time;
      // t_to_local_traj_end: Time left to complete trajectory
      double t_to_local_traj_end = traj_.local_traj.duration - t_elapsed_lc_traj;
      if ( t_to_local_traj_end < 0 )
      { 
        ROS_INFO("Drone %d: [EGOPlannerManager::computeInitState] Time left to complete local trajectory \\
          is negative, exit and wait for another call.", pp_.drone_id);
        return false;
      }
      // t_to_local_tgt: Time to reach local target
      double t_to_local_tgt = t_to_local_traj_end +
                           (traj_.global_traj.glb_t_of_lc_tgt - traj_.global_traj.last_glb_t_of_lc_tgt);

      // Number of pieces = remaining distance / piece length 
      int piece_nums = ceil((start_pos - local_target_pos).norm() / pp_.polyTraj_piece_length);
      piece_nums = piece_nums < 2 ? 2: piece_nums; // ensure piece_nums is always minimally 2

      Eigen::Matrix3d headState;
      Eigen::Matrix3d tailState;
      headState << start_pos, start_vel, start_acc;
      tailState << local_target_pos, local_target_vel, Eigen::Vector3d::Zero();
      Eigen::MatrixXd innerPs(3, piece_nums - 1);
      // piece_dur_vec: Vector of time duration of each piece. 
      // Each duration assumed to be equal, value = time to local target / piece_num 
      Eigen::VectorXd piece_dur_vec = Eigen::VectorXd::Constant(piece_nums, t_to_local_tgt / piece_nums);

      /* Generate inner waypoints based on time per piece */
      double t = piece_dur_vec(0); // Set start time t to first piece  
      for (int i = 0; i < piece_nums - 1; ++i) // For each piece
      {
        // t ----> t_to_local_traj_end ----> t_to_local_tgt

        // if not yet end of local trajectory
        if (t < t_to_local_traj_end) 
        {
          innerPs.col(i) = traj_.local_traj.traj.getPos(t + t_elapsed_lc_traj);
        }
        // if not yet reached local target but exceed local trajectory
        // we need to use global trajectory 
        else if (t <= t_to_local_tgt) 
        {
          // glb_t: t - time to local trajectory end + time last local target was set - start time of global trajectory
          double glb_t = t - t_to_local_traj_end + traj_.global_traj.last_glb_t_of_lc_tgt - traj_.global_traj.global_start_time;
          innerPs.col(i) = traj_.global_traj.traj.getPos(glb_t);
        }
        else
        {
          // time t should not exceed t_to_local_Tgt
          ROS_ERROR("Drone %d: [EGOPlannerManager::computeInitState] t=%.2f, t_to_local_traj_end=%.2f, t_to_local_tgt=%.2f", pp_.drone_id, t, t_to_local_traj_end, t_to_local_tgt);
        }

        t += piece_dur_vec(i + 1);
      }

      initMJO.reset(headState, tailState, piece_nums);
      initMJO.generate(innerPs, piece_dur_vec);
    }

    return true;
  }

  void EGOPlannerManager::getLocalTarget(
      const double planning_horizon, const Eigen::Vector3d &start_pos,
      const Eigen::Vector3d &global_end_pt, Eigen::Vector3d &local_target_pos,
      Eigen::Vector3d &local_target_vel, bool &touch_goal)
  {
    double t;
    touch_goal = false;

    // Set the last global traj local target timestamp to be that of the current global plan
    traj_.global_traj.last_glb_t_of_lc_tgt = traj_.global_traj.glb_t_of_lc_tgt;

    double t_step = planning_horizon / 20 / pp_.max_vel_;
    // double dist_min = 9999, dist_min_t = 0.0;

    // Iterate through the start of global plan until it
    // reaches the end of the plan or it has exceeded the planning horizon
    for (t = traj_.global_traj.glb_t_of_lc_tgt;
         t < (traj_.global_traj.global_start_time + traj_.global_traj.duration);
         t += t_step)
    {
      // Get pos(t), the position at each time step
      Eigen::Vector3d pos_t = traj_.global_traj.traj.getPos(t - traj_.global_traj.global_start_time);

      // If norm from start to the pos(t) exceeds planning horizon
      if ((pos_t - start_pos).norm() >= planning_horizon)
      {
        // set local target as pos(t) 
        // and timestamp of global trajectory local target as t 
        local_target_pos = pos_t;
        traj_.global_traj.glb_t_of_lc_tgt = t;
        break;
      }
    }

    // If time t exceeds duration of trajectory
    // Set local target to be the global target
    if ((t - traj_.global_traj.global_start_time) >= traj_.global_traj.duration - 1e-5) // Last global point
    {
      local_target_pos = global_end_pt;
      traj_.global_traj.glb_t_of_lc_tgt = traj_.global_traj.global_start_time + traj_.global_traj.duration;
      touch_goal = true;
    }

    // Equation: v^2 = u^2 + 2as
    //           s = (v^2 - u^2)/(2a)  // max_d Distance covered at maximum velocity and maximum acceleration from rest
    // IF distance from local target to global target < max_d
    //    Then it is possible to cover the remaining distance with zero velocity at local target
    // ELSE
    //    We require a non-zero velocity at the local target
    if ((global_end_pt - local_target_pos).norm() < (pp_.max_vel_ * pp_.max_vel_) / (2 * pp_.max_acc_))
    {
      local_target_vel = Eigen::Vector3d::Zero();
    }
    else
    {
      local_target_vel = traj_.global_traj.traj.getVel(t - traj_.global_traj.global_start_time);
    }
  }

  bool EGOPlannerManager::setLocalTrajFromOpt(const poly_traj::MinJerkOpt &opt, const bool touch_goal)
  {
    poly_traj::Trajectory traj = opt.getTraj();
    Eigen::MatrixXd cps = opt.getInitConstraintPoints(getCpsNumPrePiece());
    PtsChk_t pts_to_check;
    bool ret = ploy_traj_opt_->computePointsToCheck(traj, ConstraintPoints::two_thirds_id(cps, touch_goal), pts_to_check);
    if (ret){
      traj_.setLocalTraj(traj, pts_to_check, ros::Time::now().toSec());
    }

    return ret;
  }

  bool EGOPlannerManager::reboundReplan(
      const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel,
      const Eigen::Vector3d &start_acc, const Eigen::Vector3d &local_target_pos,
      const Eigen::Vector3d &local_target_vel, const bool flag_polyInit,
      const bool flag_randomPolyTraj, const bool touch_goal)
  {
    static int count = 0;
    printf("\033[47;30m\n[drone %d replan %d]==============================================\033[0m\n",
           pp_.drone_id, count++);
    std::cout.precision(3);
    std::cout << "start: " << start_pos.transpose() << ", " << start_vel.transpose() << "\ngoal:" << local_target_pos.transpose() << ", " << local_target_vel.transpose()
         <<std::endl;

    ploy_traj_opt_->setIfTouchGoal(touch_goal);

    ros::Time t_start = ros::Time::now();
    ros::Duration t_init, t_opt;

    /*** STEP 1: INIT. Get initial trajectory and {p,v} pairs ***/

    // t_seg_dur: duration of segment = distance between control points / maximum velocity
    double t_seg_dur = pp_.polyTraj_piece_length / pp_.max_vel_;

    poly_traj::MinJerkOpt initMJO; // Initial minimum jerk trajectory optimizer

    /*** STEP 1a: Get initial minimum jerk trajectory ***/
    // This is done using the local trajectory and the existing global trajectory (from start to actual goal)
    if (!computeInitState(start_pos, start_vel, start_acc, 
                          local_target_pos, local_target_vel,
                          flag_polyInit, flag_randomPolyTraj, t_seg_dur, 
                          initMJO))
    {
      return false;
    }

    /*** STEP 1b: Get initail constraint points ***/

    Eigen::MatrixXd initial_cstr_pts = initMJO.getInitConstraintPoints(ploy_traj_opt_->get_cps_num_perPiece_());
    vector<std::pair<int, int>> segments; // segments are only needed for ESDF Local planner and distinctive trajectories

    // Check for collision along path and set {p,v} pairs to constraint points.
    // The collision free path for the segments in collision is determined using AStar search
    // Returns a vector of pairs (seg_start_idx, seg_end_idx)
    if (ploy_traj_opt_->finelyCheckAndSetConstraintPoints(segments, initMJO, true) == PolyTrajOptimizer::CHK_RET::ERR)
    {
      return false;
    }

    t_init = ros::Time::now() - t_start;

    std::vector<Eigen::Vector3d> point_set; //Used for visualization: set of constraint points
    for (int i = 0; i < initial_cstr_pts.cols(); ++i){
      point_set.push_back(initial_cstr_pts.col(i));
    }
    visualization_->displayInitialMinJerkTraj(point_set, 0.2, 0);

    t_start = ros::Time::now();

    /*** STEP 2: OPTIMIZE ***/
    bool flag_success = false;
    poly_traj::MinJerkOpt best_MJO; // Best minimum jerk trajectory

    poly_traj::Trajectory initTraj = initMJO.getTraj();
    int P_sz = initTraj.getPieceSize();
    Eigen::MatrixXd all_pos = initTraj.getPositions();
    // Get inner_ctrl_pts, a block of size (3, P_sz-1) from column 1 onwards. This excludes the first point.
    Eigen::MatrixXd inner_ctrl_pts = all_pos.block(0, 1, 3, P_sz - 1);
    Eigen::Matrix<double, 3, 3> headState, tailState;
    headState << initTraj.getJuncPos(0), initTraj.getJuncVel(0), initTraj.getJuncAcc(0);
    tailState << initTraj.getJuncPos(P_sz), initTraj.getJuncVel(P_sz), initTraj.getJuncAcc(P_sz);
    double final_cost; // Not used for now
    flag_success = ploy_traj_opt_->optimizeTrajectory(headState, tailState,
                                                      inner_ctrl_pts, initTraj.getDurations(),
                                                      initial_cstr_pts, final_cost);

    best_MJO = ploy_traj_opt_->getMinJerkOpt();

    t_opt = ros::Time::now() - t_start;

    // // save and display planned results
    if (!flag_success)
    {
      ROS_ERROR("[EGOPlannerManager::reboundReplan] Planning unsuccessful from ploy_traj_opt_->optimizeTrajectory");
      visualization_->displayFailedList(initial_cstr_pts, 0);
      continous_failures_count_++;
      return false;
    }

    static double sum_time = 0;
    static int count_success = 0;
    sum_time += (t_init + t_opt).toSec();
    count_success++;

    std::cout << "total time (ms):\033[42m" << (t_init + t_opt).toSec()*1000
         << "\033[0m, init(ms):" << t_init.toSec()*1000
         << ",optimize(ms):" << t_opt.toSec()*1000
         << ",avg_time(ms)=" << sum_time / count_success << std::endl;
    
    setLocalTrajFromOpt(best_MJO, touch_goal);
    visualization_->displayOptimalList(initial_cstr_pts, 0);

    // success. YoY
    continous_failures_count_ = 0;
    return true;
  }

  bool EGOPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos)
  {
    auto ZERO = Eigen::Vector3d::Zero();
    Eigen::Matrix<double, 3, 3> headState, tailState;
    headState << stop_pos, ZERO, ZERO;
    tailState = headState;
    poly_traj::MinJerkOpt stopMJO;
    stopMJO.reset(headState, tailState, 2);
    stopMJO.generate(stop_pos, Eigen::Vector2d(1.0, 1.0));

    setLocalTrajFromOpt(stopMJO, false);

    return true;
  }

  bool EGOPlannerManager::checkCollision(int drone_id)
  {
    if (traj_.local_traj.start_time < 1e9) // It means my first planning has not started
      return false;

    double my_traj_start_time = traj_.local_traj.start_time;
    double other_traj_start_time = traj_.swarm_traj[drone_id].start_time;

    double t_start = std::max(my_traj_start_time, other_traj_start_time);
    double t_end = std::min(my_traj_start_time + traj_.local_traj.duration * 2 / 3,
                       other_traj_start_time + traj_.swarm_traj[drone_id].duration);

    for (double t = t_start; t < t_end; t += 0.03)
    {
      if ((traj_.local_traj.traj.getPos(t - my_traj_start_time) -
           traj_.swarm_traj[drone_id].traj.getPos(t - other_traj_start_time))
              .norm() < getSwarmClearance())
      {
        return true;
      }
    }

    return false;
  }

  bool EGOPlannerManager::planGlobalTrajWaypoints(
      poly_traj::MinJerkOpt& globalMJO,
      const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc, 
      const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {
    Eigen::Matrix<double, 3, 3> headState, tailState;
    headState << start_pos, start_vel, start_acc;
    tailState << waypoints.back(), end_vel, end_acc;
    Eigen::MatrixXd inner_ctrl_pts; // vector of 3d positions 

    if (waypoints.size() > 1)
    {
      inner_ctrl_pts.resize(3, waypoints.size() - 1);
      for (int i = 0; i < (int)waypoints.size() - 1; ++i)
      {
        inner_ctrl_pts.col(i) = waypoints[i];
      }
    }

    globalMJO.reset(headState, tailState, waypoints.size());

    // TODO Why is max_vel divided by 1.5?
    double des_vel = pp_.max_vel_ / 1.5;
    Eigen::VectorXd time_vec(waypoints.size());

    // Try replanning up to 2 times if the velocity constraints are not fulfilled
    int num_retries = 2;
    for (int j = 0; j < num_retries; ++j)
    {
      // TODO: Refactor
      // for each plan segment, calculate time using desired velocity
      for (size_t i = 0; i < waypoints.size(); ++i)
      {
        time_vec(i) = (i == 0) ? (waypoints[0] - start_pos).norm() / des_vel
                               : (waypoints[i] - waypoints[i - 1]).norm() / des_vel;
      }

      // Generate a minimum snap trajectory
      globalMJO.generate(inner_ctrl_pts, time_vec);

      // check if any point in trajectory exceeds maximum velocity
      if (globalMJO.getTraj().getMaxVelRate() < pp_.max_vel_ ||
          start_vel.norm() > pp_.max_vel_ ||
          end_vel.norm() > pp_.max_vel_)
      {
        break;
      }

      if (j == 2) // on final try
      {
        ROS_WARN("Global traj MaxVel = %f > set_max_vel", globalMJO.getTraj().getMaxVelRate());
        std::cout << "headState=" << std::endl
             << headState << std::endl;
        std::cout << "tailState=" << std::endl
             << tailState <<std::endl;
      }

      // Reduce desired velocity by a factor of 1.5 and try again
      des_vel /= 1.5;
    }



    return true;
  }

  /* Utility methods */

  void EGOPlannerManager::setSwarmTrajectories(std::shared_ptr<std::unordered_map<int, ego_planner::LocalTrajData>>& swarm_minco_trajs)
  {
    // ploy_traj_opt_->setSwarmTrajs(&traj_.swarm_traj);
    ploy_traj_opt_->assignSwarmTrajs(swarm_minco_trajs);
  }

  double EGOPlannerManager::getTrajectoryLength(poly_traj::MinJerkOpt& mjo, const double& dt)
  {
    poly_traj::Trajectory traj = mjo.getTraj();

    double total_duration = traj.getDurations().sum();
    double total_traj_length = 0.0;

    for (double t = 0; t < total_duration-dt; t += dt)
    {
      total_traj_length += (traj.getPos(t+dt) - traj.getPos(t)).norm();
    }
    
    return total_traj_length;
  }

  double EGOPlannerManager::getTrajectoryDuration(poly_traj::MinJerkOpt& mjo)
  {
    return mjo.getTraj().getDurations().sum();
  }

} // namespace ego_planner
