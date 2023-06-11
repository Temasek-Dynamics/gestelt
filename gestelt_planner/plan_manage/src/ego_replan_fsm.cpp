
#include <plan_manage/ego_replan_fsm.h>

namespace ego_planner
{
  void EGOReplanFSM::init(ros::NodeHandle &nh)
  {
    have_target_ = false;
    have_odom_ = false;
    have_recv_pre_agent_ = false;
    flag_escape_emergency_ = true;

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(nh));
    planner_manager_.reset(new EGOPlannerManager);
    planner_manager_->initPlanModules(nh, visualization_);

    /* ROS Params*/

    // This applies an offset to all trajectories received and sent so that they are relative to the world frame
    nh.param("fsm/frame_offset_x", uav_origin_to_world_tf_.position.x, 0.0);
    nh.param("fsm/frame_offset_y", uav_origin_to_world_tf_.position.y, 0.0);
    nh.param("fsm/frame_offset_z", uav_origin_to_world_tf_.position.z, 0.0);

    // Reverse signs so that the transformation is from UAV origin frame to world frame
    world_to_uav_origin_tf_.position.x = -uav_origin_to_world_tf_.position.x;
    world_to_uav_origin_tf_.position.y = -uav_origin_to_world_tf_.position.y;
    world_to_uav_origin_tf_.position.z = -uav_origin_to_world_tf_.position.z;

    /*  fsm param  */
    nh.param("fsm/flight_type", target_type_, -1);
    nh.param("fsm/thresh_replan_time", replan_time_thresh_, -1.0);
    nh.param("fsm/thresh_no_replan_meter", min_replan_dist_, -1.0);
    nh.param("fsm/planning_horizon", planning_horizen_, -1.0);
    nh.param("fsm/emergency_time", emergency_time_, 1.0);
    nh.param("fsm/fail_safe", enable_fail_safe_, true);

    int formation_num = -1;
    nh.param("formation/num", formation_num, -1);
    if (formation_num < planner_manager_->pp_.drone_id + 1)
    {
      logError("formation_num is smaller than the drone number, illegal!");
      return;
    }
    std::vector<double> pos;
    nh.getParam("formation/drone" + to_string(planner_manager_->pp_.drone_id), pos);
    formation_pos_ << pos[0], pos[1], pos[2];
    nh.getParam("formation/start", pos);

    Eigen::Vector3d formation_start;
    formation_start << pos[0], pos[1], pos[2];
    waypoints_.setStartWP(formation_start);

    /* Timer callbacks */
    tick_state_timer_ = nh.createTimer(ros::Duration(0.01), &EGOReplanFSM::tickStateTimerCB, this);
    exec_state_timer_ = nh.createTimer(ros::Duration(0.05), &EGOReplanFSM::execStateTimerCB, this);

    /* Subscribers */
    odom_sub_ = nh.subscribe("odom_world", 1, &EGOReplanFSM::odometryCallback, this);
    mandatory_stop_sub_ = nh.subscribe("mandatory_stop", 1, &EGOReplanFSM::mandatoryStopCallback, this);

    // Use MINCO trajectory to minimize the message size in wireless communication
    broadcast_ploytraj_sub_ = nh.subscribe<traj_utils::MINCOTraj>("planning/broadcast_traj_recv", 100,
                                                                  &EGOReplanFSM::RecvBroadcastMINCOTrajCallback,
                                                                  this,
                                                                  ros::TransportHints().tcpNoDelay());

    trigger_sub_ = nh.subscribe("/traj_start_trigger", 1, &EGOReplanFSM::triggerCallback, this);

    /* Publishers */
    broadcast_ploytraj_pub_ = nh.advertise<traj_utils::MINCOTraj>("planning/broadcast_traj_send", 10);
    poly_traj_pub_ = nh.advertise<traj_utils::PolyTraj>("planning/trajectory", 10);
    heartbeat_pub_ = nh.advertise<std_msgs::Empty>("planning/heartbeat", 10);
    ground_height_pub_ = nh.advertise<std_msgs::Float64>("/ground_height_measurement", 10);

    if (target_type_ == TARGET_TYPE::MANUAL_TARGET)
    {
      waypoint_sub_ = nh.subscribe("/goal", 1, &EGOReplanFSM::waypointCallback, this);
    }
    else if (target_type_ == TARGET_TYPE::PRESET_TARGET)
    {
      // Subscribe to waypoints 
      waypoints_sub_ = nh.subscribe("/waypoints", 1, &EGOReplanFSM::waypointsCB, this);
    }
    else{
      logError(string_format("Wrong target_type_ value! target_type_=%i", target_type_));
    }
  }

  /**
   * Timer Callbacks
  */

  void EGOReplanFSM::tickStateTimerCB(const ros::TimerEvent &e)
  {
    // logInfoThrottled(string_format("Current State: [%s]", StateToString(getServerState()).c_str()), 1.0);
    std_msgs::Empty heartbeat_msg;
    heartbeat_pub_.publish(heartbeat_msg);

    static int fsm_num = 0;
    if (fsm_num++ == 500)
    {
      fsm_num = 0;
      printFSMExecState();
    }

    switch (getServerState())
    {
      case INIT:
      {
        switch (getServerEvent())
        {
          case READY_E:
            setServerState(ServerState::READY);
            break;
          case EMERGENCY_STOP_E:
            logFatal("EMERGENCY STOP ACTIVATED!");
            setServerState(ServerState::EMERGENCY_STOP);
            break;
          default:
            break;
        }
        break;
      }

      case READY: 
      {
        switch (getServerEvent())
        {
          case PLAN_GLOBAL_TRAJ_E:
            setServerState(ServerState::PLAN_GLOBAL_TRAJ);
            break;
          case EMERGENCY_STOP_E:
            logFatal("EMERGENCY STOP ACTIVATED!");
            setServerState(ServerState::EMERGENCY_STOP);
            break;
          default:
            break;
        }
        break;
      }

      case PLAN_GLOBAL_TRAJ: 
      {
        switch (getServerEvent())
        {
          case EXEC_TRAJ_E:
            setServerState(ServerState::EXEC_TRAJ);
            break;
          case EMERGENCY_STOP_E:
            logFatal("EMERGENCY STOP ACTIVATED!");
            setServerState(ServerState::EMERGENCY_STOP);
            break;
          default:
            break;
        }

        break;
      }

      case PLAN_LOCAL_TRAJ:
      {

        switch (getServerEvent())
        {
          case EXEC_TRAJ_E:
            setServerState(ServerState::EXEC_TRAJ);
            break;
          case EMERGENCY_STOP_E:
            logFatal("EMERGENCY STOP ACTIVATED!");
            setServerState(ServerState::EMERGENCY_STOP);
            break;
          default:
            break;
        }

        break;
      }

      case EXEC_TRAJ:
      {
        switch (getServerEvent())
        {
          case READY_E:
            setServerState(ServerState::READY);
            break;
          case PLAN_LOCAL_TRAJ_E:
            setServerState(ServerState::PLAN_LOCAL_TRAJ);
            break;
          case EMERGENCY_STOP_E:
            logFatal("EMERGENCY STOP ACTIVATED!");
            setServerState(ServerState::EMERGENCY_STOP);
            break;
          default:
            break;
        }

        break;
      }


      case PLAN_NEW_GLOBAL_TRAJ:
      {
        switch (getServerEvent())
        {
          case EXEC_TRAJ_E:
            setServerState(ServerState::EXEC_TRAJ);
            break;
          case EMERGENCY_STOP_E:
            logFatal("EMERGENCY STOP ACTIVATED!");
            setServerState(ServerState::EMERGENCY_STOP);
            break;
          default:
            break;
        }

        break;
      }

      case EMERGENCY_STOP:
      {
        // Do nothing, vehicle requires restarting
        break;
      }
    }
  }

  void EGOReplanFSM::execStateTimerCB(const ros::TimerEvent &e){
    switch (getServerState())
    {
      case INIT:
      {
        if (have_odom_){
          setServerEvent(READY_E);
        }
        break;
      }

      case READY: 
      {
        if (have_target_) {
          setServerEvent(PLAN_GLOBAL_TRAJ_E);
        }
        break;
      }

      case PLAN_GLOBAL_TRAJ: 
      {
        // If first drone or it has received the trajectory of the previous agent.
        if (planner_manager_->pp_.drone_id <= 0 || (planner_manager_->pp_.drone_id >= 1 && have_recv_pre_agent_))
        {
          if (planFromGlobalTraj(10))
          {
            setServerEvent(EXEC_TRAJ_E);
          }
          else
          {
            logError("Failed to generate the first global trajectory! Retrying.");
          }
        }

        break;
      }

      case PLAN_LOCAL_TRAJ:
      {
        if (planFromLocalTraj(1))
        {
          setServerEvent(EXEC_TRAJ_E);
        }
        else 
        {
          logError(string_format("Replan failed upon detecting potential collision"));
          if (potential_agent_collision_)
          {
            logError(string_format("Potential agent collision detected, activating ESTOP"));
            setServerEvent(EMERGENCY_STOP_E);
          }
        }

        break;
      }

      case EXEC_TRAJ:
      {
        if (checkSensorTimeout())
        {
          setServerEvent(EMERGENCY_STOP_E);
          break;
        }

        if (checkTrajectoryClearance())
        {
          logInfo(string_format("Replanning to avoid collision."));
          setServerEvent(PLAN_LOCAL_TRAJ_E);
          break;
        }

        // if (checkGroundHeight){
        //   setServerEvent(PLAN_LOCAL_TRAJ_E);
        //   break;
        // }

        std::pair<bool,bool> GoalReachedAndReplanNeededCheck = isGoalReachedAndReplanNeeded();

        if (GoalReachedAndReplanNeededCheck.first) {
          logError("Goal reached!");
          // The navigation task completed 
          setServerEvent(READY_E);
          break;
        }
        else if (GoalReachedAndReplanNeededCheck.second) {
          // Replanning trajectory needed (Only checks for itself, not with other drones)
          setServerEvent(PLAN_LOCAL_TRAJ_E);
          break;
        }

        break;
      }

      case PLAN_NEW_GLOBAL_TRAJ:
      {
        if (planFromGlobalTraj(10)) 
        {
          setServerEvent(EXEC_TRAJ_E);
          flag_escape_emergency_ = true; // TODO Refactor
        }

        break;
      }

      case EMERGENCY_STOP:
      {
        if (flag_escape_emergency_) // Avoiding repeated calls to callEmergencyStop
        {
          callEmergencyStop(odom_pos_);
        }
        else
        {
          if (enable_fail_safe_ && odom_vel_.norm() < 0.1){
            setServerEvent(PLAN_NEW_GLOBAL_TRAJ_E);
          }
        }

        flag_escape_emergency_ = false;
        break;
      }
    }
  }

  /**
   * Subscriber Callbacks
  */

  void EGOReplanFSM::mandatoryStopCallback(const std_msgs::Empty &msg)
  {
    logError("Received a mandatory stop command!");
    setServerEvent(EMERGENCY_STOP_E);
    enable_fail_safe_ = false;
  }

  void EGOReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    odom_pos_(0) = msg->pose.pose.position.x ;
    odom_pos_(1) = msg->pose.pose.position.y ;
    odom_pos_(2) = msg->pose.pose.position.z ;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    have_odom_ = true;
  }

  void EGOReplanFSM::triggerCallback(const geometry_msgs::PoseStampedPtr &msg)
  {
    have_trigger_ = true;
    logInfo("Execution of goals triggered!");
  }

  void EGOReplanFSM::RecvBroadcastMINCOTrajCallback(const traj_utils::MINCOTrajConstPtr &msg)
  {
    traj_utils::MINCOTraj minco_traj = *msg;

    const size_t recv_id = (size_t)minco_traj.drone_id;
    if ((int)recv_id == planner_manager_->pp_.drone_id){ // Receiving the same plan produced by this very drone
      return;
    }

    if (minco_traj.drone_id < 0)
    {
      logError("drone_id < 0 is not allowed in a swarm system!");
      return;
    }
    if (minco_traj.order != 5)
    {
      logError("Only support trajectory order equals 5 now!");
      return;
    }
    if (minco_traj.duration.size() != (minco_traj.inner_x.size() + 1))
    {
      logError("Wrong trajectory parameters.");
      return;
    }
    if (planner_manager_->traj_.swarm_traj.size() > recv_id &&
        planner_manager_->traj_.swarm_traj[recv_id].drone_id == (int)recv_id &&
        minco_traj.start_time.toSec() - planner_manager_->traj_.swarm_traj[recv_id].start_time <= 0)
    {
      logWarn(string_format("Received drone %d's trajectory out of order or duplicated, abandon it.", (int)recv_id));
      return;
    }

    ros::Time t_now = ros::Time::now();
    if (abs((t_now - minco_traj.start_time).toSec()) > 0.25)
    {

      if (abs((t_now - minco_traj.start_time).toSec()) < 10.0) // 10 seconds offset, more likely to be caused by unsynced system time.
      {
        logWarn(string_format("Time stamp diff: Local - Remote Agent %d = %fs",
                 minco_traj.drone_id, (t_now - minco_traj.start_time).toSec()));
      }
      else
      {
        logError(string_format("Time stamp diff: Local - Remote Agent %d = %fs, swarm time seems not synchronized, abandon!",
                  minco_traj.drone_id, (t_now - minco_traj.start_time).toSec()));
        return;
      }
    }

    /* Fill up the buffer */
    if (planner_manager_->traj_.swarm_traj.size() <= recv_id)
    {
      for (size_t i = planner_manager_->traj_.swarm_traj.size(); i <= recv_id; i++)
      {
        LocalTrajData blank;
        blank.drone_id = -1;
        planner_manager_->traj_.swarm_traj.push_back(blank);
      }
    }

    transformMINCOTrajectoryToUAVOrigin(minco_traj);

    /* Store data */
    planner_manager_->traj_.swarm_traj[recv_id].drone_id = recv_id;
    planner_manager_->traj_.swarm_traj[recv_id].traj_id = minco_traj.traj_id;
    planner_manager_->traj_.swarm_traj[recv_id].start_time = minco_traj.start_time.toSec();

    int piece_nums = minco_traj.duration.size();
    Eigen::Matrix<double, 3, 3> headState, tailState;

    // Position here is transformed to UAV frame 
    headState <<  minco_traj.start_p[0], 
                    minco_traj.start_v[0], minco_traj.start_a[0],
                  minco_traj.start_p[1], 
                    minco_traj.start_v[1], minco_traj.start_a[1],
                  minco_traj.start_p[2], 
                    minco_traj.start_v[2], minco_traj.start_a[2];
    tailState <<  minco_traj.end_p[0], 
                    minco_traj.end_v[0], minco_traj.end_a[0],
                  minco_traj.end_p[1], 
                    minco_traj.end_v[1], minco_traj.end_a[1],
                  minco_traj.end_p[2], 
                    minco_traj.end_v[2], minco_traj.end_a[2];

    Eigen::MatrixXd innerPts(3, piece_nums - 1);
    Eigen::VectorXd durations(piece_nums);

    for (int i = 0; i < piece_nums - 1; i++){
      innerPts.col(i) <<  minco_traj.inner_x[i], 
                          minco_traj.inner_y[i], 
                          minco_traj.inner_z[i];
    }
    for (int i = 0; i < piece_nums; i++){
      durations(i) = minco_traj.duration[i];
    }

    // Optimize for minimum jerk
    poly_traj::MinJerkOpt MJO;
    MJO.reset(headState, tailState, piece_nums);
    MJO.generate(innerPts, durations);

    poly_traj::Trajectory trajectory = MJO.getTraj();
    planner_manager_->traj_.swarm_traj[recv_id].traj = trajectory;

    planner_manager_->traj_.swarm_traj[recv_id].duration = trajectory.getTotalDuration();
    planner_manager_->traj_.swarm_traj[recv_id].start_pos = trajectory.getPos(0.0);

    /* Check Collision */
    if (planner_manager_->checkCollision(recv_id))
    {
      logError(string_format("Imminent COLLISION between ownself and drone %d", recv_id));
      // TODO: What state is it expected to be in? EXEC_TRAJ?
      setServerState(PLAN_LOCAL_TRAJ);
    }

    /* Check if receive agents have lower drone id */
    if (!have_recv_pre_agent_)
    {
      // If the number of trajectories exceed or are same as current drone id
      if ((int)planner_manager_->traj_.swarm_traj.size() >= planner_manager_->pp_.drone_id)
      {
        // For each drone id
        for (int i = 0; i < planner_manager_->pp_.drone_id; ++i)
        {
          // If the drone_id of the i-th swarm trajectory is not the same as the i-th value, then break out
          if (planner_manager_->traj_.swarm_traj[i].drone_id != i)
          {
            break;
          }
          have_recv_pre_agent_ = true;
        }
      }
    }
  }

  void EGOReplanFSM::waypointCallback(const geometry_msgs::PoseStampedPtr &msg)
  {
    Eigen::Vector3d wp(
      msg->pose.position.x + world_to_uav_origin_tf_.position.x, 
      msg->pose.position.y + world_to_uav_origin_tf_.position.y, 
      1.0 + world_to_uav_origin_tf_.position.z);
    
    waypoints_.reset();

    waypoints_.addWP(wp);

    // If there are at least 2 waypoints,
    if (waypoints_.getSize() >= 2)
    {
      // Set the starting wp to be second last waypoint 
      waypoints_.setStartWP(waypoints_.getWP(waypoints_.getSize() - 2));
    }

    planNextWaypoint(waypoints_.getStartWP(), waypoints_.getLast());
  }

  void EGOReplanFSM::waypointsCB(const trajectory_server_msgs::WaypointsPtr &msg)
  {
    if (!have_odom_)
    {
      logError("No odom received, rejecting waypoints!");
      return;
    }
    if (msg->waypoints.poses.size() <= 0)
    {
      logError("Received empty waypoints");
      return;
    }
    if (msg->waypoints.header.frame_id != "world")
    {
      logError("Only waypoint goals in 'world' frame are accepted, ignoring waypoints.");
      return;
    }

    waypoints_.reset();

    // Transform received waypoints from world to UAV origin frame
    for (auto pose : msg->waypoints.poses) {
      waypoints_.addWP(Eigen::Vector3d{
        pose.position.x + world_to_uav_origin_tf_.position.x,
        pose.position.y + world_to_uav_origin_tf_.position.y,
        pose.position.z + world_to_uav_origin_tf_.position.z
      });
    }

    for (size_t i = 0; i < waypoints_.getSize(); i++)
    {
      visualization_->displayGoalPoint(waypoints_.getWP(i), Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
      ros::Duration(0.001).sleep();
    }

    planNextWaypoint(waypoints_.getStartWP(), waypoints_.getNextWP());
  }

  /**
   * Planning Methods
  */

  bool EGOReplanFSM::planFromGlobalTraj(const int trial_times /*=1*/) 
  {

    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();

    // If this is the first time planning has been called, then initialize a random polynomial
    bool flag_random_poly_init = (timesOfConsecutiveStateCalls().first == 1);

    for (int i = 0; i < trial_times; i++)
    {
      if (callReboundReplan(true, flag_random_poly_init))
      {
        return true;
      }
    }
    return false;
  }

  bool EGOReplanFSM::planFromLocalTraj(const int trial_times /*=1*/)
  {
    LocalTrajData *info = &planner_manager_->traj_.local_traj;
    double t_cur = ros::Time::now().toSec() - info->start_time;

    start_pt_ = info->traj.getPos(t_cur);
    start_vel_ = info->traj.getVel(t_cur);
    start_acc_ = info->traj.getAcc(t_cur);

    // Try replanning (2 + 'trial_times') number of times.
    bool success = callReboundReplan(false, false);

    if (!success)
    {
      success = callReboundReplan(true, false);
      if (!success)
      {
        for (int i = 0; i < trial_times; i++)
        {
          success = callReboundReplan(true, true);
          if (success)
            break;
        }
        if (!success)
        {
          return false;
        }
      }
    }

    return true;
  }

  bool EGOReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {
    // Get local target position and velocity 
    planner_manager_->getLocalTarget(
        planning_horizen_, start_pt_, end_pt_,
        local_target_pt_, local_target_vel_,
        touch_goal_);

    bool plan_success = planner_manager_->reboundReplan(
        start_pt_, start_vel_, 
        start_acc_, local_target_pt_, 
        local_target_vel_, 
        waypoints_.getStartWP(), waypoints_.getNextWP(), 
        (have_new_target_ || flag_use_poly_init),
        flag_randomPolyTraj, touch_goal_);

    have_new_target_ = false;

    if (plan_success)
    {
      // Get data from local trajectory and store in PolyTraj and MINCOTraj 
      traj_utils::PolyTraj poly_msg;
      traj_utils::MINCOTraj MINCO_msg;

      polyTraj2ROSMsg(poly_msg, MINCO_msg);

      transformMINCOTrajectoryToWorld(MINCO_msg);

      poly_traj_pub_.publish(poly_msg); // Publish to corresponding drone for execution
      broadcast_ploytraj_pub_.publish(MINCO_msg); // Broadcast to all other drones for replanning to optimize in avoiding swarm collision
    }

    return plan_success;
  }

  void EGOReplanFSM::planNextWaypoint(const Eigen::Vector3d previous_wp, const Eigen::Vector3d next_wp)
  {
    Eigen::Vector3d dir = (next_wp - previous_wp).normalized();
    // Offset end_pt_ by the formation position
    end_pt_ = next_wp + Eigen::Vector3d(dir(0) * formation_pos_(0) - dir(1) * formation_pos_(1),
                                        dir(1) * formation_pos_(0) + dir(0) * formation_pos_(1),
                                        formation_pos_(2));

    std::vector<Eigen::Vector3d> one_pt_wps;
    one_pt_wps.push_back(end_pt_);
    bool plan_success = planner_manager_->planGlobalTrajWaypoints(
        odom_pos_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
        one_pt_wps, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    visualization_->displayGoalPoint(next_wp, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (plan_success)
    {
      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->traj_.global_traj.duration / step_size_t);
      // gloabl_traj is only used for visualization
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->traj_.global_traj.traj.getPos(i * step_size_t);
      }

      have_target_ = true;
      have_new_target_ = true;

      // TODO Refactor
      /*** FSM ***/
      if (getServerState() != READY)
      {
        // If already executing a trajectory then wait for it to finish
        // Wait for Server to enter execute trajectory phase
        while (getServerState() != EXEC_TRAJ)
        {
          ros::spinOnce();
          ros::Duration(0.001).sleep();
        }
        // If not in READY state, Go to PLAN_LOCAL_TRAJ
        setServerEvent(PLAN_LOCAL_TRAJ_E);
      }

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      logError("Unable to generate global trajectory! Undefined actions!");
    }
  }

  /* Checking methods */

  std::pair<bool,bool> EGOReplanFSM::isGoalReachedAndReplanNeeded(){
    // boolean values to denote current state of plan execution
    bool goal_reached{false}, replan_needed{false};

    /* determine if need to replan */
    LocalTrajData *info = &planner_manager_->traj_.local_traj;
    double t_cur = ros::Time::now().toSec() - info->start_time;
    t_cur = min(info->duration, t_cur);

    // Get position at current time
    Eigen::Vector3d pos = info->traj.getPos(t_cur);
    // TODO: Try using actual position 
    // Eigen::Vector3d pos = odom_pos_;

    // Local target is within tolerance of the goal point
    bool within_goal_tol = ((local_target_pt_ - end_pt_).norm() < 1e-2);
    // Below the threshold to trigger replanning
    bool no_replan_dist = (end_pt_ - pos).norm() < min_replan_dist_;
    // Exceeded timeout to replan
    bool replan_time_exceeded = t_cur > replan_time_thresh_ ;

    if (target_type_ == TARGET_TYPE::PRESET_TARGET) 
    {
      // GOAL Reached and current time exceeded duration
      if (within_goal_tol && (t_cur > (info->duration - 1e-2)))
      {
        // Restart the target and trigger
        have_target_ = false;

        waypoints_.setStartWP(waypoints_.getNextWP());
        // planNextWaypoint(waypoints_.getStartWP(), waypoints_.getWP(0));

        /* The navigation task completed */
        goal_reached = true;
        replan_needed = false;
      }
      else if (!waypoints_.isFinalWP() && no_replan_dist)
      { 
        waypoints_.setStartWP(waypoints_.getNextWP());
        waypoints_.iterateNextWP();

        planNextWaypoint(waypoints_.getStartWP(), waypoints_.getNextWP());

        goal_reached = false;
        replan_needed = false;
      }
      else if (no_replan_dist 
              && planner_manager_->grid_map_->getInflateOccupancy(end_pt_))
      {
        have_target_ = false;

        logError("The goal is in obstacles, performing an emergency stop.");
        callEmergencyStop(odom_pos_);

        /* The navigation task completed */
        goal_reached = true;
        replan_needed = false;
      }
      else if (replan_time_exceeded 
              || (!within_goal_tol 
                  && planner_manager_->traj_.local_traj.pts_chk.back().back().first - t_cur < emergency_time_))
      {
        goal_reached = false;
        replan_needed = true;
      }
    }
    else // Manual Target
    {
      if (within_goal_tol && (t_cur > (info->duration - 1e-2))) 
      {
        // Restart the target and trigger
        have_target_ = false;

        /* The navigation task completed */
        goal_reached = true;
        replan_needed = false;
      }
      else if (no_replan_dist 
              && planner_manager_->grid_map_->getInflateOccupancy(end_pt_))
      {
        have_target_ = false;

        logError("The goal is in obstacles, performing an emergency stop.");
        callEmergencyStop(odom_pos_);

        /* The navigation task completed */
        goal_reached = true;
        replan_needed = false;
      }
      else if (replan_time_exceeded
              || (!within_goal_tol 
                  && planner_manager_->traj_.local_traj.pts_chk.back().back().first - t_cur < emergency_time_))
      {
        goal_reached = false;
        replan_needed = true;
      }
    }

    return std::make_pair(goal_reached, replan_needed);
  }

  bool EGOReplanFSM::checkSensorTimeout()
  {
    if (planner_manager_->grid_map_->getOdomDepthTimeout())
    {
      logError("Depth Image/Pose Timeout! EMERGENCY_STOP");
      enable_fail_safe_ = false;
      return true;
    }
    return false;
  }

  bool EGOReplanFSM::checkGroundHeight()
  {
    // TODO: Implement ground height checking
    double height;
    measureGroundHeight(height);

    return false;
  }

  bool EGOReplanFSM::checkTrajectoryClearance()
  {
    LocalTrajData *info = &planner_manager_->traj_.local_traj;
    
    if (info->traj_id <= 0){ // Return if no local trajectory yet
      return false;
    }

    double t_cur = ros::Time::now().toSec() - info->start_time;
    PtsChk_t pts_chk = info->pts_chk; // Points to use for checking

    const double CLEARANCE = 0.8 * planner_manager_->getSwarmClearance();
    auto id_ratio = info->traj.locatePieceIdxWithRatio(t_cur);
    // cout << "t_cur=" << t_cur << " info->duration=" << info->duration << endl;
    
    // i_start is piece index
    size_t i_start = floor((id_ratio.first + id_ratio.second) * planner_manager_->getCpsNumPrePiece());

    if (i_start >= pts_chk.size())
    {
      // logError("i_start >= pts_chk.size()");
      return false;
    }

    size_t j_start = 0; // idx of point within constraint piece
    // cout << "i_start=" << i_start << " pts_chk.size()=" << pts_chk.size() << " pts_chk[i_start].size()=" << pts_chk[i_start].size() << endl;
    for (; i_start < pts_chk.size(); ++i_start)
    {
      for (j_start = 0; j_start < pts_chk[i_start].size(); ++j_start)
      {
        // If time of point being checked exceeds current time,
        // Check for potential collision from that particular index onwards
        if (pts_chk[i_start][j_start].first > t_cur)
        {
          goto find_ij_start;
        }
      }
    }
    
    find_ij_start:;

      // Is local target near the goal?
      const bool target_near_goal = ((local_target_pt_ - end_pt_).norm() < 1e-2);
      size_t i_end = target_near_goal ? pts_chk.size() : pts_chk.size() * 3 / 4;
      for (size_t i = i_start; i < i_end; ++i)
      {
        for (size_t j = j_start; j < pts_chk[i].size(); ++j)
        {

          double t = pts_chk[i][j].first; // time
          Eigen::Vector3d pos = pts_chk[i][j].second; //position
          bool in_obs_grid = planner_manager_->grid_map_->getInflateOccupancy(pos); // Indicates if occupancy grid is occupied

          if (!in_obs_grid){
            // Iterate through trajectories of other agents
            for (size_t id = 0; id < planner_manager_->traj_.swarm_traj.size(); id++)
            {
              // Skip own trajectory or if drone ID of trajectory does not match desired ID
              // Or if the trajectory of other drones are not planned yet
              if ((planner_manager_->traj_.swarm_traj.at(id).drone_id != (int)id) ||
                  (planner_manager_->traj_.swarm_traj.at(id).drone_id == planner_manager_->pp_.drone_id))
              {
                continue;
              }

              // Calculate time for other drone
              double t_X = t - (info->start_time - planner_manager_->traj_.swarm_traj.at(id).start_time);
              // If time t_X is valid
              if (t_X > 0 && t_X < planner_manager_->traj_.swarm_traj.at(id).duration) 
              {
                Eigen::Vector3d agent_predicted_pos = planner_manager_->traj_.swarm_traj.at(id).traj.getPos(t_X);
                double dist = (pos - agent_predicted_pos).norm();

                if (dist < CLEARANCE)
                {
                  logWarn(string_format("Clearance between drone %d and drone %d is %f, too close!",
                          planner_manager_->pp_.drone_id, (int)id, dist));

                  potential_agent_collision_ = ((t - t_cur) < emergency_time_);
                  return true;
                }
              }
            }
          }
          else {
            return true;
          }

        }
        j_start = 0;
      }

      return EMPTY_E;
  }

  /**
   * Helper Methods
  */
 
  // display the FSM state along with other indicators (have_odom, have_target, have_trigger etc.)
  void EGOReplanFSM::printFSMExecState()
  {
    std::string msg{""};
    msg += string_format("[FSM]: state: %s", StateToString(getServerState()).c_str());

    if (!have_odom_)
    {
      msg += ", waiting for odom";
    }
    if (!have_target_)
    {
      msg += ", waiting for target";
    }
    // if (!have_trigger_)
    // {
    //   msg += ", waiting for trigger";
    // }
    if (planner_manager_->pp_.drone_id >= 1 && !have_recv_pre_agent_)
    {
      msg += ", haven't receive traj from previous drone";
    }

    logInfo(msg);
  }

  std::pair<int, EGOReplanFSM::ServerState> EGOReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, ServerState>(continously_called_times_, getServerState());
  }

  void EGOReplanFSM::polyTraj2ROSMsg(traj_utils::PolyTraj &poly_msg, traj_utils::MINCOTraj &MINCO_msg)
  {
    // Get local trajectory
    auto data = &planner_manager_->traj_.local_traj;

    Eigen::VectorXd durs = data->traj.getDurations();
    int piece_num = data->traj.getPieceNum();
    poly_msg.drone_id = planner_manager_->pp_.drone_id;
    poly_msg.traj_id = data->traj_id;
    poly_msg.start_time = ros::Time(data->start_time);
    poly_msg.order = 5; // todo, only support order = 5 now.
    poly_msg.duration.resize(piece_num);
    poly_msg.coef_x.resize(6 * piece_num);
    poly_msg.coef_y.resize(6 * piece_num);
    poly_msg.coef_z.resize(6 * piece_num);

    // For each point
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

    MINCO_msg.drone_id = planner_manager_->pp_.drone_id;
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

  bool EGOReplanFSM::measureGroundHeight(double &height)
  {
    if (planner_manager_->traj_.local_traj.pts_chk.size() < 3) // means planning have not started
    {
      return false;
    }

    auto traj = &planner_manager_->traj_.local_traj;
    auto map = planner_manager_->grid_map_;
    ros::Time t_now = ros::Time::now();

    double forward_t = 2.0 / planner_manager_->pp_.max_vel_; //2.0m
    double traj_t = (t_now.toSec() - traj->start_time) + forward_t;
    if (traj_t <= traj->duration)
    {
      Eigen::Vector3d forward_p = traj->traj.getPos(traj_t);

      double reso = map->getResolution();
      for (;; forward_p(2) -= reso)
      {
        int ret = map->getOccupancy(forward_p);
        if (ret == -1) // reach map bottom
        {
          return false;
        }
        if (ret == 1) // reach the ground
        {
          height = forward_p(2);

          std_msgs::Float64 height_msg;
          height_msg.data = height;
          ground_height_pub_.publish(height_msg);

          return true;
        }
      }
    }

    return false;
  }

  bool EGOReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {
    planner_manager_->EmergencyStop(stop_pos);

    traj_utils::PolyTraj poly_msg;
    traj_utils::MINCOTraj MINCO_msg;

    polyTraj2ROSMsg(poly_msg, MINCO_msg);

    transformMINCOTrajectoryToWorld(MINCO_msg);

    poly_traj_pub_.publish(poly_msg); // Publish to own trajectory server under current drone_id for execution
    broadcast_ploytraj_pub_.publish(MINCO_msg); // Broadcast to all other drones for replanning to optimize in avoiding swarm collision

    return true;
  }

} // namespace ego_planner
