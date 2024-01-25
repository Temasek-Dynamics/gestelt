
#include <ego_planner_fsm/ego_planner_fsm.h>

namespace ego_planner
{
  void EGOReplanFSM::init(ros::NodeHandle &nh, ros::NodeHandle &pnh)
  {

    have_target_ = false;
    have_odom_ = false;
    have_recv_pre_agent_ = false;
    flag_escape_emergency_ = true;

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(pnh));
    planner_manager_.reset(new EGOPlannerManager);
    planner_manager_->initPlanModules(nh, pnh, visualization_);

    /* Benchmarking */
    // time_benchmark_ = std::make_shared<TimeBenchmark>();
    // time_benchmark_->add_ids({
    //   std:vector<std::string>{
    //     "planFromLocalTraj", 
    //     "grid_map_update_occupancy", 
    //   }
    // });
    // planner_manager_->grid_map_->initTimeBenchmark(time_benchmark_);

    /*  fsm param  */
    pnh.param("fsm/waypoint_type", waypoint_type_, -1);
    pnh.param("fsm/thresh_replan_time", replan_time_thresh_, -1.0);
    pnh.param("fsm/thresh_no_replan_meter", min_replan_dist_, -1.0);
    pnh.param("fsm/planning_horizon", planning_horizon_, -1.0);
    pnh.param("fsm/emergency_time", emergency_time_, 1.0);
    pnh.param("fsm/fail_safe", enable_fail_safe_, true);

    // int formation_num = -1;
    // pnh.param("formation/num", formation_num, -1);
    // if (formation_num < planner_manager_->pp_.drone_id + 1)
    // {
    //   logError("formation_num is smaller than the drone number, illegal!");
    //   return;
    // }
    // std::vector<double> pos;
    // pnh.getParam("formation/drone" + std::to_string(planner_manager_->pp_.drone_id), pos);
    // formation_pos_ << pos[0], pos[1], pos[2];
    // pnh.getParam("formation/start", pos);
 
    // Eigen::Vector3d formation_start;
    // formation_start << pos[0], pos[1], pos[2];
    // waypoints_.setStartWP(formation_start);

    // double pub_state_freq, tick_state_freq, exec_state_freq;
    // pnh.param("fsm/pub_state_freq", pub_state_freq, 10.0);
    // pnh.param("fsm/tick_state_freq", tick_state_freq, 100.0);
    // pnh.param("fsm/exec_state_freq", exec_state_freq, 20.0);

    // std::string odom_topic;
    // pnh.param("grid_map/odom", odom_topic, std::string("odom"));

    /* Timer callbacks */
    // pub_state_timer_ = nh.createTimer(ros::Duration(1/pub_state_freq), &EGOReplanFSM::pubStateTimerCB, this);
    // tick_state_timer_ = nh.createTimer(ros::Duration(1/tick_state_freq), &EGOReplanFSM::tickStateTimerCB, this);
    // exec_state_timer_ = nh.createTimer(ros::Duration(1/exec_state_freq), &EGOReplanFSM::execStateTimerCB, this);

    /* Subscribers */
    // odom_sub_ = nh.subscribe("fsm_odom", 1, &EGOReplanFSM::odometryCallback, this);
    // mandatory_stop_sub_ = nh.subscribe("/mandatory_stop_to_planner", 1, &EGOReplanFSM::mandatoryStopCallback, this);

    // Use MINCO trajectory to minimize the message size in wireless communication
    broadcast_ploytraj_sub_ = nh.subscribe<traj_utils::MINCOTraj>("/broadcast_traj_to_planner", 100,
                                                                  &EGOReplanFSM::RecvBroadcastMINCOTrajCallback,
                                                                  this,
                                                                  ros::TransportHints().tcpNoDelay());

    /* Publishers */
    broadcast_ploytraj_pub_ = nh.advertise<traj_utils::MINCOTraj>("/broadcast_traj_from_planner", 10);
    poly_traj_pub_ = pnh.advertise<traj_utils::PolyTraj>("planner/trajectory", 10);
    heartbeat_pub_ = pnh.advertise<std_msgs::Empty>("planner/heartbeat", 10);
    // ground_height_pub_ = pnh.advertise<std_msgs::Float64>("/ground_height_measurement", 10);
    state_pub_ = pnh.advertise<std_msgs::String>("planner/state", 10);
    // time_benchmark_pub_ = nh.advertise<gestelt_msgs::TimeBenchmark>("plan_time_benchmark", 10);

    // if (waypoint_type_ == TARGET_TYPE::MANUAL_TARGET)
    // {
    //   waypoint_sub_ = nh.subscribe("/goal", 1, &EGOReplanFSM::waypointCB, this);
    // }
    // else if (waypoint_type_ == TARGET_TYPE::PRESET_TARGET)
    // {
    //   // Subscribe to waypoints 
    //   waypoints_sub_ = nh.subscribe("planner/goals", 1, &EGOReplanFSM::waypointsCB, this);
    // }
    // else{
    //   logError(str_fmt("Invalid waypoint type value! target_type=%i, it is either 1 or 2", waypoint_type_));
    // }

    /* Get Transformation from origin to world frame and vice versa */
    pnh.param("grid_map/uav_origin_frame", uav_origin_frame_, std::string("world"));
    pnh.param("grid_map/global_frame", global_frame_, std::string("world"));
    pnh.param("fsm/tf_lookup_timeout", tf_lookup_timeout_, 60.0);

    // tfListener_.reset(new tf2_ros::TransformListener(tfBuffer_));

    // geometry_msgs::TransformStamped transform;
    // try
    // {
    //   transform = tfBuffer_.lookupTransform(global_frame_, uav_origin_frame_, ros::Time(0), ros::Duration(tf_lookup_timeout_));
    // }
    // catch (const tf2::TransformException &ex)
    // {
    //   ROS_ERROR_THROTTLE(1,
    //       "[Ego Planner FSM]: Error in lookupTransform of %s in %s", uav_origin_frame_.c_str(), global_frame_.c_str());
    //   ROS_WARN_THROTTLE(1, "%s",ex.what());
    //   ros::shutdown();
    // }

    // // Transformation from world frame to UAV frame
    // uav_origin_to_world_tf_(0) = transform.transform.translation.x;
    // uav_origin_to_world_tf_(1) = transform.transform.translation.y;
    // uav_origin_to_world_tf_(2) = transform.transform.translation.z;

    // // Reverse signs so that the transformation is from UAV origin frame to world frame
    // world_to_uav_origin_tf_(0) = -uav_origin_to_world_tf_(0);
    // world_to_uav_origin_tf_(1) = -uav_origin_to_world_tf_(1);
    // world_to_uav_origin_tf_(2) = -uav_origin_to_world_tf_(2);

    // Transformation from world frame to UAV frame
    uav_origin_to_world_tf_(0) = 0.0;
    uav_origin_to_world_tf_(1) = 0.0;
    uav_origin_to_world_tf_(2) = 0.0;

    // Reverse signs so that the transformation is from UAV origin frame to world frame
    world_to_uav_origin_tf_(0) = 0.0;
    world_to_uav_origin_tf_(1) = 0.0;
    world_to_uav_origin_tf_(2) = 0.0;

    debug_start_sub_ = pnh.subscribe("debug/plan_start", 5, &EGOReplanFSM::debugStartCB, this);
    debug_goal_sub_ = pnh.subscribe("debug/plan_goal", 5, &EGOReplanFSM::debugGoalCB, this);

  }

  void EGOReplanFSM::debugStartCB(const geometry_msgs::PoseConstPtr &msg)
  {
    ROS_INFO("[EGOReplanFSM]: Received debug start (%f, %f, %f)", 
          msg->position.x,
          msg->position.y,
          msg->position.z);

    odom_pos_(0) = msg->position.x ;
    odom_pos_(1) = msg->position.y ;
    odom_pos_(2) = msg->position.z ;

    odom_vel_(0) = 0.0;
    odom_vel_(1) = 0.0;
    odom_vel_(2) = 0.0;

    have_odom_ = true;
  }

  /**
   * Timer Callbacks
  */
  void EGOReplanFSM::pubStateTimerCB(const ros::TimerEvent &e)
  {
    std_msgs::Empty heartbeat_msg;
    heartbeat_pub_.publish(heartbeat_msg);

    std_msgs::String planner_state;
    planner_state.data = StateToString(getServerState());
    state_pub_.publish(planner_state);

    // Publish time benchmarks
    // gestelt_msgs::TimeBenchmark time_bench_msg;
    // time_bench_msg.planner_cpu_time = time_benchmark_->get_elapsed_cpu_time("planFromLocalTraj");
    // time_bench_msg.planner_wall_time = time_benchmark_->get_elapsed_wall_time("planFromLocalTraj");

    // time_bench_msg.gridmap_update_occ_cpu_time = time_benchmark_->get_elapsed_cpu_time("grid_map_update_occupancy");
    // time_bench_msg.gridmap_update_occ_wall_time = time_benchmark_->get_elapsed_wall_time("grid_map_update_occupancy");
    
    // time_benchmark_pub_.publish(time_bench_msg);

  }

  void EGOReplanFSM::tickStateTimerCB(const ros::TimerEvent &e)
  {
    // logInfoThrottled(str_fmt("Current State: [%s]", StateToString(getServerState()).c_str()), 1.0);
    
    static int fsm_num = 0;
    if (fsm_num++ == 500)
    {
      fsm_num = 0;
      // printFSMExecState();
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
        ROS_INFO("READY");
        if (have_target_) {
          setServerEvent(PLAN_GLOBAL_TRAJ_E);
        }
        break;
      }

      case PLAN_GLOBAL_TRAJ: 
      {
        ROS_INFO("PLAN_GLOBAL_TRAJ");
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
        ROS_INFO("PLAN_LOCAL_TRAJ");
        if (planFromLocalTraj(3))
        {
          setServerEvent(EXEC_TRAJ_E);
        }
        else 
        {
          logError(str_fmt("Plan from local trajectory failed, possibly from potential collision!"));
          if (potential_agent_to_agent_collision_)
          {
            logError(str_fmt("Potential agent to agent collision detected, ESTOP has been disabled from activation for debugging"));

            // logError(str_fmt("Potential agent to agent collision detected, activating ESTOP"));
            // setServerEvent(EMERGENCY_STOP_E);
          }
        }
        break;
      }

      case EXEC_TRAJ:
      {
        ROS_INFO("EXEC_TRAJ");
        if (checkSensorTimeout())
        {
          setServerEvent(EMERGENCY_STOP_E);
          break;
        }

        if (checkTrajectoryClearance())
        {
          logInfo(str_fmt("Replanning to avoid collision."));
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
      logWarn(str_fmt("Received drone %d's trajectory out of order or duplicated, abandon it.", (int)recv_id));
      return;
    }

    ros::Time t_now = ros::Time::now();
    if (abs((t_now - minco_traj.start_time).toSec()) > 0.25)
    {

      if (abs((t_now - minco_traj.start_time).toSec()) < 10.0) // 10 seconds offset, more likely to be caused by unsynced system time.
      {
        logWarn(str_fmt("Time stamp diff: Local - Remote Agent %d = %fs",
                 minco_traj.drone_id, (t_now - minco_traj.start_time).toSec()));
      }
      else
      {
        logError(str_fmt("Time stamp diff: Local - Remote Agent %d = %fs, swarm time seems not synchronized, abandon!",
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
    transformMINCOFromWorldToOrigin(minco_traj);

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
      logError(str_fmt("Imminent COLLISION between ownself and drone %d", recv_id));
      // TODO: What state is it expected to be in? EXEC_TRAJ?
      setServerState(PLAN_LOCAL_TRAJ);
    }

    /* Check if receive agents have lower drone id */
    if (!have_recv_pre_agent_)
    {
      // If the number of trajectories exceed or are same as current drone id
      if ((int)planner_manager_->traj_.swarm_traj.size() >= planner_manager_->pp_.drone_id)
      {
        // For each drone id up to current drone id
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

  void EGOReplanFSM::waypointCB(const geometry_msgs::PoseStampedPtr &msg)
  {
    Eigen::Vector3d wp(
        msg->pose.position.x,
        msg->pose.position.y,
        msg->pose.position.z);

    wp += world_to_uav_origin_tf_;

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

  void EGOReplanFSM::waypointsCB(const gestelt_msgs::GoalsPtr &msg)
  {
    if (!have_odom_)
    {
      logError("No odom received, rejecting waypoints!");
      return;
    }
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

    waypoints_.reset();

    for (auto& pos : msg->transforms) {
      // Transform received waypoints from world to UAV origin frame
      waypoints_.addWP(Eigen::Vector3d{pos.translation.x, pos.translation.y, pos.translation.z} + world_to_uav_origin_tf_);
    }

    for (size_t i = 0; i < waypoints_.getSize(); i++)
    {
      visualization_->displayGoalPoint(waypoints_.getWP(i), Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
      ros::Duration(0.001).sleep();
    }

    planNextWaypoint(waypoints_.getStartWP(), waypoints_.getNextWP());
  }

  void EGOReplanFSM::debugGoalCB(const geometry_msgs::PoseConstPtr &msg)
  {
    ROS_INFO("[EGOReplanFSM]: Received debug goal (%f, %f, %f)", 
          msg->position.x,
          msg->position.y,
          msg->position.z);

    goal_pos_ = Eigen::Vector3d{
          msg->position.x,
          msg->position.y,
          msg->position.z};

    planNextWaypoint(odom_pos_, goal_pos_);

    if (!planFromGlobalTraj(5)){
      ROS_ERROR("Failed to plan from global trajectory");
      return;
    }
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
    bool flag_random_poly_init = (continously_called_times_ == 1);
    bool success = false;
    for (int i = 0; i < trial_times; i++)
    {
      if (getLocalTargetAndReboundReplan(true, flag_random_poly_init))
      {
        success = true;
        break;
      }
    }

    return success;
  }

  bool EGOReplanFSM::planFromLocalTraj(const int trial_times /*=1*/)
  {
    // std::string benchmark_id = "planFromLocalTraj";
    // time_benchmark_->start_stopwatch(benchmark_id);

    LocalTrajData *info = &planner_manager_->traj_.local_traj;
    double t_cur = ros::Time::now().toSec() - info->start_time;

    start_pt_ = info->traj.getPos(t_cur);
    start_vel_ = info->traj.getVel(t_cur);
    start_acc_ = info->traj.getAcc(t_cur);

    // Start without initializing new poly and no randomized poly
    bool success = getLocalTargetAndReboundReplan(false, false);

    if (!success)
    {
      // Retry: Initialize new poly and no randomized poly
      success = getLocalTargetAndReboundReplan(true, false);
      if (!success)
      {
        for (int i = 0; i < trial_times; i++)
        {
          // Retry: Initialize new poly and randomized poly
          if (getLocalTargetAndReboundReplan(true, true)){
            success = true;
            break;
          }
        }
      }
    }

    // time_benchmark_->stop_stopwatch(benchmark_id);

    return success;
  }

  bool EGOReplanFSM::getLocalTargetAndReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {
    // Get local target position and velocity, with planning horizon
    planner_manager_->getLocalTarget(
        planning_horizon_, start_pt_, end_pt_,
        local_target_pt_, local_target_vel_,
        touch_goal_);

    bool flag_polyInit = (have_new_target_ || flag_use_poly_init);
    
    bool plan_success = planner_manager_->reboundReplan(
        start_pt_, start_vel_, start_acc_, 
        local_target_pt_, local_target_vel_, 
        flag_polyInit,
        flag_randomPolyTraj, touch_goal_);

    have_new_target_ = false;

    if (plan_success)
    {
      // Get data from local trajectory and store in PolyTraj and MINCOTraj 
      traj_utils::PolyTraj poly_msg; 
      traj_utils::MINCOTraj MINCO_msg; 

      polyTraj2ROSMsg(poly_msg, MINCO_msg);

      transformMINCOFromOriginToWorld(MINCO_msg);

      poly_traj_pub_.publish(poly_msg); // (In drone origin frame) Publish to corresponding drone for execution
      broadcast_ploytraj_pub_.publish(MINCO_msg); // (In world frame) Broadcast to all other drones for replanning to optimize in avoiding swarm collision
    }
    else {
      ROS_ERROR("planner_manager_->reboundReplan not successful");
    }

    return plan_success;
  }

  void EGOReplanFSM::planNextWaypoint(const Eigen::Vector3d previous_wp, const Eigen::Vector3d next_wp)
  {
    // Eigen::Vector3d dir = (next_wp - previous_wp).normalized();
    // Offset end_pt_ by the formation position
    // end_pt_ = next_wp + Eigen::Vector3d(dir(0) * formation_pos_(0) - dir(1) * formation_pos_(1),
    //                                     dir(1) * formation_pos_(0) + dir(0) * formation_pos_(1),
    //                                     formation_pos_(2));

    end_pt_ = next_wp;

    std::vector<Eigen::Vector3d> one_pt_wps;
    one_pt_wps.push_back(end_pt_);

    // Plan global trajectory starting from agent's current position
    // with 0 starting/ending acceleration and velocity. With a single waypoint
    poly_traj::MinJerkOpt globalMJO; // Global minimum jerk trajectory
    bool plan_success = planner_manager_->planGlobalTrajWaypoints(
        globalMJO,
        odom_pos_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
        one_pt_wps, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    planner_manager_->traj_.setGlobalTraj(globalMJO.getTraj(), ros::Time::now().toSec());

    visualization_->displayGoalPoint(next_wp, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (plan_success)
    {
      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->traj_.global_traj.duration / step_size_t);
      // global_traj is only used for visualization
      vector<Eigen::Vector3d> global_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        global_traj[i] = planner_manager_->traj_.global_traj.traj.getPos(i * step_size_t);
      }

      have_target_ = true;
      have_new_target_ = true;
      
      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(global_traj, 0.1, 0);
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
    t_cur = std::min(info->duration, t_cur);

    // Get position at current time
    Eigen::Vector3d pos = info->traj.getPos(t_cur);

    // Local target is within tolerance of the goal point
    bool within_goal_tol = ((local_target_pt_ - end_pt_).norm() < 1e-2);
    // Below the threshold to trigger replanning
    bool no_replan_dist = (end_pt_ - pos).norm() < min_replan_dist_;
    // Exceeded timeout to replan
    bool replan_time_exceeded = t_cur > replan_time_thresh_ ;

    if (waypoint_type_ == TARGET_TYPE::PRESET_TARGET) 
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
    if (planner_manager_->grid_map_->getPoseDepthTimeout())
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
    // std::cout << "t_cur=" << t_cur << " info->duration=" << info->duration << std::endl;
    
    // i_start is piece index
    size_t i_start = floor((id_ratio.first + id_ratio.second) * planner_manager_->getCpsNumPrePiece());

    if (i_start >= pts_chk.size())
    {
      // logError("i_start >= pts_chk.size()");
      return false;
    }

    size_t j_start = 0; // idx of point within constraint piece
    // std::cout << "i_start=" << i_start << " pts_chk.size()=" << pts_chk.size() << " pts_chk[i_start].size()=" << pts_chk[i_start].size() << std::endl;
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

      // Iterate through each piece index
      for (size_t i = i_start; i < i_end; ++i)
      {
        // Iterate through each point within piece index
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
                  logWarn(str_fmt("Clearance between drones %d and %d is %f, too close!",
                          planner_manager_->pp_.drone_id, (int)id, dist));

                  potential_agent_to_agent_collision_ = ((t - t_cur) < emergency_time_);
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
    msg += str_fmt("[FSM]: state: %s", StateToString(getServerState()).c_str());

    if (!have_odom_)
    {
      msg += ", waiting for odom";
    }
    if (!have_target_)
    {
      msg += ", waiting for target";
    }
    if (planner_manager_->pp_.drone_id >= 1 && !have_recv_pre_agent_)
    {
      msg += ", haven't receive traj from previous drone";
    }

    logInfo(msg);
  }

  void EGOReplanFSM::polyTraj2ROSMsg(traj_utils::PolyTraj &poly_msg, traj_utils::MINCOTraj &MINCO_msg)
  {
    // Get local trajectory
    auto data = &planner_manager_->traj_.local_traj;

    Eigen::VectorXd durs = data->traj.getDurations();
    int piece_num = data->traj.getPieceSize();
    poly_msg.drone_id = planner_manager_->pp_.drone_id;
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

    transformMINCOFromOriginToWorld(MINCO_msg);

    poly_traj_pub_.publish(poly_msg); // Publish to own trajectory server under current drone_id for execution
    broadcast_ploytraj_pub_.publish(MINCO_msg); // Broadcast to all other drones for replanning to optimize in avoiding swarm collision

    return true;
  }

} // namespace ego_planner
