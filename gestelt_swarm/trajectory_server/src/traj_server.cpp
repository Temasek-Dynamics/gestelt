#include <traj_server/traj_server.h>

using namespace Eigen;

/* Initialization methods */

void TrajServer::init(ros::NodeHandle& nh)
{
  /* ROS Params*/
  nh.param("drone_id", drone_id_, 0);
  nh.param("origin_frame", origin_frame_, std::string("world"));
  std::string drone_model_mesh_filepath;
  nh.param("drone_model_mesh", drone_model_mesh_filepath, std::string(""));

  nh.param("traj_server/takeoff_height", takeoff_height_, 1.0);
  nh.param("traj_server/time_forward", time_forward_, -1.0);
  nh.param("traj_server/pub_cmd_freq", pub_cmd_freq_, 25.0);
  double sm_tick_freq; // Frequency to tick the state machine transitions
  nh.param("traj_server/state_machine_tick_freq", sm_tick_freq, 50.0);
  double server_state_pub_freq;
  nh.param("traj_server/server_state_pub_freq", server_state_pub_freq, 5.0);
  double debug_freq;
  nh.param("traj_server/debug_freq", debug_freq, 10.0);

  nh.param("traj_server/max_poses_to_track", max_poses_to_track_, 250);
  nh.param("traj_server/error_tracking_window", error_tracking_window_, 2.5);

  nh.param("traj_server/planner_heartbeat_timeout", planner_heartbeat_timeout_, 0.5);
  nh.param("traj_server/ignore_heartbeat", ignore_heartbeat_, false);

  nh.param("traj_server/pos_limit/max_x", position_limits_.max_x, -1.0);
  nh.param("traj_server/pos_limit/min_x", position_limits_.min_x, -1.0);
  nh.param("traj_server/pos_limit/max_y", position_limits_.max_y, -1.0);
  nh.param("traj_server/pos_limit/min_y", position_limits_.min_y, -1.0);
  nh.param("traj_server/pos_limit/max_z", position_limits_.max_z, -1.0);
  nh.param("traj_server/pos_limit/min_z", position_limits_.min_z, -1.0);
  
  nh.param("traj_server/mode", traj_mode_, 0);


  /* Subscribers */
  if (traj_mode_ == TrajMode::POLYTRAJ) {
    traj_sub_ = nh.subscribe("planning/trajectory", 10, &TrajServer::polyTrajCallback, this);
  }
  else if (traj_mode_ == TrajMode::MULTIDOFJOINTTRAJECTORY) {
    traj_sub_ = nh.subscribe("planning/trajectory", 10, &TrajServer::multiDOFJointTrajectoryCb, this);
  }

  heartbeat_sub_ = nh.subscribe("heartbeat", 10, &TrajServer::heartbeatCallback, this);
  uav_state_sub_ = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, &TrajServer::UAVStateCb, this);
  planner_state_sub_ = nh.subscribe("/planner_state", 10, &TrajServer::plannerStateCB, this);

  pose_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, &TrajServer::UAVPoseCB, this);
  odom_sub_ = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 1, &TrajServer::UAVOdomCB, this);

  set_server_state_sub_ = nh.subscribe<std_msgs::Int8>("/traj_server_event", 10, &TrajServer::serverEventCb, this);

  /* Publishers */
  server_state_pub_ = nh.advertise<trajectory_server_msgs::State>("/server_state", 50);
  pos_cmd_raw_pub_ = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 50);
  uav_mesh_pub_ = nh.advertise<visualization_msgs::Marker>("model", 50);

  uav_path_pub_ = nh.advertise<nav_msgs::Path>("/uav_path", 50);

  tracking_error_pub_ = nh.advertise<trajectory_server_msgs::TrackingError>("/tracking_error", 50);

  /* Service clients */
  arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
  set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

  /* Timer callbacks */
  exec_traj_timer_ = nh.createTimer(ros::Duration(1/pub_cmd_freq_), &TrajServer::execTrajTimerCb, this);
  tick_state_timer_ = nh.createTimer(ros::Duration(1/sm_tick_freq), &TrajServer::tickServerStateTimerCb, this);
  state_pub_timer_ = nh.createTimer(ros::Duration(1/server_state_pub_freq), &TrajServer::pubServerStateTimerCb, this);
  debug_timer_ = nh.createTimer(ros::Duration(1/debug_freq), &TrajServer::debugTimerCb, this);

  initModelMesh(drone_model_mesh_filepath);

  logInfo("Initialized");
}

void TrajServer::initModelMesh(const std::string& drone_model_mesh_filepath){

  // input mesh model
  model_mesh_.header.frame_id = origin_frame_;
  model_mesh_.ns = "drone" + std::to_string(drone_id_);
  model_mesh_.id = drone_id_;
  model_mesh_.type = visualization_msgs::Marker::MESH_RESOURCE;
  model_mesh_.action = visualization_msgs::Marker::ADD;

  model_mesh_.scale.x = 0.4;
  model_mesh_.scale.y = 0.4;
  model_mesh_.scale.z = 0.4;
  model_mesh_.color.a = 0.7;
  model_mesh_.color.r = 0.5;
  model_mesh_.color.g = 0.5;
  model_mesh_.color.b = 0.5;
  model_mesh_.mesh_resource = drone_model_mesh_filepath;
}

/* Subscriber Callbacks */

void TrajServer::heartbeatCallback(std_msgs::EmptyPtr msg)
{
  heartbeat_time_ = ros::Time::now();
}

void TrajServer::polyTrajCallback(traj_utils::PolyTrajPtr msg)
{
  if (msg->order != 5)
  {
    logError("[traj_server] Only support trajectory order equals 5 now!");
    return;
  }
  if (msg->duration.size() * (msg->order + 1) != msg->coef_x.size())
  {
    logError("[traj_server] WRONG trajectory parameters, ");
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
    cMats[i].row(0) << msg->coef_x[i6 + 0], msg->coef_x[i6 + 1], msg->coef_x[i6 + 2],
        msg->coef_x[i6 + 3], msg->coef_x[i6 + 4], msg->coef_x[i6 + 5];
    cMats[i].row(1) << msg->coef_y[i6 + 0], msg->coef_y[i6 + 1], msg->coef_y[i6 + 2],
        msg->coef_y[i6 + 3], msg->coef_y[i6 + 4], msg->coef_y[i6 + 5];
    cMats[i].row(2) << msg->coef_z[i6 + 0], msg->coef_z[i6 + 1], msg->coef_z[i6 + 2],
        msg->coef_z[i6 + 3], msg->coef_z[i6 + 4], msg->coef_z[i6 + 5];

    dura[i] = msg->duration[i];
  }

  traj_.reset(new poly_traj::Trajectory(dura, cMats));

  start_time_ = msg->start_time;
  traj_duration_ = traj_->getTotalDuration();
  // traj_id_ = msg->traj_id;

  startMission();
}

void TrajServer::multiDOFJointTrajectoryCb(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr &msg)
{
  if (getServerState() != ServerState::MISSION){ 
    logError("Executing Joint Trajectory while not in MISSION mode. Ignoring!");
    return;
  }

  startMission();

  // msg.joint_names: contain "base_link"
  // msg.points: Contains only a single point
  // msg.points[0].time_from_start: current_sample_time_

  std::string frame_id = msg->joint_names[0];

  // Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), 
  //                 acc(Eigen::Vector3d::Zero()), jer(Eigen::Vector3d::Zero());
  // std::pair<double, double> yaw_yawdot(0, 0);

  geomMsgsVector3ToEigenVector3(msg->points[0].transforms[0].translation, last_mission_pos_);
  geomMsgsVector3ToEigenVector3(msg->points[0].velocities[0].linear, last_mission_vel_);

  geomMsgsVector3ToEigenVector3(msg->points[0].accelerations[0].linear, last_mission_acc_);

  last_mission_yaw_ = quaternionToYaw(msg->points[0].transforms[0].rotation);
  last_mission_yaw_dot_ = msg->points[0].velocities[0].angular.z; //yaw rate
}

void TrajServer::plannerStateCB(const std_msgs::String::ConstPtr &msg)
{
  planner_state_ = msg->data;
}

void TrajServer::UAVStateCb(const mavros_msgs::State::ConstPtr &msg)
{
  // logInfoThrottled(string_format("State: Mode[%s], Connected[%d], Armed[%d]", msg->mode.c_str(), msg->connected, msg->armed), 1.0);
  uav_current_state_ = *msg;
}

void TrajServer::UAVPoseCB(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  uav_pose_ = *msg; 
  uav_poses_.push_back(uav_pose_);

  if (uav_poses_.size() > max_poses_to_track_) {
    uav_poses_.pop_front(); // Remove the oldest pose
  }

  // Publish mesh visualization
	model_mesh_.pose = msg->pose;
  model_mesh_.header.stamp = msg->header.stamp;

	uav_mesh_pub_.publish(model_mesh_);
}

void TrajServer::UAVOdomCB(const nav_msgs::Odometry::ConstPtr &msg)
{
  uav_odom_ = *msg;
}

void TrajServer::serverEventCb(const std_msgs::Int8::ConstPtr & msg)
{
  if (msg->data < 0 || msg->data > ServerEvent::EMPTY_E){
    logError("Invalid server event, ignoring...");
  } 
  setServerEvent(ServerEvent(msg->data));
}

/* Timer Callbacks */

void TrajServer::execTrajTimerCb(const ros::TimerEvent &e)
{
  switch (getServerState()){
    case ServerState::INIT:
      // Do nothing, drone is not initialized
      break;
    
    case ServerState::IDLE:
      // Do nothing, drone has not taken off
      break;
    
    case ServerState::TAKEOFF:
      execTakeOff();
      break;
    
    case ServerState::LAND:
      execLand();
      break;
    
    case ServerState::HOVER:
      execHover();
      break;
    
    case ServerState::MISSION:
      if (isMissionComplete()){
        // logInfoThrottled("Waiting for mission", 5.0);
        execHover();
      }
      else {
        if (!ignore_heartbeat_ && isPlannerHeartbeatTimeout()){
          logErrorThrottled("[traj_server] Lost heartbeat from the planner.", 1.0);
          execHover();
        }

        execMission();
      }
      break;

    case ServerState::E_STOP:
      // Do nothing, drone should stop all motors immediately
      break;
  }
}

void TrajServer::tickServerStateTimerCb(const ros::TimerEvent &e)
{
  // logInfoThrottled(string_format("Current Server State: [%s]", StateToString(getServerState()).c_str()), 1.0);

  switch (getServerState())
  {
    case ServerState::INIT:
      {
        // Wait for FCU Connection
        if (uav_current_state_.connected){
          logInfo("[INIT] Connected to flight stack!");
          setServerState(ServerState::IDLE);
        }
        else {
          logInfoThrottled("[INIT] Initializing Server, waiting for connection to FCU...", 2.0 );
        }

        break;
      }
    case ServerState::IDLE:
      // logInfoThrottled("[IDLE] Ready to take off", 5.0 );

      switch (getServerEvent())
      {
        case TAKEOFF_E:
          logInfo("[IDLE] UAV Attempting takeoff");
          setServerState(ServerState::TAKEOFF);
          break;
        case LAND_E:
          logWarn("[IDLE] IGNORED EVENT. UAV has not taken off, unable to LAND");
          break;
        case MISSION_E:
          logWarn("[IDLE] IGNORED EVENT. Please TAKEOFF first before setting MISSION mode");
          break;
        case HOVER_E:
          logWarn("[IDLE] IGNORED EVENT. No mission to cancel");
          break;
        case E_STOP_E:
          logFatal("[IDLE] EMERGENCY STOP ACTIVATED!");
          setServerState(ServerState::E_STOP);
          break;
        case EMPTY_E:
          // Default case if no event sent
          break;
      }
      break;
    
    case ServerState::TAKEOFF:

      switch (getServerEvent())
      {
        case TAKEOFF_E:
          // logWarn("[TAKEOFF] IGNORED EVENT. UAV already attempting taking off");
          break;
        case LAND_E:
          logInfo("[TAKEOFF] Attempting landing");
          setServerState(ServerState::LAND);
          break;
        case MISSION_E:
          logWarn("[TAKEOFF] IGNORED EVENT. Wait until UAV needs to take off before accepting mission command");
          break;
        case HOVER_E:
          logWarn("[TAKEOFF] IGNORED EVENT. No mission to cancel");
          break;
        case E_STOP_E:
          logFatal("[TAKEOFF] EMERGENCY STOP ACTIVATED!");
          setServerState(ServerState::E_STOP);
          break;
        case EMPTY_E:
          // Default case if no event sent
          break;
      }

      if (!is_uav_ready()){
        logInfo("[TAKEOFF] Calling toggle offboard mode");
        toggle_offboard_mode(true);
      }

      if (isTakenOff()){
        logInfo("[TAKEOFF] Take off complete");
        setServerState(ServerState::HOVER);
      }
      else {
        logInfoThrottled("[TAKEOFF] Taking off...", 1.0 );
      }

      break;
    
    case ServerState::LAND:

      switch (getServerEvent())
      {
        case TAKEOFF_E:
          logInfo("[LAND] UAV Attempting takeoff");
          setServerState(ServerState::TAKEOFF);
          break;
        case LAND_E:
          logWarn("[LAND] IGNORED EVENT. UAV already attempting landing");
          break;
        case MISSION_E:
          logWarn("[LAND] IGNORED EVENT. UAV is landing, it needs to take off before accepting mission command");
          break;
        case HOVER_E:
          logWarn("[LAND] IGNORED EVENT. No mission to cancel");
          break;
        case E_STOP_E:
          logFatal("[LAND] EMERGENCY STOP ACTIVATED!");
          setServerState(ServerState::E_STOP);
          break;
        case EMPTY_E:
          // Default case if no event sent
          break;
      }

      if (isLanded()){
        logInfo("[LAND] Landing complete");
        setServerState(ServerState::IDLE);
      }
      else {
        logInfoThrottled("[LAND] landing...", 1.0);
      }

      setServerState(ServerState::IDLE);
      break;
    
    case ServerState::HOVER:

      switch (getServerEvent())
      {
        case TAKEOFF_E:
          logWarn("[HOVER] IGNORED EVENT. UAV already took off. Currently in [HOVER] mode");
          break;
        case LAND_E:
          logInfo("[HOVER] Attempting landing");
          setServerState(ServerState::LAND);
          break;
        case MISSION_E:
          logInfo("[HOVER] UAV entering [MISSION] mode.");
          setServerState(ServerState::MISSION);
          break;
        case HOVER_E:
          logWarn("[HOVER] IGNORED EVENT. No mission to cancel");
          break;
        case E_STOP_E:
          logFatal("[HOVER] EMERGENCY STOP ACTIVATED!");
          setServerState(ServerState::E_STOP);
          break;
        case EMPTY_E:
          // Default case if no event sent
          break;
      }

      break;
    
    case ServerState::MISSION:

      switch (getServerEvent())
      {
        case TAKEOFF_E:
          logWarn("[MISSION] IGNORED EVENT. UAV already took off. Currently in [MISSION] mode");
          break;
        case LAND_E:
          logWarn("[MISSION] Mission cancelled! Landing...");
          endMission();
          setServerState(ServerState::LAND);
          break;
        case MISSION_E:
          logWarn("[MISSION] IGNORED EVENT. UAV already in [MISSION] mode");
          break;
        case HOVER_E:
          logWarn("[MISSION] Mission cancelled! Hovering...");
          endMission();
          setServerState(ServerState::HOVER);
          break;
        case E_STOP_E:
          logFatal("[MISSION] EMERGENCY STOP ACTIVATED!");
          setServerState(ServerState::E_STOP);
          break;
        case EMPTY_E:
          // Default case if no event sent
          break;
      }

      if (!isMissionComplete()){
        // logInfoThrottled("[MISSION] executing mission...", 5.0);
      }
      break;

    case ServerState::E_STOP:
      logFatalThrottled("[E_STOP] Currently in E STOP State, please reset the vehicle and trajectory server!", 1.0);
      break;

  }
}

void TrajServer::pubServerStateTimerCb(const ros::TimerEvent &e){
  trajectory_server_msgs::State state_msg;

  state_msg.drone_id = drone_id_;
  state_msg.traj_server_state = StateToString(getServerState());
  state_msg.planner_server_state = planner_state_;
  state_msg.uav_state = uav_current_state_.mode;
  state_msg.armed = uav_current_state_.armed;

  server_state_pub_.publish(state_msg);
}

void TrajServer::debugTimerCb(const ros::TimerEvent &e){
  nav_msgs::Path uav_path;
  uav_path.header.stamp = ros::Time::now();
  uav_path.header.frame_id = origin_frame_; 
  uav_path.poses = std::vector<geometry_msgs::PoseStamped>(uav_poses_.begin(), uav_poses_.end());

  uav_path_pub_.publish(uav_path);

  if (getServerState() == ServerState::MISSION){
    trajectory_server_msgs::TrackingError track_err;

    track_err.average_time_window = error_tracking_window_; 

    // Position
    track_err.err_x = fabs(uav_odom_.pose.pose.position.x - last_mission_pos_(0));  
    track_err.err_y = fabs(uav_odom_.pose.pose.position.y - last_mission_pos_(1)); 
    track_err.err_z = fabs(uav_odom_.pose.pose.position.z - last_mission_pos_(2)); 

    track_err.err_xy = sqrt(
      track_err.err_x * track_err.err_x + track_err.err_y * track_err.err_y); 
      
    // Velocity
    track_err.err_x_dot = fabs(uav_odom_.twist.twist.linear.x - last_mission_vel_(0));  
    track_err.err_y_dot = fabs(uav_odom_.twist.twist.linear.y - last_mission_vel_(1)); 
    track_err.err_z_dot = fabs(uav_odom_.twist.twist.linear.z - last_mission_vel_(2)); 

    // TODO calculation is wrong
    track_err.err_xy_dot = sqrt(
      track_err.err_x_dot * track_err.err_x_dot + track_err.err_y_dot * track_err.err_y_dot); 

    // Yaw and yaw rate
    tf2::Quaternion uav_quat_tf;
    tf2::convert(uav_odom_.pose.pose.orientation, uav_quat_tf);
    tf2::Matrix3x3 m(uav_quat_tf);
    double uav_roll, uav_pitch, uav_yaw;
    m.getRPY(uav_roll, uav_pitch, uav_yaw);

    track_err.err_yaw = fabs(uav_yaw - last_mission_yaw_); 
    track_err.err_yaw_dot = fabs(uav_odom_.twist.twist.angular.z - last_mission_yaw_dot_);  
    
    // Get average values

    err_xy_vec.push_back(std::make_pair(track_err.err_xy, ros::Time::now().toSec()));
    err_xy_dot_vec.push_back(std::make_pair(track_err.err_xy_dot , ros::Time::now().toSec()));
    err_yaw_vec.push_back(std::make_pair(track_err.err_yaw , ros::Time::now().toSec()));

    auto update_window = [window=error_tracking_window_] (std::deque<std::pair<double, double>>& v)
    {
      while ( !v.empty() 
        && (ros::Time::now().toSec() - v.front().second) > window)
      {
        v.pop_front();
      }
    };

    auto get_avg = [](std::deque<std::pair<double, double>>& v)
    {
      double sum = 0;
      for (auto& elem : v)
          sum += elem.first;
      return sum / v.size();
    };

    update_window(err_xy_vec);
    update_window(err_xy_dot_vec);
    update_window(err_yaw_vec);

    track_err.average_err_xy = get_avg(err_xy_vec);
    track_err.average_err_xy_dot = get_avg(err_xy_dot_vec);
    track_err.average_err_yaw = get_avg(err_yaw_vec);

    // Get max values

    auto get_max_val = [](std::deque<std::pair<double, double>>& v)
    {
      double max_val = -1.0;
      for (auto& elem : v){
        if (elem.first > max_val){
          max_val = elem.first;
        }
      }
      return max_val;
    };

    track_err.max_err_xy = get_max_val(err_xy_vec);
    track_err.max_err_xy_dot = get_max_val(err_xy_dot_vec);
    track_err.max_err_yaw = get_max_val(err_yaw_vec);

    tracking_error_pub_.publish(track_err);
  }

  // TODO Add marker visualization to visualize error between plan and actual position
}

/* Trajectory execution methods */

void TrajServer::execLand()
{
  Eigen::Vector3d pos;
  pos << uav_pose_.pose.position.x, uav_pose_.pose.position.y, landed_height_;
  uint16_t type_mask = 2552; // Ignore Velocity, Acceleration

  publishCmd(pos, Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(), last_mission_yaw_, 0, type_mask);
}

void TrajServer::execTakeOff()
{ 
  Eigen::Vector3d pos;
  pos << uav_pose_.pose.position.x, uav_pose_.pose.position.y, takeoff_height_;
  uint16_t type_mask = 2552; // Ignore Velocity, Acceleration

  publishCmd(pos, Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(), last_mission_yaw_, 0, type_mask);

}

void TrajServer::execHover()
{
  uint16_t type_mask = 2552; // Ignore Velocity, Acceleration
  Eigen::Vector3d pos;
  pos = last_mission_pos_;

  if (pos(2) < 0.1){
    pos(2) = takeoff_height_;
  }

  publishCmd(pos, Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(), last_mission_yaw_, 0, type_mask);
}

void TrajServer::execMission()
{
  if (traj_mode_ == TrajMode::POLYTRAJ){
    /* no publishing before receive traj_ and have heartbeat */
    if (heartbeat_time_.toSec() <= 1e-5)
    {
      logErrorThrottled("[traj_server] No heartbeat from the planner received", 1.0);
      return;
    }

    ros::Time time_now = ros::Time::now();
    // Time elapsed since start of trajectory
    double t_cur = (time_now - start_time_).toSec();

    Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), jer(Eigen::Vector3d::Zero());
    std::pair<double, double> yaw_yawdot(0, 0);

    static ros::Time time_last = ros::Time::now();
    // IF time elapsed is below duration of trajectory, then continue to send command
    if (t_cur >= 0.0 && t_cur < traj_duration_)
    {
      pos = traj_->getPos(t_cur);
      vel = traj_->getVel(t_cur);
      acc = traj_->getAcc(t_cur);
      jer = traj_->getJer(t_cur);

      /*** calculate yaw ***/
      yaw_yawdot = calculate_yaw(t_cur, pos, (time_now - time_last).toSec());

      time_last = time_now;
      last_mission_pos_ = pos;
      last_mission_vel_ = vel;

      uint16_t type_mask = 2048; // Ignore yaw rate

      if (!checkPositionLimits(position_limits_, pos)) {
        // If position safety limit check failed, switch to hovering mode
        setServerEvent(ServerEvent::HOVER_E);
      }

      publishCmd(pos, vel, acc, jer, yaw_yawdot.first, yaw_yawdot.second, type_mask);
    }
    // IF time elapsed is longer then duration of trajectory, then nothing is done
    else if (t_cur >= traj_duration_) // Finished trajectory
    {
      // logWarn("t_cur exceeds traj_duration! No commands to send");
      endMission();
    }
    else {
      logWarn(string_format("Invalid time, current time is negative at %d!",t_cur));
    }
    
  }
  else if (traj_mode_ == TrajMode::MULTIDOFJOINTTRAJECTORY){
    uint16_t type_mask = 2048; // Ignore yaw rate

    publishCmd(last_mission_pos_, last_mission_vel_, 
              last_mission_acc_, last_mission_jerk_, 
              last_mission_yaw_, last_mission_yaw_dot_, type_mask);
  }
}

void TrajServer::startMission(){
  if (getServerState() == ServerState::MISSION){
    mission_completed_ = false;
  }
}

void TrajServer::endMission()
{
  mission_completed_ = true;
}

/* Conditional checking methods */

bool TrajServer::isLanded()
{
  // Check that difference between desired landing height and current UAV position
  // is within tolerance 
  return abs(uav_pose_.pose.position.z - landed_height_) < take_off_landing_tol_;
}

bool TrajServer::isTakenOff()
{
  // Check that difference between desired landing height and current UAV position
  // is within tolerance 
  return abs(uav_pose_.pose.position.z - takeoff_height_) < take_off_landing_tol_;
}

bool TrajServer::isMissionComplete()
{
  return mission_completed_;
}

bool TrajServer::isPlannerHeartbeatTimeout(){
  return (ros::Time::now() - heartbeat_time_).toSec() > planner_heartbeat_timeout_;
}

/* Publisher methods */

void TrajServer::publishCmd(
  Vector3d p, Vector3d v, Vector3d a, Vector3d j, double yaw, double yaw_rate, uint16_t type_mask)
{
  mavros_msgs::PositionTarget pos_cmd;

  pos_cmd.header.stamp = ros::Time::now();
  pos_cmd.header.frame_id = origin_frame_;
  pos_cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
  pos_cmd.type_mask = type_mask;
  // pos_cmd.type_mask = 1024; // Ignore Yaw
  // pos_cmd.type_mask = 2048; // ignore yaw_rate
  // pos_cmd.type_mask = 2496; // Ignore Acceleration
  // pos_cmd.type_mask = 3520; // Ignore Acceleration and Yaw
  // pos_cmd.type_mask = 2552; // Ignore Acceleration, Velocity, 
  // pos_cmd.type_mask = 3576; // Ignore Acceleration, Velocity and Yaw

  pos_cmd.position.x = p(0);
  pos_cmd.position.y = p(1);
  pos_cmd.position.z = p(2);
  pos_cmd.velocity.x = v(0);
  pos_cmd.velocity.y = v(1);
  pos_cmd.velocity.z = v(2);
  pos_cmd.acceleration_or_force.x = a(0);
  pos_cmd.acceleration_or_force.y = a(1);
  pos_cmd.acceleration_or_force.z = a(2);
  pos_cmd.yaw = yaw;
  pos_cmd.yaw_rate = yaw_rate;
  pos_cmd_raw_pub_.publish(pos_cmd);
}

/* Helper methods */

bool TrajServer::toggle_offboard_mode(bool toggle)
  {
    bool arm_val = false;
    std::string set_mode_val = "AUTO.LOITER"; 
    if (toggle){
      arm_val = true;
      set_mode_val = "OFFBOARD"; 
    }

    auto conditions_fulfilled = [&] () {
      return (toggle ? is_uav_ready() : is_uav_idle());
    };

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = arm_val;

    mavros_msgs::SetMode set_mode_srv;
    set_mode_srv.request.custom_mode = set_mode_val;

    // Make sure takeoff is not immediately sent, 
    // this will help to stream the correct data to the program first.
    // Will give a 1sec buffer
    // ros::Duration(1.0).sleep();

    ros::Rate rate(pub_cmd_freq_);

    // send a few setpoints before starting
    for (int i = 0; ros::ok() && i < 10; i++)
    {
      execTakeOff();
      ros::spinOnce();
      rate.sleep();
    }
    ros::Time last_request_t = ros::Time::now();

    while (!conditions_fulfilled()){

      bool request_timeout = ((ros::Time::now() - last_request_t) > ros::Duration(2.0));

      if (uav_current_state_.mode != set_mode_val && request_timeout)
      {
        if (set_mode_client.call(set_mode_srv))
        {
          if (set_mode_srv.response.mode_sent){
            logInfo(string_format("Setting %s mode successful", set_mode_val.c_str()));
          }
          else {
            logInfo(string_format("Setting %s mode failed", set_mode_val.c_str()));
          }
        }
        else {
          logInfo("Service call to PX4 set_mode_client failed");
        }

        last_request_t = ros::Time::now();
      }
      else if (uav_current_state_.armed != arm_val && request_timeout) 
      {
        if (arming_client.call(arm_cmd)){
          if (arm_cmd.response.success){
            logInfo(string_format("Setting arm to %d successful", arm_val));
          }
          else {
            logInfo(string_format("Setting arm to %d failed", arm_val));
          }
        }
        else {
          logInfo("Service call to PX4 arming_client failed");
        }

        last_request_t = ros::Time::now();
      }
      ros::spinOnce();
      rate.sleep();        
    }

    return true;
  }

std::pair<double, double> TrajServer::calculate_yaw(double t_cur, Eigen::Vector3d &pos, double dt)
{
  constexpr double YAW_DOT_MAX_PER_SEC = 2 * M_PI;
  constexpr double YAW_DOT_DOT_MAX_PER_SEC = 5 * M_PI;
  std::pair<double, double> yaw_yawdot(0, 0);

  Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_
                            ? traj_->getPos(t_cur + time_forward_) - pos
                            : traj_->getPos(traj_duration_) - pos;
  double yaw_temp = dir.norm() > 0.1
                        ? atan2(dir(1), dir(0))
                        : last_mission_yaw_;

  double yawdot = 0;
  double d_yaw = yaw_temp - last_mission_yaw_;
  if (d_yaw >= M_PI)
  {
    d_yaw -= 2 * M_PI;
  }
  if (d_yaw <= -M_PI)
  {
    d_yaw += 2 * M_PI;
  }

  const double YDM = d_yaw >= 0 ? YAW_DOT_MAX_PER_SEC : -YAW_DOT_MAX_PER_SEC;
  const double YDDM = d_yaw >= 0 ? YAW_DOT_DOT_MAX_PER_SEC : -YAW_DOT_DOT_MAX_PER_SEC;
  double d_yaw_max;
  if (fabs(last_mission_yaw_dot_ + dt * YDDM) <= fabs(YDM))
  {
    // yawdot = last_mission_yaw_dot_ + dt * YDDM;
    d_yaw_max = last_mission_yaw_dot_ * dt + 0.5 * YDDM * dt * dt;
  }
  else
  {
    // yawdot = YDM;
    double t1 = (YDM - last_mission_yaw_dot_) / YDDM;
    d_yaw_max = ((dt - t1) + dt) * (YDM - last_mission_yaw_dot_) / 2.0;
  }

  if (fabs(d_yaw) > fabs(d_yaw_max))
  {
    d_yaw = d_yaw_max;
  }
  yawdot = d_yaw / dt;

  double yaw = last_mission_yaw_ + d_yaw;
  if (yaw > M_PI)
    yaw -= 2 * M_PI;
  if (yaw < -M_PI)
    yaw += 2 * M_PI;
  yaw_yawdot.first = yaw;
  yaw_yawdot.second = yawdot;

  last_mission_yaw_ = yaw_yawdot.first;
  last_mission_yaw_dot_ = yaw_yawdot.second;

  yaw_yawdot.second = yaw_temp;

  return yaw_yawdot;
}

bool TrajServer::checkPositionLimits(SafetyLimits position_limits, Vector3d p){
  if (p(0) < position_limits.min_x || p(0) > position_limits.max_x){
    logError(string_format("Commanded x position (%f) exceeded x limits (%f-%f)", 
      p(0), position_limits.min_x, position_limits.max_x));

    return false;
  }
  else if (p(1) < position_limits.min_y || p(1) > position_limits.max_y) {

    logError(string_format("Commanded y position (%f) exceeded y limits (%f-%f)", 
      p(1), position_limits.min_y, position_limits.max_y));

    return false;
  }
  else if (p(2) < position_limits.min_z || p(2) > position_limits.max_z) {

    logError(string_format("Commanded z position (%f) exceeded z limits (%f-%f)", 
      p(2), position_limits.min_z, position_limits.max_z));

    return false;
  }

  return true;
}

void TrajServer::geomMsgsVector3ToEigenVector3(const geometry_msgs::Vector3& geom_vect, Eigen::Vector3d& eigen_vect){
  eigen_vect(0) = geom_vect.x;
  eigen_vect(1) = geom_vect.x;
  eigen_vect(2) = geom_vect.x;
}

double TrajServer::quaternionToYaw(const geometry_msgs::Quaternion& quat){
  tf2::Quaternion uav_quat_tf;
  tf2::convert(quat, uav_quat_tf);
  tf2::Matrix3x3 m(uav_quat_tf);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  return yaw;
}

/* FSM Methods */

void TrajServer::setServerState(ServerState des_state)
{
  logInfo(string_format("Transitioning server state: %s -> %s", 
    StateToString(getServerState()).c_str(), StateToString(des_state).c_str()));

  server_state_ = des_state;
}

ServerState TrajServer::getServerState()
{
  return server_state_;
}

void TrajServer::setServerEvent(ServerEvent event)
{
  // logInfo(string_format("Set server event: %s", EventToString(event).c_str()));

  server_event_ = event;
}

ServerEvent TrajServer::getServerEvent()
{
  // logInfo(string_format("Retrieved server event: %s", EventToString(server_event_).c_str()));
  ServerEvent event = server_event_;
  server_event_ = ServerEvent::EMPTY_E;  // Reset to empty

  return event;
}
