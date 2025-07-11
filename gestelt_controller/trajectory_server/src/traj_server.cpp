#include <traj_server/traj_server.h>

using namespace Eigen;

/* Initialization methods */

void TrajectoryServer::init(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
  
  /////////////////
  /* ROS Params*/
  /////////////////
  pnh.param("drone_id", drone_id_, 0);
  pnh.param("origin_frame", origin_frame_, std::string("world"));

  // Operational params
  pnh.param("takeoff_height", takeoff_height_, 1.0);
  pnh.param("minimum_hover_height", min_hover_height_, 0.25);

  //mission params
  pnh.param("mission_command_mode", cmd_mode_num, 1);

  // Safety bounding box params
  pnh.param("enable_safety_box", enable_safety_box_, true);
  pnh.param("safety_box/max_x", safety_box_.max_x, -1.0);
  pnh.param("safety_box/min_x", safety_box_.min_x, -1.0);
  pnh.param("safety_box/max_y", safety_box_.max_y, -1.0);
  pnh.param("safety_box/min_y", safety_box_.min_y, -1.0);
  pnh.param("safety_box/max_z", safety_box_.max_z, -1.0);
  pnh.param("safety_box/min_z", safety_box_.min_z, -1.0);

  // Frequency params
  pnh.param("pub_cmd_freq", pub_cmd_freq_, 25.0); // frequency to publish commands
  double state_machine_tick_freq; // Frequency to tick the state machine transitions
  pnh.param("state_machine_tick_freq", state_machine_tick_freq, 50.0);
  double debug_freq; // Frequency to publish debug information
  pnh.param("debug_freq", debug_freq, 10.0);
  pnh.param("warp_jax", warp_jax, 0.0);

  //Set mission_command_mode
  setMissionCmd(MissionCmdMode(IntToMission(int(1))));
  std::cout <<"I AM IN THE NEW SCRIPTTTTTTTTTTTTTTTTTTTTTTTTTT\n";
  last_odom_time = ros::Time(0);
  last_pose_time = ros::Time(0);
  initial_map_to_warp = 0;

  /////////////////
  /* Subscribers */
  /////////////////
  // Subscription to commands
  command_server_sub_ = nh.subscribe<gestelt_msgs::Command>("traj_server/command", 5, &TrajectoryServer::serverCommandCb, this);
  swarm_command_server_sub_ = nh.subscribe<std_msgs::Int8>("/traj_server/swarm_command", 5, &TrajectoryServer::swarmServerCommandCb, this);
  mission_command_server_sub_ = nh.subscribe<std_msgs::Int8>("/traj_server/mission_command", 5, &TrajectoryServer::missionServerCommandCb, this);

  // Subscription to planner adaptor
  exec_traj_sub_ = nh.subscribe<gestelt_msgs::ExecTrajectory>("planner_adaptor/exec_trajectory", 5, &TrajectoryServer::execTrajCb, this);
  // exec_lowlvl_cmd_sub_ = nh.subscribe<gestelt_msgs::ExecTrajectory>("planner_adaptor/exec_low_level_cmd", 5, &TrajectoryServer::execLowLvlCmdCb, this);

  // Subscription to UAV (via MavROS)
  uav_state_sub_ = nh.subscribe<mavros_msgs::State>("mavros/state", 5, &TrajectoryServer::UAVStateCb, this);
  pose_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 5, &TrajectoryServer::UAVPoseCB, this);
  odom_sub_ = nh.subscribe<nav_msgs::Odometry>("mavros/local_position/odom", 5, &TrajectoryServer::UAVOdomCB, this);
  geom_ctrl_sub_ = nh.subscribe<mavros_msgs::AttitudeTarget>("geom_ctrl", 5, &TrajectoryServer::geomCb, this);

  /////////////////
  /* Publishers */
  /////////////////
  pos_cmd_raw_pub_ = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 50);
  vel_cmd_raw_pub_ = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 50);
  server_state_pub_ = nh.advertise<gestelt_msgs::CommanderState>("traj_server/state", 50);
  vel_magnitude_pub_ = nh.advertise<std_msgs::Float32>("vel_magnitude", 50);
  low_lvl_cmd_raw_pub_ = nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 1);
  angular_rates_pub_ = nh.advertise<nav_msgs::Odometry>("warp/local_position/odom", 1);
  warp_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("warp/local_position/pose", 1);

  ////////////////////
  /* Service clients */
  ////////////////////
  arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

  /////////////////
  /* Timer callbacks */
  /////////////////
  exec_traj_timer_ = nh.createTimer(ros::Duration(1/pub_cmd_freq_), &TrajectoryServer::execTrajTimerCb, this);
  tick_state_timer_ = nh.createTimer(ros::Duration(1/state_machine_tick_freq), &TrajectoryServer::tickServerStateTimerCb, this);
  debug_timer_ = nh.createTimer(ros::Duration(1/debug_freq), &TrajectoryServer::debugTimerCb, this);

  // Initialize ignore flags for mavros position target command
  IGNORE_POS = mavros_msgs::PositionTarget::IGNORE_PX | mavros_msgs::PositionTarget::IGNORE_PY | mavros_msgs::PositionTarget::IGNORE_PZ;
  IGNORE_VEL = mavros_msgs::PositionTarget::IGNORE_VX | mavros_msgs::PositionTarget::IGNORE_VY | mavros_msgs::PositionTarget::IGNORE_VZ;
  IGNORE_ACC = mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ;
  ATTITUDE_CTRL = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE | mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE | mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
  USE_FORCE = mavros_msgs::PositionTarget::FORCE;
  IGNORE_YAW = mavros_msgs::PositionTarget::IGNORE_YAW;
  IGNORE_YAW_RATE = mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
}

/* Subscriber Callbacks */

void TrajectoryServer::execTrajCb(const gestelt_msgs::ExecTrajectory::ConstPtr &msg)
{
  if (getServerState() != ServerState::MISSION){ 
    logErrorThrottled("Executing Joint Trajectory while not in MISSION mode. Ignoring!", 1.0);
    return;
  }

  last_traj_msg_time_ = ros::Time::now();

  std::lock_guard<std::mutex> cmd_guard(cmd_mutex_);

  // std::string frame_id = msg->header.frame_id;
  
  mission_type_mask_ = msg->type_mask; 

  if (getMissionCmd() == MissionCmdMode::PVA){
  geomMsgsVector3ToEigenVector3(msg->transform.translation, last_mission_pos_);
  last_mission_yaw_ = quaternionToRPY(msg->transform.rotation)(2); // yaw

  // ROS_INFO("Last mission yaw: %f", last_mission_yaw_);

  geomMsgsVector3ToEigenVector3(msg->velocity.linear, last_mission_vel_);
  last_mission_yaw_dot_ = msg->velocity.angular.z; //yaw rate
  // ROS_INFO("received velocity: %f, %f, %f", last_mission_vel_(0), last_mission_vel_(1), last_mission_vel_(2));

  geomMsgsVector3ToEigenVector3(msg->acceleration.linear, last_mission_acc_);
  // ROS_INFO("received acceleration: %f, %f, %f", last_mission_acc_(0), last_mission_acc_(1), last_mission_acc_(2));
  }

  if (getMissionCmd() == MissionCmdMode::VEL){
  // ROS_INFO("Last mission yaw: %f", last_mission_yaw_);

  geomMsgsVector3ToEigenVector3(msg->velocity.linear, last_mission_vel_);
  last_mission_yaw_dot_ = msg->velocity.angular.z; //yaw rate
  // ROS_INFO("received velocity: %f, %f, %f", last_mission_vel_(0), last_mission_vel_(1), last_mission_vel_(2));
  }

  if (getMissionCmd() == MissionCmdMode::ATTITUDE){
    last_mission_thrust_vector_ = msg->throttle;
    geomMsgsVector3ToEigenVector3(msg->angular_rates.angular, last_mission_warp_body_rates_);
    geomMsgsVector4ToEigenVector4(msg->transform.rotation, last_mission_quaternion_);

    ct_omega_mode_ = msg->type_mask; 

    if (ct_omega_mode_ == 1) //means it is in the body rates mode. Need to transform the body rates mode from warp back to map frame
    {
        // If warp_jax == 0, this means we are using policy trained in warp. So we have to change the output from warp global frame to body nwu frame
        // if warp_jax == 1, this means we are using policy trained in jax. So it is already in body nwu frame.
        if (warp_jax == 0.0){
        
        geometry_msgs::Vector3Stamped output_bodyrates_vector;
        output_bodyrates_vector.vector.x = last_mission_warp_body_rates_(0);
        output_bodyrates_vector.vector.y = last_mission_warp_body_rates_(1);
        output_bodyrates_vector.vector.z = last_mission_warp_body_rates_(2);

        try {
            // Lookup the transformation from input frame to target frame
            geometry_msgs::TransformStamped transformStamped;
            transformStamped = tfBuffer.lookupTransform("body", "warp", ros::Time(0));

            // Transform the vector
            geometry_msgs::Vector3Stamped transformed_output_bodyrates_vector;
            tf2::doTransform(output_bodyrates_vector, transformed_output_bodyrates_vector, transformStamped);
            // ROS_INFO("Transformed Vector: x=%.2f, y=%.2f, z=%.2f", 
            //          transformed_vector.vector.x, transformed_vector.vector.y, transformed_vector.vector.z);
            
            last_mission_body_rates_(0) = transformed_output_bodyrates_vector.vector.x;
            last_mission_body_rates_(1) = transformed_output_bodyrates_vector.vector.y;
            last_mission_body_rates_(2) = transformed_output_bodyrates_vector.vector.z;

            // std::cout << "This is x: " << last_mission_body_rates_(0) << "\n";
            // std::cout << "This is y: " << last_mission_body_rates_(1) << "\n";
            // std::cout << "This is z: " << last_mission_body_rates_(2) << "\n";
           } 
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("Could not transform vector: %s", ex.what());
        }
        }
      else if (warp_jax == 1.0){
        geomMsgsVector3ToEigenVector3(msg->angular_rates.angular, last_mission_body_rates_);
      }
    }

  }

}

void TrajectoryServer::UAVStateCb(const mavros_msgs::State::ConstPtr &msg)
{
  // logInfoThrottled(str_fmt("State: Mode[%s], Connected[%d], Armed[%d]", msg->mode.c_str(), msg->connected, msg->armed), 1.0);
  uav_current_state_ = *msg;
}

void TrajectoryServer::UAVPoseCB(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  if (first_pose_){
    last_mission_pos_(0) = msg->pose.position.x;
    last_mission_pos_(1) = msg->pose.position.y;
    num_pose_msgs_++;
    if (num_pose_msgs_ > 100){
      first_pose_ = false;
      logInfo(str_fmt("Taking off 2d pose locked to (%f, %f)", last_mission_pos_(0), last_mission_pos_(1)));
    }
  }

  uav_pose_ = *msg; 

  ros::Time stamp = msg->header.stamp;
  ros::Duration latency = stamp - last_pose_time;
  // if (latency.toSec() > 0.02)  // 0.1 seconds = 100 ms
  // {
  //   ROS_WARN("Pose timestamp is delayed by %.3f ms!", latency.toSec() * 1000.0);
  // }


  last_pose_time = stamp;

  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "map";
  transformStamped.child_frame_id = "body" + drone_id_;
  transformStamped.transform.translation.x = msg->pose.position.x;
  transformStamped.transform.translation.y = msg->pose.position.y;
  transformStamped.transform.translation.z = msg->pose.position.z;
  transformStamped.transform.rotation.x = msg->pose.orientation.x;
  transformStamped.transform.rotation.y = msg->pose.orientation.y;
  transformStamped.transform.rotation.z = msg->pose.orientation.z;
  transformStamped.transform.rotation.w = msg->pose.orientation.w;

  br.sendTransform(transformStamped);   

}

void TrajectoryServer::UAVOdomCB(const nav_msgs::Odometry::ConstPtr &msg)
{
  uav_odom_ = *msg;
  ros::Time stamp = msg->header.stamp;
  ros::Duration latency = stamp - last_odom_time;
  // if (latency.toSec() > 0.01)  // 0.1 seconds = 100 ms
  // {
  //   ROS_WARN("Odometry timestamp is delayed by %.3f ms!", latency.toSec() * 1000.0);
  // }


  last_odom_time = stamp;
  Eigen::Vector3d vel_vect = Eigen::Vector3d{
                                msg->twist.twist.linear.x, 
                                msg->twist.twist.linear.y, 
                                msg->twist.twist.linear.z};
  

  std_msgs::Float32 vel_mag_msg;
  vel_mag_msg.data = vel_vect.norm();

  vel_magnitude_pub_.publish(vel_mag_msg);

  geometry_msgs::Vector3Stamped input_vector;
  input_vector.vector.x = msg->twist.twist.angular.x;
  input_vector.vector.y = msg->twist.twist.angular.y;
  input_vector.vector.z = msg->twist.twist.angular.z;

  geometry_msgs::Vector3Stamped input_linearvel_vector;
  input_linearvel_vector.vector.x = msg->twist.twist.linear.x;
  input_linearvel_vector.vector.y = msg->twist.twist.linear.y;
  input_linearvel_vector.vector.z = msg->twist.twist.linear.z;

  if (initial_map_to_warp == 0){
      try{
        transformStamped_map_to_warp = tfBuffer.lookupTransform("warp", "map", ros::Time(0));
        map2warp_transform_quat(0) = transformStamped_map_to_warp.transform.rotation.x;
        map2warp_transform_quat(1) = transformStamped_map_to_warp.transform.rotation.y;
        map2warp_transform_quat(2) = transformStamped_map_to_warp.transform.rotation.z;
        map2warp_transform_quat(3) = transformStamped_map_to_warp.transform.rotation.w;
        initial_map_to_warp = 1;
      }
      catch (tf2::TransformException &ex) {
      ROS_WARN("Could not transform vector: %s", ex.what());
      }
      }

  try {
      // Lookup the transformation from input frame to target frame
      geometry_msgs::TransformStamped transformStamped;
      // Note that if warp_jax = 0, we are assuming that we are transforming directly to warp frame. If we use jax policy, then we transform to global map nwu frame. This doesnt affect
      // the pose because we will just take the pose from map directly if we using jax.
      if (warp_jax == 0.0){
        transformStamped = tfBuffer.lookupTransform("warp", "body", ros::Time(0));
      }
      else if (warp_jax==1.0){
        transformStamped = tfBuffer.lookupTransform("map", "body", ros::Time(0));
      }
      

      // Transform the vector
      geometry_msgs::Vector3Stamped transformed_vector;
      geometry_msgs::Vector3Stamped transformed_linearvel_vector;
      tf2::doTransform(input_vector, transformed_vector, transformStamped);
      tf2::doTransform(input_linearvel_vector, transformed_linearvel_vector, transformStamped);
      // ROS_INFO("Transformed Vector: x=%.2f, y=%.2f, z=%.2f", 
      //          transformed_vector.vector.x, transformed_vector.vector.y, transformed_vector.vector.z);
      nav_msgs::Odometry transformed_odom;
      transformed_odom.header.stamp = ros::Time::now();
      transformed_odom.twist.twist.angular.x = transformed_vector.vector.x; 
      transformed_odom.twist.twist.angular.y = transformed_vector.vector.y; 
      transformed_odom.twist.twist.angular.z = transformed_vector.vector.z; 
      transformed_odom.twist.twist.linear.x = transformed_linearvel_vector.vector.x;
      transformed_odom.twist.twist.linear.y = transformed_linearvel_vector.vector.y;
      transformed_odom.twist.twist.linear.z = transformed_linearvel_vector.vector.z;
      angular_rates_pub_.publish(transformed_odom);

      Eigen::Vector4d map_frame_quat(uav_pose_.pose.orientation.x, uav_pose_.pose.orientation.y, uav_pose_.pose.orientation.z, uav_pose_.pose.orientation.w);
      // Eigen::Vector4d map_frame_quat(0.0,0.707,0.0,0.707);
      Eigen::Vector4d final_quat;
      quaternion_multiplication(map_frame_quat, map2warp_transform_quat, final_quat);
      // std::cout << "Matrix values for final_quat:\n" << final_quat << std::endl;
      geometry_msgs::PoseStamped warp_pose;
      // std::cout << "printing x value: " << final_quat(0);
      // std::cout << "printing y value: " << final_quat(1);
      // std::cout << "printing z value: " << final_quat(2);
      // std::cout << "printing w value: " << final_quat(3);
      geometry_msgs::TransformStamped transformStamped_wb;
      transformStamped_wb = tfBuffer.lookupTransform("warp", "body", ros::Time(0));
      warp_pose.header.frame_id = "warp";
      warp_pose.header.stamp = ros::Time::now();
      warp_pose.pose.position.x = transformStamped_wb.transform.translation.x;
      warp_pose.pose.position.y = transformStamped_wb.transform.translation.y;
      warp_pose.pose.position.z = transformStamped_wb.transform.translation.z;
      warp_pose.pose.orientation.x = final_quat(0);
      warp_pose.pose.orientation.y = final_quat(1);
      warp_pose.pose.orientation.z = final_quat(2);
      warp_pose.pose.orientation.w = final_quat(3);

      warp_pose_pub_.publish(warp_pose);




  } 
  catch (tf2::TransformException &ex) {
      ROS_WARN("Could not transform vector: %s", ex.what());
  }


}

void TrajectoryServer::geomCb(const mavros_msgs::AttitudeTarget::ConstPtr & msg)
{
  // std::cout << "I am in here now\n";
  geom_body_rate(0) = msg->body_rate.x;
  geom_body_rate(1) = msg->body_rate.y;
  geom_body_rate(2) = msg->body_rate.z;
  geom_thrust = msg-> thrust;
  last_traj_msg_time_ = ros::Time::now();
}

void TrajectoryServer::swarmServerCommandCb(const std_msgs::Int8::ConstPtr & msg)
{
  if (msg->data < 0 || msg->data > ServerEvent::EMPTY_E){
    logError("Invalid server command, ignoring...");
  }

  setServerEvent(ServerEvent(msg->data));
}

void TrajectoryServer::serverCommandCb(const gestelt_msgs::Command::ConstPtr & msg)
{
  if (msg->command < 0 || msg->command > ServerEvent::EMPTY_E){
    logError("Invalid server command, ignoring...");
  }

  setServerEvent(ServerEvent(msg->command));
}

void TrajectoryServer::missionServerCommandCb(const std_msgs::Int8::ConstPtr & msg)
{
  if (msg->data < 0){
    logError("Invalid server command, ignoring...");
  }
  std::cout<< "Me here....\n";
  std::cout << "current mission command is here" << getMissionCmd() << "\n";
  setMissionCmd(MissionCmdMode(IntToMission(msg->data)));
  // std::cout << "Mission Mode Changed to" << cmd_mode_num;
  // std::cout << "Mission Mode Changed to" << cmd_mode_num;
  
}

/* Timer Callbacks */

void TrajectoryServer::execTrajTimerCb(const ros::TimerEvent &e)
{
  // has received vel value
  // ROS_INFO("execTrajTimerCb received velocity: %f, %f, %f", last_mission_vel_(0), last_mission_vel_(1), last_mission_vel_(2));

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
      if (!isExecutingMission()){ // isExecutingMission() is true if last exec trajectory message did not exceed timeout
        logInfoThrottled("No Mission Received, waiting...", 5.0);
        execHover();
      }
      else {
        execMission();
      }
      break;

    case ServerState::E_STOP:
      // Drone should stop all motors immediately
      execLand();
      break;
  }
}

void TrajectoryServer::tickServerStateTimerCb(const ros::TimerEvent &e)
{
  // logInfoThrottled(str_fmt("Current Server State: [%s]", StateToString(getServerState()).c_str()), 1.0);

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
          logInfoThrottled("[INIT] Initializing Trajectory Server, waiting for connection to FCU...", 2.0 );
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
          logWarn("[IDLE] IGNORED COMMAND. UAV has not taken off, unable to LAND");
          break;
        case MISSION_E:
          logWarn("[IDLE] IGNORED COMMAND. Please TAKEOFF first before setting MISSION mode");
          break;
        case HOVER_E:
          logWarn("[IDLE] IGNORED COMMAND. No mission to cancel");
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
          // logWarn("[TAKEOFF] IGNORED COMMAND. UAV already attempting taking off");
          break;
        case LAND_E:
          logInfo("[TAKEOFF] Attempting landing");
          setServerState(ServerState::LAND);
          break;
        case MISSION_E:
          logWarn("[TAKEOFF] IGNORED COMMAND to MISSION. Wait until UAV needs to take off before accepting mission command");
          break;
        case HOVER_E:
          logWarn("[TAKEOFF] IGNORED COMMAND to HOVER. Currently TAKING OFF");
          break;
        case E_STOP_E:
          logFatal("[TAKEOFF] EMERGENCY STOP ACTIVATED!");
          setServerState(ServerState::E_STOP);
          break;
        case EMPTY_E:
          // Default case if no event sent
          break;
      }

      if (!isUAVReady()){
        logInfo("[TAKEOFF] Calling toggle offboard mode");
        toggleOffboardMode(true);
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
          logInfo("[LAND] UAV taking off");
          setServerState(ServerState::TAKEOFF);
          break;
        case LAND_E:
          logWarn("[LAND] IGNORED COMMAND to LAND. UAV already attempting landing");
          break;
        case MISSION_E:
          logWarn("[LAND] IGNORED COMMAND to MISISON. UAV is landing, it needs to take off before accepting mission command");
          break;
        case HOVER_E:
          logWarn("[LAND] IGNORED COMMAND to HOVER. UAV needs to TAKE OFF before it can HOVER.");
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
          logWarn("[HOVER] IGNORED COMMAND to TAKE OFF. UAV already took off. Currently in [HOVER] mode");
          break;
        case LAND_E:
          logInfo("[HOVER] UAV is LANDING");
          setServerState(ServerState::LAND);
          break;
        case MISSION_E:
          logInfo("[HOVER] UAV entering [MISSION] mode.");
          setServerState(ServerState::MISSION);
          break;
        case HOVER_E:
          logWarn("[HOVER] IGNORED COMMAND to HOVER. Already hovering...");
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
          logWarn("[MISSION] IGNORED COMMAND. UAV already took off. Currently in [MISSION] mode");
          break;
        case LAND_E:
          logWarn("[MISSION] Mission cancelled! Landing...");
          setServerState(ServerState::LAND);
          break;
        case MISSION_E:
          logWarn("[MISSION] IGNORED COMMAND. UAV already in [MISSION] mode");
          break;
        case HOVER_E:
          logWarn("[MISSION] Mission cancelled! Hovering...");
          if (mission_hover_set_state == false){
            last_mission_pos_[0] = uav_pose_.pose.position.x;
            last_mission_pos_[1] = uav_pose_.pose.position.y;
            last_mission_pos_[2] = uav_pose_.pose.position.z;
            mission_hover_set_state = true;
          }
          
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

      break;

    case ServerState::E_STOP:
      logFatalThrottled("[E_STOP] Currently in E STOP State, please reset the vehicle and trajectory server!", 1.0);
      break;
  }
}

void TrajectoryServer::debugTimerCb(const ros::TimerEvent &e){
  // Publish current Commander state
  gestelt_msgs::CommanderState state_msg;

  state_msg.drone_id = drone_id_;
  state_msg.traj_server_state = StateToString(getServerState());
  state_msg.planner_server_state = "UNIMPLEMENTED";
  state_msg.uav_state = uav_current_state_.mode;
  state_msg.armed = uav_current_state_.armed;

  server_state_pub_.publish(state_msg);

}

/* Trajectory execution methods */

void TrajectoryServer::execLand()
{
  int type_mask = IGNORE_VEL | IGNORE_ACC | IGNORE_YAW_RATE ; // Ignore Velocity, Acceleration and yaw rate

  Eigen::Vector3d pos = Eigen::Vector3d{
    uav_pose_.pose.position.x, uav_pose_.pose.position.y, landed_height_};

  publishCmd( pos, Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(), 
              last_mission_yaw_, 0, 
              type_mask);
}

void TrajectoryServer::execTakeOff()
{ 
  int type_mask = IGNORE_VEL | IGNORE_ACC | IGNORE_YAW_RATE ; // Ignore Velocity, Acceleration and yaw rate
  
  Eigen::Vector3d pos = last_mission_pos_;
  last_mission_pos_(2) = takeoff_height_;
  pos(2) = takeoff_height_;

  logInfo(str_fmt("[TAKEOFF] Taking off to position (%f, %f, %f)", pos(0), pos(1), pos(2)));

  publishCmd( pos, Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(), 
              last_mission_yaw_, 0, 
              type_mask);
}

void TrajectoryServer::execHover()
{
  int type_mask = IGNORE_VEL | IGNORE_ACC | IGNORE_YAW_RATE ; // Ignore Velocity, Acceleration and yaw rate
  Eigen::Vector3d pos = Eigen::Vector3d{
    uav_pose_.pose.position.x, uav_pose_.pose.position.y, uav_pose_.pose.position.z};

  // ensure that hover z position does not fall below 0.2m
  pos(2) = pos(2) < min_hover_height_ ? min_hover_height_ : pos(2);
  publishCmd( last_mission_pos_, Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(), 
              last_mission_yaw_, 0, 
              type_mask);

  if (getMissionCmd() != MissionCmdMode::PVA){
    std::cout << "Setting....\n";
    setMissionCmd(MissionCmdMode(IntToMission(int(1))));
  }
  
}

void TrajectoryServer::execMission()
{
  std::lock_guard<std::mutex> cmd_guard(cmd_mutex_);

  if (getMissionCmd() == MissionCmdMode::PVA){
  publishCmd( last_mission_pos_, last_mission_vel_, last_mission_acc_, last_mission_jerk_, 
              last_mission_yaw_, last_mission_yaw_dot_, 
              mission_type_mask_);
  }
  else if(getMissionCmd() == MissionCmdMode::ATTITUDE){
  publishLowLvlCmd( last_mission_body_rates_, last_mission_thrust_vector_, last_mission_quaternion_, last_mission_pos_, ct_omega_mode_);
  }
  else if(getMissionCmd() == MissionCmdMode::VEL){
  publishVelCmd( last_mission_vel_, last_mission_pos_, ct_omega_mode_);
  }
  else if (getMissionCmd() == MissionCmdMode::GEOM){
    std::cout << "IN HERE\n";
    publishGeomCmd( geom_body_rate, geom_thrust);
  }
}

/* Publisher methods */

void TrajectoryServer::publishCmd(
  Vector3d p, Vector3d v, Vector3d a, Vector3d j, double yaw, double yaw_rate, uint16_t type_mask)
{
  if (enable_safety_box_ && !checkPositionLimits(safety_box_, p)) {
    // If position safety limit check failed, switch to hovering mode
    setServerEvent(ServerEvent::HOVER_E);
  }

  mavros_msgs::PositionTarget pos_cmd;

  pos_cmd.header.stamp = ros::Time::now();
  pos_cmd.header.frame_id = origin_frame_;
  pos_cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
  pos_cmd.type_mask = 2048;

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
  // ROS_INFO("Velocity for final command: %f, %f, %f", v(0), v(1), v(2));
  // ROS_INFO("Acceleration for final command: %f, %f, %f", a(0), a(1), a(2));
  pos_cmd_raw_pub_.publish(pos_cmd);
}

void TrajectoryServer::publishVelCmd(
  Vector3d v, Vector3d p, uint16_t ct_omega_mode_)
{
  if (enable_safety_box_ && !checkPositionLimits(safety_box_, p)) {
    // If position safety limit check failed, switch to hovering mode
    setServerEvent(ServerEvent::HOVER_E);
  }
  geometry_msgs::TwistStamped vel_cmd;
  vel_cmd.header.stamp = ros::Time::now();
  vel_cmd.header.frame_id = origin_frame_;
  vel_cmd.twist.linear.x = v(0);
  vel_cmd.twist.linear.y = v(1);
  vel_cmd.twist.linear.z = v(2);

  vel_cmd_raw_pub_.publish(vel_cmd);

}

void TrajectoryServer::publishGeomCmd(
  Vector3d geom_bodyrate, double geom_thrust){

    mavros_msgs::AttitudeTarget low_lvl_cmd;
    low_lvl_cmd.header.stamp = ros::Time::now();
    low_lvl_cmd.header.frame_id = origin_frame_;
    low_lvl_cmd.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE; // Ignore orientation
    double collective_thrust = geom_thrust;
    low_lvl_cmd.thrust = collective_thrust; ///(single_motor_max_thrust_*4);
    low_lvl_cmd.body_rate.x = geom_bodyrate[0];
    low_lvl_cmd.body_rate.y = geom_bodyrate[1];
    low_lvl_cmd.body_rate.z = geom_bodyrate[2];

    low_lvl_cmd_raw_pub_.publish(low_lvl_cmd);

  }

void TrajectoryServer::publishLowLvlCmd(
  Vector3d omega, double collective_thrust_vector, Vector4d quaternion, Vector3d p, uint16_t ct_omega_mode_)
{
  if (enable_safety_box_ && !checkPositionLimits(safety_box_, p)) {
    // If position safety limit check failed, switch to hovering mode
    setServerEvent(ServerEvent::HOVER_E);
  }
  mavros_msgs::AttitudeTarget low_lvl_cmd;
  low_lvl_cmd.header.stamp = ros::Time::now();
  low_lvl_cmd.header.frame_id = origin_frame_;
  if (ct_omega_mode_ == 0){
    low_lvl_cmd.type_mask = ATTITUDE_CTRL;
    double collective_thrust = collective_thrust_vector;
    low_lvl_cmd.thrust = collective_thrust; //(single_motor_max_thrust_*4);
    low_lvl_cmd.orientation.x = quaternion[0];
    low_lvl_cmd.orientation.y = quaternion[1];
    low_lvl_cmd.orientation.z = quaternion[2];
    low_lvl_cmd.orientation.w = quaternion[3];
    //  std::cout << "Matrix values for R1:\n" << quaternion << std::endl;
  }
  else if (ct_omega_mode_ == 1){
    low_lvl_cmd.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE; // Ignore orientation
    double collective_thrust = collective_thrust_vector;
    low_lvl_cmd.thrust = collective_thrust; ///(single_motor_max_thrust_*4);
    low_lvl_cmd.body_rate.x = omega[0];
    low_lvl_cmd.body_rate.y = omega[1];
    low_lvl_cmd.body_rate.z = omega[2];
  }
  
  low_lvl_cmd_raw_pub_.publish(low_lvl_cmd);

}


/* Helper methods */

bool TrajectoryServer::toggleOffboardMode(bool toggle)
  {
    bool arm_val = false;
    std::string set_mode_val = "AUTO.LOITER"; 
    if (toggle){
      arm_val = true;
      set_mode_val = "OFFBOARD"; 
    }

    auto conditions_fulfilled = [&] () {
      return (toggle ? isUAVReady() : isUAVIdle());
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
            logInfo(str_fmt("Setting %s mode successful", set_mode_val.c_str()));
          }
          else {
            logInfo(str_fmt("Setting %s mode failed", set_mode_val.c_str()));
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
            logInfo(str_fmt("Setting arm to %d successful", arm_val));
          }
          else {
            logInfo(str_fmt("Setting arm to %d failed", arm_val));
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

bool TrajectoryServer::checkPositionLimits(SafetyLimits position_limits, Vector3d p){

  if (p(0) < position_limits.min_x || p(0) > position_limits.max_x){
    logError(str_fmt("Commanded x position (%f) exceeded limits (%f,%f)", 
      p(0), position_limits.min_x, position_limits.max_x));

    return false;
  }
  else if (p(1) < position_limits.min_y || p(1) > position_limits.max_y) {

    logError(str_fmt("Commanded y position (%f) exceeded limits (%f,%f)", 
      p(1), position_limits.min_y, position_limits.max_y));

    return false;
  }
  else if (p(2) < position_limits.min_z || p(2) > position_limits.max_z) {

    logError(str_fmt("Commanded z position (%f) exceeded limits (%f,%f)", 
      p(2), position_limits.min_z, position_limits.max_z));

    return false;
  }

  return true;
}

void TrajectoryServer::geomMsgsVector3ToEigenVector3(const geometry_msgs::Vector3& geom_vect, Eigen::Vector3d& eigen_vect){
  eigen_vect(0) = geom_vect.x;
  eigen_vect(1) = geom_vect.y;
  eigen_vect(2) = geom_vect.z;
}

void TrajectoryServer::geomMsgsVector4ToEigenVector4(const geometry_msgs::Quaternion& geom_vect, Eigen::Vector4d& eigen_vect){
  eigen_vect(0) = geom_vect.x;
  eigen_vect(1) = geom_vect.y;
  eigen_vect(2) = geom_vect.z;
  eigen_vect(3) = geom_vect.w;
}

Eigen::Vector3d TrajectoryServer::quaternionToRPY(const geometry_msgs::Quaternion& quat){
  // Quaternionf q << quat.x, quat.y, quat.z, quat.w;
  Eigen::Quaterniond q(quat.w, quat.x, quat.y, quat.z);

  Vector3d euler = q.toRotationMatrix().eulerAngles(0, 1, 2); // In roll, pitch and yaw

  return euler;
}

void TrajectoryServer::quaternion_multiplication(const Eigen::Vector4d& q1, Eigen::Vector4d& q2, Eigen::Vector4d& q_f){
  Eigen::Quaterniond q1_e(q1(3), q1(0), q1(1), q1(2));
  Eigen::Quaterniond q2_e(q2(3), q2(0), q2(1), q2(2));

  Eigen::Matrix3d R1 = q1_e.toRotationMatrix();
  Eigen::Matrix3d R2 = q2_e.toRotationMatrix();
  Eigen::Matrix3d R3 = R2.transpose();

  // std::cout << "Matrix values for R1:\n" << R1 << std::endl;
  // std::cout << "Matrix values for R3:\n" << R3 << std::endl;
  // std::cout << "Matrix values for R2:\n" << R2 << std::endl;

  Eigen::Matrix3d R_combined = R2 * R1 * R3;
  Eigen::Matrix3d R_combined2 = R2 * R3;

  // std::cout << "Matrix values for Rcombined:\n" << R_combined << std::endl;

  Eigen::Quaterniond q_combined(R_combined);

  // std::cout << "Quaternion (w, x, y, z): " 
  //         << q_combined.w() << ", " 
  //         << q_combined.x() << ", " 
  //         << q_combined.y() << ", " 
  //         << q_combined.z() << std::endl;


  q_f = Eigen::Vector4d(q_combined.x(), q_combined.y(), q_combined.z(), q_combined.w());

}


