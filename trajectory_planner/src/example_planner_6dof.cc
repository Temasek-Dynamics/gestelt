#include <trajectory_planner/example_planner.h>

ExamplePlanner::ExamplePlanner(ros::NodeHandle& nh)
    : nh_(nh),
      max_v_(2.0),
      max_a_(2.0),
      max_ang_v_(2.0),
      max_ang_a_(2.0),
      current_velocity_(Eigen::Vector3d::Zero()),
      current_angular_velocity_(Eigen::Vector3d::Zero()),
      current_pose_(Eigen::Affine3d::Identity()) {
        
  // Load params
  if (!nh_.getParam(ros::this_node::getName() + "/max_v", max_v_)){
    ROS_WARN("[example_planner] param max_v not found");
  }
  if (!nh_.getParam(ros::this_node::getName() + "/max_a", max_a_)){
    ROS_WARN("[example_planner] param max_a not found");
  }
  if (!nh_.getParam(ros::this_node::getName() + "/max_ang_v", max_ang_v_)){
    ROS_WARN("[example_planner] param max_ang_v not found");
  }
  if (!nh_.getParam(ros::this_node::getName() + "/max_ang_a", max_ang_a_)){
    ROS_WARN("[example_planner] param max_ang_a not found");
  }
        
  // create publisher for RVIZ markers
  pub_markers_ =
      nh.advertise<visualization_msgs::MarkerArray>("trajectory_markers", 0);

  pub_trajectory_ =
      nh.advertise<mav_planning_msgs::PolynomialTrajectory>("trajectory",
                                                              0);

  // subscriber for Odometry
  sub_odom_ =
      nh.subscribe("uav_pose", 1, &ExamplePlanner::uavOdomCallback, this);

  goal_waypoints_sub_ = nh.subscribe("/planner/goals", 10, &ExamplePlanner::waypointsCB, this);
}

// Callback to get current Pose of UAV
void ExamplePlanner::uavOdomCallback(const nav_msgs::Odometry::ConstPtr& odom) {

  // store current position in our planner
  tf::poseMsgToEigen(odom->pose.pose, current_pose_);

  // store current velocity
  tf::vectorMsgToEigen(odom->twist.twist.linear, current_velocity_);
  tf::vectorMsgToEigen(odom->twist.twist.angular, current_angular_velocity_);
}

void ExamplePlanner::waypointsCB(const gestelt_msgs::GoalsPtr &msg){
  goal_waypoints_linear_.clear(); 
  goal_waypoints_angular_.clear();
  goal_waypoints_vel_linear_.clear();
  goal_waypoints_vel_angular_.clear();
  ROS_INFO("[Trajectory Planner] No. of waypoints: %ld", msg->waypoints_linear.size());
  ADD_VEL_CONSTRAINT_ = msg->ADD_VEL_CONSTRAINT; 
  for (auto position : msg->waypoints_linear)  {
    // ROS_INFO("[Trajectory Planner] Waypoints are: %f, %f, %f", position.x, position.y, position.z);
    Eigen::Vector3d wp_linear(position.x, position.y, position.z);
    goal_waypoints_linear_.push_back(wp_linear);
  }

  for (auto orientation : msg->waypoints_angular)  {
    // ROS_INFO("[Trajectory Planner] Waypoints are: %f, %f, %f", orientation.x, orientation.y, orientation.z);
    Eigen::Vector3d wp_angular;
    Eigen::Affine3d affineTransformation(Eigen::AngleAxisd(orientation.z, Eigen::Vector3d::UnitZ()) *
                                        Eigen::AngleAxisd(orientation.y, Eigen::Vector3d::UnitY()) *
                                        Eigen::AngleAxisd(orientation.x, Eigen::Vector3d::UnitX()));
    
    mav_msgs::vectorFromRotationMatrix(affineTransformation.rotation(), &wp_angular); 
    goal_waypoints_angular_.push_back(wp_angular);

  }

    for (auto vel_linear : msg->waypoints_velocity_linear)  {
    // ROS_INFO("[Trajectory Planner] Waypoints are: %f, %f, %f", vel_linear.x, vel_linear.y, vel_linear.z);
    Eigen::Vector3d wp_vel_linear(vel_linear.x, vel_linear.y, vel_linear.z);
    goal_waypoints_vel_linear_.push_back(wp_vel_linear);
  }

  for (auto vel_angular : msg->waypoints_velocity_angular)  {
    // ROS_INFO("[Trajectory Planner] Waypoints are: %f, %f, %f", vel_angular.x, vel_angular.y, vel_angular.z);
    Eigen::Vector3d wp_vel_angular(vel_angular.x, vel_angular.y, vel_angular.z);
    goal_waypoints_vel_angular_.push_back(wp_vel_angular);
  }

  mav_trajectory_generation::Trajectory trajectory;
  planTrajectory(goal_waypoints_linear_, 
                  goal_waypoints_angular_, 
                  goal_waypoints_vel_linear_, 
                  goal_waypoints_vel_angular_, 
                  ADD_VEL_CONSTRAINT_,
                  &trajectory);
  publishTrajectory(trajectory);
}

// Method to set maximum speed.
void ExamplePlanner::setMaxSpeed(const double max_v) {
  max_v_ = max_v;
}

// Plans a trajectory from the current position to the a goal position and velocity
bool ExamplePlanner::planTrajectory(
    const std::vector<Eigen::Vector3d>& goal_pos_linear, 
    const std::vector<Eigen::Vector3d>& goal_pos_angular,  
    const std::vector<Eigen::Vector3d>& goal_vel_linear,
    const std::vector<Eigen::Vector3d>& goal_vel_angular,
    bool ADD_VEL_CONSTRAINT,
    mav_trajectory_generation::Trajectory* trajectory) {
    // Your implementation here

  assert(trajectory);
  trajectory->clear();
  // 3 Dimensional trajectory => 3D position
  // 4 Dimensional trajectory => 3D position + yaw
  // 6 Dimensional trajectory => through SE(3) space, position and orientation 
  bool success = false;
  if (goal_pos_linear.size() == goal_pos_angular.size()) 
  {
    mav_trajectory_generation::Trajectory trajectory_trans, trajectory_rot;
    success = planLinearTrajectory(goal_pos_linear, 
                                    goal_vel_linear, 
                                    current_pose_.translation(),
                                    current_velocity_,
                                    max_v_,
                                    max_a_, 
                                    ADD_VEL_CONSTRAINT,
                                    &trajectory_trans);

    Eigen::Vector3d current_rot_vec;
    mav_msgs::vectorFromRotationMatrix(current_pose_.rotation(), &current_rot_vec);
    success &= planAngularTrajectory(goal_pos_angular,
                                    goal_vel_angular,
                                    current_rot_vec,
                                    current_angular_velocity_,
                                    max_ang_v_, 
                                    max_ang_a_, 
                                    ADD_VEL_CONSTRAINT,
                                    &trajectory_rot);

    
    // Combine trajectories.
    success &= trajectory_trans.getTrajectoryWithAppendedDimension(trajectory_rot, &(*trajectory));
  
    return true;
  // } 
  // else if (dimension == 3) 
  // {
  //   success = planTrajectory(
  //       goal_pos, goal_vel, current_pose_.translation(), current_velocity_,
  //       max_v_, max_a_, &(*trajectory));
  //   return success;
  // } 
  // else if (dimension == 4) 
  // {
  //   Eigen::Vector4d start_pos_4d, start_vel_4d;
  //   start_pos_4d << current_pose_.translation(),
  //       mav_msgs::yawFromQuaternion(
  //           (Eigen::Quaternion)current_pose_.rotation());
  //   start_vel_4d << current_velocity_, 0.0; 
  //   success = planTrajectory(
  //       goal_pos, goal_vel, start_pos_4d, start_vel_4d, max_v_, max_a_,
  //       &(*trajectory));
  //   return success;
  } 
  else 
  {
    ROS_WARN("[WARNING] Given waypoints does not have equal number of position and orientation waypoints");
    return false;
  }
}

// Plans a trajectory from a start position and velocity to a goal position and velocity
bool ExamplePlanner::planLinearTrajectory(const std::vector<Eigen::Vector3d>& goal_pos,
                                    const std::vector<Eigen::Vector3d>& goal_vel,
                                    const Eigen::Vector3d& start_pos,
                                    const Eigen::Vector3d& start_vel,
                                    double v_max, double a_max, 
                                    bool ADD_VEL_CONSTRAINT,
                                    mav_trajectory_generation::Trajectory* trajectory) {
  assert(trajectory);
  const int dimension = 3;
  // Array for all waypoints and their constraints
  mav_trajectory_generation::Vertex::Vector vertices;

  // Optimize up to 4th order derivative (SNAP)
  const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
  // we have 2 vertices:
  // start = desired start vector
  // end = desired end vector
  mav_trajectory_generation::Vertex start(dimension), end(dimension);

  /******* Configure start point *******/
  start.makeStartOrEnd(start_pos, derivative_to_optimize);
  start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, start_vel);
  vertices.push_back(start);

  /*******Configure mid points*********/
  if (goal_pos.size()>1){
    for (size_t i = 0; i < goal_pos.size() - 1; i++ ){
      mav_trajectory_generation::Vertex middle_wp(dimension);
      middle_wp.addConstraint(mav_trajectory_generation::derivative_order::POSITION, goal_pos[i]);
      if (ADD_VEL_CONSTRAINT == true){
            middle_wp.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, goal_vel[i]);
      }
      vertices.push_back(middle_wp);
    }
  } 

  /******* Configure end point *******/
  // set end point constraints to desired position and set all derivatives to zero
  end.makeStartOrEnd(goal_pos.back(), derivative_to_optimize);
  end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, Eigen::Vector3d::Zero());
  vertices.push_back(end);

  // setimate initial segment times
  std::vector<double> segment_times;
  segment_times = estimateSegmentTimesVelocityRamp(vertices, v_max, a_max);

  // Set up polynomial solver with default params
  mav_trajectory_generation::NonlinearOptimizationParameters parameters;

  // set up optimization problem
  const int N = 10;
  mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
  opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);

  // constrain velocity and acceleration
  opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, v_max);
  opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, a_max);

  // solve trajectory
  opt.optimize();

  // get trajectory as polynomial parameters
  opt.getTrajectory(&(*trajectory));
  trajectory->scaleSegmentTimesToMeetConstraints(v_max, a_max);
  
  return true;
}

bool ExamplePlanner::planAngularTrajectory(const std::vector<Eigen::Vector3d>& goal_pos,
                                    const std::vector<Eigen::Vector3d>& goal_vel,
                                    const Eigen::Vector3d& start_pos,
                                    const Eigen::Vector3d& start_vel,
                                    double v_max, double a_max, 
                                    bool ADD_VEL_CONSTRAINT,
                                    mav_trajectory_generation::Trajectory* trajectory) {
  assert(trajectory);
  const int dimension = 3;
  // Array for all waypoints and their constraints
  mav_trajectory_generation::Vertex::Vector vertices;

  // Optimize up to 4th order derivative (SNAP)
  const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
  // we have 2 vertices:
  // start = desired start vector
  // end = desired end vector
  mav_trajectory_generation::Vertex start(dimension), end(dimension);

  /******* Configure start point *******/
  start.makeStartOrEnd(start_pos, derivative_to_optimize);
  start.addConstraint(mav_trajectory_generation::derivative_order::ANGULAR_VELOCITY, start_vel);
  vertices.push_back(start);

  /*******Configure mid points*********/
  if (goal_pos.size()>1){
    for (size_t i = 0; i < goal_pos.size() - 1; i++ ){
      mav_trajectory_generation::Vertex middle_wp(dimension);
      middle_wp.addConstraint(mav_trajectory_generation::derivative_order::ORIENTATION, goal_pos[i]);
      if (ADD_VEL_CONSTRAINT == true){
            middle_wp.addConstraint(mav_trajectory_generation::derivative_order::ANGULAR_VELOCITY, goal_vel[i]);
      }
      vertices.push_back(middle_wp);
    }
  } 

  /******* Configure end point *******/
  // set end point constraints to desired position and set all derivatives to zero
  end.makeStartOrEnd(goal_pos.back(), derivative_to_optimize);
  end.addConstraint(mav_trajectory_generation::derivative_order::ANGULAR_VELOCITY, Eigen::Vector3d::Zero());
  vertices.push_back(end);

  // setimate initial segment times
  std::vector<double> segment_times;
  segment_times = estimateSegmentTimesVelocityRamp(vertices, v_max, a_max);

  // Set up polynomial solver with default params
  mav_trajectory_generation::NonlinearOptimizationParameters parameters;

  // set up optimization problem
  const int N = 10;
  mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
  opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);

  // constrain velocity and acceleration
  opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, v_max);
  opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, a_max);

  // solve trajectory
  opt.optimize();

  // get trajectory as polynomial parameters
  opt.getTrajectory(&(*trajectory));
  trajectory->scaleSegmentTimesToMeetConstraints(v_max, a_max);
  
  return true;
}
                                    

bool ExamplePlanner::publishTrajectory(const mav_trajectory_generation::Trajectory& trajectory){
  // send trajectory as markers to display them in RVIZ
  visualization_msgs::MarkerArray markers;
  double distance =
      0.2; // Distance by which to seperate additional markers. Set 0.0 to disable.
  std::string frame_id = "world";

  mav_trajectory_generation::drawMavTrajectory(trajectory,
                                               distance,
                                               frame_id,
                                               &markers);
  pub_markers_.publish(markers);

  // send trajectory to be executed on UAV
  mav_planning_msgs::PolynomialTrajectory msg;
  mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(trajectory,
                                                                 &msg);
  msg.header.frame_id = "world";
  pub_trajectory_.publish(msg);
  return true;
}

