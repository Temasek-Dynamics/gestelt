#include <trajectory_planner/example_planner.h>

ExamplePlanner::ExamplePlanner(ros::NodeHandle& nh) :
    nh_(nh),
    max_v_(5.0),
    max_a_(8.0),
    max_j_(10.0),
    segment_time_factor_(0.6),
    current_velocity_(Eigen::Vector3d::Zero()),
    current_pose_(Eigen::Affine3d::Identity()) {
      
  // Load params
  if (!nh_.getParam(ros::this_node::getName() + "/max_v", max_v_)){
    ROS_WARN("[example_planner] param max_v not found");
  }
  if (!nh_.getParam(ros::this_node::getName() + "/max_a", max_a_)){
    ROS_WARN("[example_planner] param max_a not found");
  }
  if (!nh_.getParam(ros::this_node::getName() + "/segment_time_scaling_factor", segment_time_factor_)){
    ROS_WARN("[example_planner] param max_a not found");
  }

  nh.param("trajectory_frame", trajectory_frame_id_, std::string("world"));

  // create publisher for RVIZ markers
  pub_markers_ =
      nh.advertise<visualization_msgs::MarkerArray>("trajectory_markers", 0);

  pub_trajectory_ =
      nh.advertise<mav_planning_msgs::PolynomialTrajectory4D>("trajectory",
                                                              0);

  pub_traj_total_time_ = nh.advertise<std_msgs::Float32>("trajectory_total_time", 0);
  // subscriber for Odometry
  sub_odom_ =
      nh.subscribe("uav_pose", 1, &ExamplePlanner::uavOdomCallback, this);

  goal_waypoints_sub_ = nh.subscribe("/waypoints", 1, &ExamplePlanner::waypointsCB, this);
  // time_factor_sub_ = nh.subscribe("/planner/time_factor", 1, &ExamplePlanner::timeFactorCB, this);
  // std::cout<<"Max vel: "<<max_v_<<std::endl<<"Max Acc: " <<max_a_<<std::endl <<"Max Jerk: "<<max_j_<<std::endl;
  
}

// Callback to get current Pose of UAV
void ExamplePlanner::uavOdomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
  // store current position in our planner
  tf::poseMsgToEigen(odom->pose.pose, current_pose_);

  // store current velocity
  tf::vectorMsgToEigen(odom->twist.twist.linear, current_velocity_);
}
// // Callback to get the time factor of each segment
// void ExamplePlanner::timeFactorCB(const std_msgs::Float32::ConstPtr &msg){
//   segment_time_factor_ = msg->data;
// }
// Callback to get waypoints
void ExamplePlanner::waypointsCB(const gestelt_msgs::GoalsPtr &msg){
  goal_waypoints_.clear(); // Clear existing goal waypoints
  goal_waypoints_vel_.clear(); // Clear existing goal waypoints vel
  goal_waypoints_acc_.clear(); // Clear existing goal waypoints acc
  segment_time_factor_ = msg->time_factor.data;
  max_v_ = msg->max_vel.data;
  max_a_ = msg->max_acc.data;

  ROS_INFO("[Trajectory Planner] No. of waypoints: %ld", msg->waypoints.size());
   
  for (auto pose : msg->waypoints) {
    Eigen::Vector3d wp(
        pose.position.x,
        pose.position.y,
        pose.position.z);
    // Transform received waypoints from world to UAV origin frame
    goal_waypoints_.push_back(wp);
    // ROS_INFO("MSG waypoints: %f, %f, %f", wp[0], wp[1], wp[2]);
  }
  for (auto vel : msg->velocities) {
    Eigen::Vector3d wp_vel(
        vel.linear.x,
        vel.linear.y,
        vel.linear.z);
    // Transform received waypoints from world to UAV origin frame
    goal_waypoints_vel_.push_back(wp_vel);
    // ROS_INFO("MSG_VEL waypoints: %f, %f, %f", wp_vel[0], wp_vel[1], wp_vel[2]);
  }

  for (auto acc : msg->accelerations) {
    Eigen::Vector3d wp_acc(
        acc.linear.x,
        acc.linear.y, 
        acc.linear.z);
    // Transform received waypoints from world to UAV origin frame
    goal_waypoints_acc_.push_back(wp_acc);
    // ROS_INFO("MSG_ACC waypoints: %f, %f, %f", wp_acc[0], wp_acc[1], wp_acc[2]);
  }

  mav_trajectory_generation::Trajectory trajectory;
  planTrajectory(goal_waypoints_,goal_waypoints_vel_, goal_waypoints_acc_,msg, &trajectory);
  publishTrajectory(trajectory);
}

// Method to set maximum speed.
void ExamplePlanner::setMaxSpeed(const double max_v) {
  max_v_ = max_v;
  // std::cout<<max_v<<std::endl;
}

// Plans a trajectory from the current position to the a goal position and velocity
// we neglect attitude here for simplicity
bool ExamplePlanner::planTrajectory(const std::vector<Eigen::Vector3d>& wp_pos,
                                    const std::vector<Eigen::Vector3d>& wp_vel,
                                    const std::vector<Eigen::Vector3d>& wp_acc,
                                    const gestelt_msgs::GoalsPtr &msg,
                                    mav_trajectory_generation::Trajectory* trajectory) {

  // 3 Dimensional trajectory => through carteisan space, no orientation
  const int dimension = 3;
  // Optimze up to 4th order derivative (SNAP)
  const int derivative_to_optimize =
      mav_trajectory_generation::derivative_order::SNAP;

  // Array for all waypoints and their constrains
  mav_trajectory_generation::Vertex::Vector vertices;

  mav_trajectory_generation::Vertex start(dimension), end(dimension);

  /******* Configure start point *******/
  start.makeStartOrEnd(current_pose_.translation(),
                       derivative_to_optimize);
  start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                      current_velocity_);
  vertices.push_back(start);

  for (size_t i = 0; i < wp_pos.size() - 1; i++ ){
    mav_trajectory_generation::Vertex middle_wp(dimension);

    middle_wp.addConstraint(mav_trajectory_generation::derivative_order::POSITION, wp_pos[i]);

    // velocity constraint
    if (wp_vel.size() > 0 && msg->velocities_mask[i].data == false ){
      middle_wp.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, wp_vel[i]);
    }

    //acceleration constraint
    if (wp_acc.size() > 0 && msg->accelerations_mask[i].data == false){
      middle_wp.addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, wp_acc[i]);
    }

    vertices.push_back(middle_wp);
  }

  /******* Configure end point *******/
  end.makeStartOrEnd(wp_pos.back(),
                     derivative_to_optimize);
  end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                    Eigen::Vector3d::Zero());
  vertices.push_back(end);

  // setimate initial segment times
  std::vector<double> segment_times;
  // segment_times = estimateSegmentTimes(vertices, max_v_, max_a_);
 
  segment_times = estimateSegmentTimesVelocityRamp(vertices, max_v_, max_a_);

  for(int i = 0; i<segment_times.size(); i++){

    // time allocation for 2 gates trajectory - 85deg and 0deg passes.
    if (i == 0 || i == segment_times.size()-1){
      segment_times[i] *= 1;
    }
    else{
      segment_times[i] *= segment_time_factor_;
    }
    

  
    // std::cout<<"MODIFIED Time allocation of segment "<<i+1<<": "<<segment_times[i]<<std::endl;
  }
  
  double total_traj_time = std::accumulate(segment_times.begin(), segment_times.end(), 0.0);
  std_msgs::Float32 total_time_msg;
  total_time_msg.data = total_traj_time;
  pub_traj_total_time_.publish(total_time_msg);
  /*
  * Linear optimization
  */
  const int N = 10;
  mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
  opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);

  opt.solveLinear();

  // Get segments
  mav_trajectory_generation::Segment::Vector segments;
  opt.getSegments(&segments);

  opt.getTrajectory(&(*trajectory));
  for(int i = 0; i<segment_times.size(); i++){
    std::cout<<"Time allocation of segment "<<i+1<<": "<<segment_times[i]<<std::endl;
  }
  /*
  * Non-linear optimization
  */

  // // Set up polynomial solver with default params
  // mav_trajectory_generation::NonlinearOptimizationParameters parameters;
  // parameters.max_iterations = 1000;
  // parameters.f_rel = 0.05;
  // parameters.x_rel = 0.1;
  // parameters.time_penalty = 500.0;
  // parameters.initial_stepsize_rel = 0.1;
  // parameters.inequality_constraint_tolerance = 0.1;

  // // set up optimization problem
  // const int N = 10;

  // mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
  // opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);

  // // constrain velocity and acceleration
  // opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, max_v_);
  // opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, max_a_);
  // // opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::JERK, max_j_);
  // opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::SNAP, 0.0);
  // // solve trajectory
  // opt.optimize();

  // // get trajectory as polynomial parameters
  // opt.getTrajectory(&(*trajectory));
  // trajectory->scaleSegmentTimesToMeetConstraints(max_v_, max_a_);
  return true;
}

// Plans a trajectory from the current position to the a goal position and velocity
// we neglect attitude here for simplicity
// bool ExamplePlanner::planTrajectory(const Eigen::VectorXd& goal_pos,
//                                     const Eigen::VectorXd& goal_vel,
//                                     mav_trajectory_generation::Trajectory* trajectory) {

//   // 3 Dimensional trajectory => through carteisan space, no orientation
//   const int dimension = 3;

//   // Array for all waypoints and their constrains
//   mav_trajectory_generation::Vertex::Vector vertices;

//   // Optimze up to 4th order derivative (SNAP)
//   const int derivative_to_optimize =
//       mav_trajectory_generation::derivative_order::SNAP;

//   // we have 2 vertices:
//   // Start = current position
//   // end = desired position and velocity
//   mav_trajectory_generation::Vertex start(dimension), end(dimension);

//   /******* Configure start point *******/
//   // set start point constraints to current position and set all derivatives to zero
//   start.makeStartOrEnd(current_pose_.translation(),
//                        derivative_to_optimize);

//   // set start point's velocity to be constrained to current velocity
//   start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
//                       current_velocity_);

//   // add waypoint to list
//   vertices.push_back(start);

//   /******* Configure end point *******/
//   // set end point constraints to desired position and set all derivatives to zero
//   end.makeStartOrEnd(goal_pos,
//                      derivative_to_optimize);

//   // set start point's velocity to be constrained to current velocity
//   end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
//                     goal_vel);

//   // add waypoint to list
//   vertices.push_back(end);

//   // setimate initial segment times
//   std::vector<double> segment_times;
//   segment_times = estimateSegmentTimes(vertices, max_v_, max_a_);

//   // Set up polynomial solver with default params
//   mav_trajectory_generation::NonlinearOptimizationParameters parameters;

//   // set up optimization problem
//   const int N = 10;
//   mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
//   opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);

//   // constrain velocity and acceleration
//   opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, max_v_);
//   opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, max_a_);

//   // solve trajectory
//   opt.optimize();

//   // get trajectory as polynomial parameters
//   opt.getTrajectory(&(*trajectory));

//   return true;
// }

bool ExamplePlanner::publishTrajectory(const mav_trajectory_generation::Trajectory& trajectory){
  // send trajectory as markers to display them in RVIZ
  visualization_msgs::MarkerArray markers;
  double distance =
      0.0; // Distance by which to seperate additional markers. Set 0.0 to disable.
  std::string frame_id = trajectory_frame_id_;

  mav_trajectory_generation::drawMavTrajectory(trajectory,
                                               distance,
                                               frame_id,
                                               &markers);
  pub_markers_.publish(markers);

  // send trajectory to be executed on UAV
  mav_planning_msgs::PolynomialTrajectory4D msg;
  mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(trajectory,
                                                                 &msg);
  msg.header.frame_id = trajectory_frame_id_;
  pub_trajectory_.publish(msg);

  ROS_INFO("[Trajectory Planner] No. of segments: %ld", msg.segments.size());

  return true;
}

