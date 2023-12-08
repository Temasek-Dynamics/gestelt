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
  ROS_INFO("[Trajectory Planner] No. of waypoints: %ld", msg->waypoints.size());
   
  for (auto pose : msg->waypoints)  {
    ROS_INFO("[Trajectory Planner] Waypoints are: %f, %f, %f, %f, %f, %f, %f", pose.position.x, pose.position.y, pose.position.z, pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    Eigen::Vector3d wp_linear;
    wp_linear << pose.position.x, pose.position.y, pose.position.z;
    tf2::Quaternion quaternion(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);

    // Convert quaternion to Euler angles
    tf2::Matrix3x3 rotation_matrix(quaternion);
    double roll, pitch, yaw;
    rotation_matrix.getRPY(roll, pitch, yaw);
    Eigen::Vector3d wp_angular(roll, pitch, yaw);    // Store Euler angles into Eigen vector
    
    goal_waypoints_linear_.push_back(wp_linear);
    goal_waypoints_angular_.push_back(wp_angular);
    ROS_INFO("[Trajectory Planner] Recieved waypoints: %f, %f, %f, %f, %f, %f", wp_linear[0], 
                                                                                wp_linear[1], 
                                                                                wp_linear[2],
                                                                                wp_angular[0], 
                                                                                wp_angular[1], 
                                                                                wp_angular[2]);

    for (const auto& vec : goal_waypoints_linear_) {
        ROS_INFO("Vector: %d, %d, %d" , vec.transpose());
    }

  }

  mav_trajectory_generation::Trajectory trajectory;
  planTrajectory(goal_waypoints_linear_, goal_waypoints_angular_, &trajectory);
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
    mav_trajectory_generation::Trajectory* trajectory) {
    // Your implementation here


  assert(trajectory);
  trajectory->clear();
  for (const auto& vec : goal_pos_linear) {
        ROS_INFO("Vector: %f, %f, %f" , vec.transpose());
    }
  // 3 Dimensional trajectory => 3D position
  // 4 Dimensional trajectory => 3D position + yaw
  // 6 Dimensional trajectory => through SE(3) space, position and orientation
  const int dimension = goal_pos_linear.size();
  bool success = false;
  Eigen::Vector3d goal_position;
  if (dimension == 3) 
  {
    mav_trajectory_generation::Trajectory trajectory_trans, trajectory_rot;
    goal_position = goal_pos_linear[0];

    // // Translation trajectory.
    
    // std::vector<Eigen::Vector3d> goal_positions_converted; // Create a new vector for converted positions

    // for (const auto& pos : goal_pos_linear) {
    // // Construct Eigen::Vector3d objects from Eigen::Matrix<double, 3, 1>
    //   goal_position << pos[0], pos[1], pos[2];
    //   goal_positions_converted.push_back(goal_position);
    // } 

    success = planTrajectory_1(
                              goal_position, current_pose_.translation(),
                              current_velocity_, max_v_, max_a_, &trajectory_trans);

    // Rotation trajectory.
    Eigen::Vector3d goal_rotation = goal_pos_angular[0];
    Eigen::Vector3d current_rot_vec;
    mav_msgs::vectorFromRotationMatrix(current_pose_.rotation(), &current_rot_vec);
    
    success &= planTrajectory_1(
                                goal_rotation, current_rot_vec, current_angular_velocity_,
                                max_ang_v_, max_ang_a_, &trajectory_rot);

    // Combine trajectories.
    success &= trajectory_trans.getTrajectoryWithAppendedDimension(trajectory_rot, &(*trajectory));
    return success;
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
  //           (Eigen::Quaterniond)current_pose_.rotation());
  //   start_vel_4d << current_velocity_, 0.0;
  //   success = planTrajectory(
  //       goal_pos, goal_vel, start_pos_4d, start_vel_4d, max_v_, max_a_,
  //       &(*trajectory));
  //   return success;
  } 
  else 
  {
    // LOG(WARNING) <<goal_positions_converted <<"            "<<goal_pos_angular<<"Dimension must be 3, 4 or 6 to be valid.";
    return false;
  }
}

// Plans a trajectory from a start position and velocity to a goal position and velocity
bool ExamplePlanner::planTrajectory_1(const Eigen::Vector3d& goal_pos,
                                    const Eigen::Vector3d& start_pos,
                                    const Eigen::Vector3d& start_vel,
                                    double v_max, double a_max,
                                    mav_trajectory_generation::Trajectory* trajectory) {
  assert(trajectory);
  const int dimension = goal_pos.size();
  // Array for all waypoints and their constraints
  mav_trajectory_generation::Vertex::Vector vertices;

  // Optimze up to 4th order derivative (SNAP)
  const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;

  // Inside the planTrajectory_1 function
  // Eigen::VectorXd goal_pos_vector(3 * goal_pos.size()); // Construct a vector of appropriate size

  // int idx = 0;
  // for (const auto& pos : goal_pos) {
  //     goal_pos_vector.segment<3>(idx) << pos[0], pos[1], pos[2];
  //     idx += 3;
  // }

// Now use goal_pos_vector in your makeStartOrEnd function


  // we have 2 vertices:
  // start = desired start vector
  // end = desired end vector
  mav_trajectory_generation::Vertex start(dimension), end(dimension);

  /******* Configure start point *******/
  start.makeStartOrEnd(start_pos, derivative_to_optimize);
  start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                      start_vel);
  vertices.push_back(start);

  /******* Configure end point *******/
  // set end point constraints to desired position and set all derivatives to zero
  end.makeStartOrEnd(goal_pos, derivative_to_optimize);
  end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                    Eigen::Vector3d::Zero());
  vertices.push_back(end);

  // setimate initial segment times
  std::vector<double> segment_times;
  segment_times = estimateSegmentTimes(vertices, v_max, a_max);

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
  ROS_INFO("[Trajectory Planner] No. of segments: %ld", msg.segments.size());
//   for (const auto& segment : msg.segments) {
//     ROS_INFO("[Trajectory planner] Published trajectory segment: %f, %f, %f, %f",
//               segment.x, segment.y, segment.z, segment.yaw);
// }

  return true;
}

