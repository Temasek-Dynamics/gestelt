#include <ruckig_trajectory_generator/ruckig_trajectory_generator.h>

RuckigPlanner::RuckigPlanner(ros::NodeHandle& nh_, 
                            ros::NodeHandle& nh_private_) : 
      publish_whole_trajectory_(false),
      dt_(0.01),
      current_sample_time_(0.0),
      current_velocity_(Eigen::Vector3d::Zero()),
      current_pose_(Eigen::Affine3d::Identity()),
      trajectory_(trajectory_) {

    nh_private_.param("publish_whole_trajectory", publish_whole_trajectory_,
                    publish_whole_trajectory_);
    nh_private_.param("dt", dt_, dt_);

    command_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>("command/trajectory", 1);   //new
    stop_srv_ = nh_.advertiseService("stop_sampling", &RuckigPlanner::stopSamplingCallback, this);      //new
    position_hold_client_ = nh_.serviceClient<std_srvs::Empty>("back_to_position_hold");            //new

    const bool oneshot = false;
    const bool autostart = false;
    publish_timer_ = nh_.createTimer(ros::Duration(dt_),
                                    &RuckigPlanner::commandTimerCallback,
                                    this, oneshot, autostart);                                        //new

    referencePublisher_ = nh_.advertise<mavros_msgs::PositionTarget>("/reference", 50);
    goalsSubscriber_ = nh_.subscribe("/planner/goals", 1, &RuckigPlanner::waypointCB, this);
    sub_odom_ = nh_.subscribe("uav_pose", 1, &RuckigPlanner::uavOdomCallback, this);
}

RuckigPlanner::~RuckigPlanner() {
    publish_timer_.stop(); 
}

// Callback to get current Pose of UAV
void RuckigPlanner::uavOdomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
  // store current position in our planner
  tf::poseMsgToEigen(odom->pose.pose, current_pose_);

  // store current velocity
  tf::vectorMsgToEigen(odom->twist.twist.linear, current_velocity_);
}


void RuckigPlanner::waypointCB(const gestelt_msgs::Goals::ConstPtr& msg) {
    goal_waypoints_.clear(); // Clear existing goal waypoints
    goal_waypoints_vel_.clear(); // Clear existing goal waypoints vel
    goal_waypoints_acc_.clear(); // Clear existing goal waypoints acc

    ROS_INFO("[Trajectory Planner] No. of waypoints: %ld", msg->waypoints.size());
    
    for (auto pose : msg->waypoints) {
        Eigen::Vector3d wp(
            pose.position.x,
            pose.position.y,
            pose.position.z);
        // Transform received waypoints from world to UAV origin frame
        goal_waypoints_.push_back(wp);
        ROS_INFO("MSG waypoints: %f, %f, %f", wp[0], wp[1], wp[2]);
    }
    for (auto vel : msg->velocities) {
        Eigen::Vector3d wp_vel( 
            vel.linear.x,
            vel.linear.y,
            vel.linear.z);
        // Transform received waypoints from world to UAV origin frame
        goal_waypoints_vel_.push_back(wp_vel);
        ROS_INFO("MSG_VEL waypoints: %f, %f, %f", wp_vel[0], wp_vel[1], wp_vel[2]);
    }

    for (auto acc : msg->accelerations) {
        Eigen::Vector3d wp_acc(
            acc.linear.x,
            acc.linear.y, 
            acc.linear.z);
        // Transform received waypoints from world to UAV origin frame
        goal_waypoints_acc_.push_back(wp_acc);
        ROS_INFO("MSG_ACC waypoints: %f, %f, %f", wp_acc[0], wp_acc[1], wp_acc[2]);
    }

    planner(goal_waypoints_, goal_waypoints_vel_, goal_waypoints_acc_);
}

void RuckigPlanner::planner(const std::vector<Eigen::Vector3d>& wp_pos,
                            const std::vector<Eigen::Vector3d>& wp_vel,
                            const std::vector<Eigen::Vector3d>& wp_acc) {
    ROS_INFO("Planner working");
    ruckig::InputParameter<3> input;

    // Replace with actual values from odom
    input.current_position = {0.0, 0.0, 1.2};
    input.current_velocity = {0.0, 0.0, 0.0};
    input.current_acceleration = {0.0, 0.0, 0.0};

    auto first_waypoint = wp_pos.front();
    auto first_waypoint_vel = wp_vel.front();
    auto first_waypoint_acc = wp_acc.front();
    std::cout<<"POS: " <<first_waypoint[0]<<", " <<first_waypoint[1]<<", "<<first_waypoint[2]<<std::endl;
    std::cout<<"VEL: " <<first_waypoint_vel[0]<<", " <<first_waypoint_vel[1]<<", "<<first_waypoint_vel[2]<<std::endl;
    std::cout<<"ACC: " <<first_waypoint_acc[0]<<", " <<first_waypoint_acc[1]<<", "<<first_waypoint_acc[2]<<std::endl;


    input.target_position = {first_waypoint[0], first_waypoint[1], first_waypoint[2]};
    input.target_velocity = {first_waypoint_vel[0], first_waypoint_vel[1], first_waypoint_vel[2]};
    input.target_acceleration = {first_waypoint_acc[0], first_waypoint_acc[1], first_waypoint_acc[2]};

    input.max_velocity = {3.0, 3.0, 3.0};
    trajectory_.~Trajectory();
    trajectory_ = ruckig::Trajectory<3>();

    ruckig::Ruckig<3> otg;
    

    // Calculate the trajectory in an offline manner
    ruckig::Result result = otg.calculate(input, trajectory_);
    if (result == ruckig::Result::ErrorInvalidInput) {
        ROS_WARN_STREAM("Invalid input!");
    }

    ROS_INFO("Trajectory duration: %.4f [s]", trajectory_.get_duration());
    processTrajectory();
    // generate_trajectory_points(trajectory_);
}



//////------------------NOT IN USE-------------------////////////
void RuckigPlanner::generate_trajectory_points(const ruckig::Trajectory<3>& trajectory) {
    ROS_INFO("Generating trajectory points");
    setPosition.clear();
    setVelocity.clear();
    setAcceleration.clear();
    double flight_duration = trajectory.get_duration();     //outputs maximum flight duration
    double time_stamp = 0;

    while (time_stamp <= flight_duration) {
        std::array<double, 3> new_position, new_velocity, new_acceleration;
        trajectory.at_time(time_stamp, new_position, new_velocity, new_acceleration);
        setPosition.push_back(new_position);
        setVelocity.push_back(new_velocity);
        setAcceleration.push_back(new_acceleration);
        time_stamp += 1.0 / 25.0;                           // 25Hz for publishing commands
        pubTrajectory(new_position, new_velocity, new_acceleration, time_stamp);
    }
}
///////--------------NOT USING-------------------//////////
///////------------ONLY FOR DEBUGGING PURPOSE-----------------//////////
void RuckigPlanner::pubTrajectory(const std::array<double, 3>& pos, 
                                const std::array<double, 3>& vel, 
                                const std::array<double, 3>& acc, 
                                double time_stamp) {
    mavros_msgs::PositionTarget msg;
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = "World";
    msg.position.x = pos[0];
    msg.position.y = pos[1];
    msg.position.z = pos[2];

    msg.velocity.x = vel[0];
    msg.velocity.y = vel[1];
    msg.velocity.z = vel[2];

    msg.acceleration_or_force.x = acc[0];
    msg.acceleration_or_force.y = acc[1];
    msg.acceleration_or_force.z = acc[2];

    msg.yaw_rate = 0;
    msg.yaw = -M_PI / 2.0;

    msg.header = header;

    referencePublisher_.publish(msg);
    ROS_INFO("Trajectory Published");
}
//////-------------------------------------///////////


void RuckigPlanner::processTrajectory(){
    // ROS_INFO("processTrajectroy started");
    // Call the service call to takeover publishing commands.
    if (position_hold_client_.exists()) {
        std_srvs::Empty empty_call;
        position_hold_client_.call(empty_call);
    }

    if (publish_whole_trajectory_) {
        //------------NOT NEEDED--------------//
        // Publish the entire trajectory at once.
        // mav_msgs::EigenTrajectoryPoint::Vector trajectory_points;
        // mav_trajectory_generation::sampleWholeTrajectory(trajectory_, dt_,
        //                                                     &trajectory_points);
        // trajectory_msgs::MultiDOFJointTrajectory msg_pub;
        // msgMultiDofJointTrajectoryFromEigen(trajectory_points, &msg_pub);
        // command_pub_.publish(msg_pub);
        //-------------------------------------//
        ROS_WARN_STREAM("[RuckigPlanner]Error in processTrajectory(), publish_whole_trajectory_=TRUE");
        return;
        } 
    else {
        publish_timer_.start();
        current_sample_time_ = 0.0;
        start_time_ = ros::Time::now();
        // ROS_INFO("processTrajectroy ended");
    }
}


bool RuckigPlanner::stopSamplingCallback(
                            std_srvs::EmptyRequest& request, 
                            std_srvs::EmptyResponse& response) {
    publish_timer_.stop();
    return true;
}
//working
void RuckigPlanner::commandTimerCallback(const ros::TimerEvent&) {
    if (current_sample_time_ <= trajectory_.get_duration()) {
        // ROS_INFO("Timer started");
        trajectory_msgs::MultiDOFJointTrajectory msg;
        trajectory_msgs::MultiDOFJointTrajectoryPoint point;
        new_position = {};
        new_velocity = {};
        new_acceleration = {};
        trajectory_.at_time(current_sample_time_, new_position, new_velocity, new_acceleration);
        std::cout<<"Total trajectory duration: " << trajectory_.get_duration()<<std::endl;
        std::cout << "new position: " << new_position[0]<<", "<<new_position[1]<<", "<<new_position[2] << std::endl;
        std::cout << "new velocity: " << new_velocity[0]<<", "<<new_velocity[1]<<", "<<new_velocity[2] << std::endl;
        std::cout << "new acceleration: " << new_acceleration[0]<<", "<<new_acceleration[1]<<", "<<new_acceleration[2] << std::endl;
        std::cout<< "Sampling Time: " << current_sample_time_<<std::endl;
        // ROS_INFO("Timer working");
        
        if (new_position.empty()) {
            publish_timer_.stop();
            ROS_WARN_STREAM("[RuckigPlanner]Empty new_position");
        }
        // ROS_INFO("Timer working 2");

        msg.header.stamp = ros::Time::now();
        msg.joint_names.push_back("base_link");

        geometry_msgs::Transform transform;
        geometry_msgs::Twist velocity, acceleration;
        transform.translation.x = new_position[0];
        transform.translation.y = new_position[1];
        transform.translation.z = new_position[2];
        point.transforms.push_back(transform);

        velocity.linear.x = new_velocity[0];
        velocity.linear.y = new_velocity[1];
        velocity.linear.z = new_velocity[2];
        point.velocities.push_back(velocity);

        acceleration.linear.x = new_acceleration[0];
        acceleration.linear.y = new_acceleration[1];
        acceleration.linear.z = new_acceleration[2];
        point.accelerations.push_back(acceleration);

        point.time_from_start = ros::Duration(current_sample_time_);

        msg.points.push_back(point);
        // ROS_INFO("Timer working 6");

        command_pub_.publish(msg);
        current_sample_time_ += dt_;
        ROS_INFO("[RuckigPlanner]Published trajectory");
    } 
    else{
        publish_timer_.stop();
    }
}