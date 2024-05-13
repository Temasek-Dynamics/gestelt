#include <fake_drone/fake_drone.h>

FakeDrone::FakeDrone(ros::NodeHandle& nh, ros::NodeHandle& pnh) {

	double sim_update_freq;

	/* ROS Params */
	pnh.param<int>("uav/id", drone_id_, -1);
	pnh.param<double>("uav/sim_update_frequency", sim_update_freq, -1.0);
	pnh.param<double>("uav/pose_pub_frequency", pose_pub_freq_, -1.0);
	pnh.param<double>("uav/tf_broadcast_frequency", tf_broadcast_freq_, -1.0);

	pnh.param<std::string>("uav/origin_frame", uav_origin_frame_, "world");
	pnh.param<std::string>("uav/base_link_frame", base_link_frame_, "base_link");

	pnh.param<double>("uav/offboard_timeout", offboard_timeout_, -1.0);
	pnh.param<double>("uav/init_x", init_pos_(0), 0.0);
	pnh.param<double>("uav/init_y", init_pos_(1), 0.0);
	pnh.param<double>("uav/init_z", init_pos_(2), 0.0);

	pnh.param<bool>("uav/start_in_offboard", start_in_offboard_, false);

	node_name_ = "fake_drone" + std::to_string(drone_id_);
	
	/* Subscribers and publishers*/

	setpoint_raw_local_sub_ = nh.subscribe<mavros_msgs::PositionTarget>(
		"mavros/setpoint_raw/local", 5, &FakeDrone::setpointRawCmdCb, this);
	odom_pub_ = nh.advertise<nav_msgs::Odometry>(
		"mavros/local_position/odom", 10);
	pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>(
		"mavros/local_position/pose", 10);

	mavros_state_pub_ = nh.advertise<mavros_msgs::State>("mavros/state", 10, true);

	/**
	 * Timers that handles drone state at each time frame 
	*/
	sim_update_timer_ = nh.createTimer(
		ros::Duration(1/sim_update_freq), 
		&FakeDrone::simUpdateTimer, this, false, false);

	/**
	 * Service servers
	*/

	arming_srv_server_ = nh.advertiseService("mavros/cmd/arming", 
		&FakeDrone::armSrvCb, this);

	set_mode_srv_server_ = nh.advertiseService("mavros/set_mode", 
		&FakeDrone::setModeCb, this);

	// Set initial position
	cmd_des_.pos_targ.position.x = init_pos_(0);
	cmd_des_.pos_targ.position.y = init_pos_(1);
	cmd_des_.pos_targ.position.z = init_pos_(2);

	cmd_des_.q = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);

	setStateFromCmd(cur_pose_, cmd_des_);

	// // Choose a color for the trajectory using random values
	// std::random_device dev;
	// std::mt19937 generator(dev());
	// std::uniform_real_distribution<double> dis(0.0, 1.0);
	// color_vect_ = Eigen::Vector4d(dis(generator), dis(generator), dis(generator), 0.5);

	printf("[fake_drone] %sdrone%d%s created with start_pose [%s%.2lf %.2lf %.2lf%s]! \n", 
		KGRN, drone_id_, KNRM,
		KBLU, init_pos_(0), init_pos_(1), init_pos_(2), KNRM);
	
	last_pose_pub_time_ = ros::Time::now();
	last_tf_broadcast_time_ = ros::Time::now();
	last_cmd_received_time_ = ros::Time::now();
	last_mavros_state_pub_time_ = ros::Time::now();

	if (start_in_offboard_){
		printf("[fake_drone] %sdrone%d%s starting in offboard mode! \n", 
			KGRN, drone_id_, KNRM);

		mavros_state_.armed = true;
		mavros_state_.custom_mode = "OFFBOARD";
	}

	sim_update_timer_.start();
}

FakeDrone::~FakeDrone()
{
	sim_update_timer_.stop();
}

/* Subscriber Callbacks*/

void FakeDrone::setpointRawCmdCb(const mavros_msgs::PositionTarget::ConstPtr &msg)
{	
	if (!isUAVReady()){
		ROS_WARN_THROTTLE_NAMED(1.0, node_name_, "UAV is not ready (i.e armed and OFFBOARD mode), ignoring command");
		return;
	}

	cmd_mutex_.lock();

	cmd_des_.pos_targ = *msg;
	cmd_des_.q = calcUAVOrientation(msg->acceleration_or_force, msg->yaw);

	cmd_mutex_.unlock();

	last_cmd_received_time_ = ros::Time::now();
}

/* Service callbacks */
bool FakeDrone::armSrvCb(mavros_msgs::CommandBool::Request &req,
		mavros_msgs::CommandBool::Response &res)
{
	ROS_INFO_NAMED(node_name_, "Received request to arm");

	// ros::Duration(1.0).sleep();
	mavros_state_.armed = req.value;
	res.success = true;

	ROS_INFO_NAMED(node_name_, "Arming successful");

	return true;
}

bool FakeDrone::setModeCb(mavros_msgs::SetMode::Request &req,
		mavros_msgs::SetMode::Response &res)
{
	ROS_INFO_NAMED(node_name_, "Received request set mode from %s to %s", 
		mavros_state_.custom_mode.c_str() ,req.custom_mode.c_str());

	// ros::Duration(1.0).sleep();
	mavros_state_.custom_mode = req.custom_mode;
	res.mode_sent = true;
	
	ROS_INFO_NAMED(node_name_, "Set mode to %s successful", req.custom_mode.c_str());

	return true;
}

/* Timer Callbacks*/

void FakeDrone::simUpdateTimer(const ros::TimerEvent &)
{
	setStateFromCmd(cur_pose_, cmd_des_);

	if ((ros::Time::now() - last_mavros_state_pub_time_).toSec() > (1/pose_pub_freq_))
	{
		pubMavrosState();
		last_mavros_state_pub_time_ = ros::Time::now();
	}

	if ((ros::Time::now() - last_pose_pub_time_).toSec() > (1/pose_pub_freq_))
	{
		state_mutex_.lock();
		odom_pub_.publish(cur_pose_.odom);
		pose_pub_.publish(cur_pose_.pose);
		state_mutex_.unlock();

		last_pose_pub_time_ = ros::Time::now();
	}

	if ((ros::Time::now() - last_tf_broadcast_time_).toSec() > (1/tf_broadcast_freq_))
	{
		// origin to base link tf
		geometry_msgs::TransformStamped o_to_bl_tf;

		o_to_bl_tf.header.stamp = ros::Time::now();
		o_to_bl_tf.header.frame_id = uav_origin_frame_;
		o_to_bl_tf.child_frame_id = base_link_frame_;
		
		o_to_bl_tf.transform.translation.x = cur_pose_.pose.pose.position.x;
		o_to_bl_tf.transform.translation.y = cur_pose_.pose.pose.position.y;
		o_to_bl_tf.transform.translation.z = cur_pose_.pose.pose.position.z;

		o_to_bl_tf.transform.rotation = cur_pose_.pose.pose.orientation;
		
		tf_broadcaster_.sendTransform(o_to_bl_tf);

		last_tf_broadcast_time_ = ros::Time::now();
	}

}

/* Checks */

bool FakeDrone::isUAVReady() 
{
return (mavros_state_.custom_mode == "OFFBOARD") 
	&& mavros_state_.armed;
}

bool FakeDrone::isOffboardCmdTimeout(double timeout)
{
	// isOffboardCmdTimeout is true if command exceeds timeout
	ros::Duration duration = ros::Time::now() - last_cmd_received_time_;
	return duration.toSec() > timeout;
}

/* Helper Methods */

Eigen::Quaterniond FakeDrone::calcUAVOrientation(
	const geometry_msgs::Vector3& acc, const double& yaw_rad)
{
	Eigen::Vector3d alpha = Eigen::Vector3d(acc.x, acc.y, acc.z) + Eigen::Vector3d(0,0,9.81);
	Eigen::Vector3d xC(cos(yaw_rad), sin(yaw_rad), 0);
	Eigen::Vector3d yC(-sin(yaw_rad), cos(yaw_rad), 0);
	Eigen::Vector3d xB = (yC.cross(alpha)).normalized();
	Eigen::Vector3d yB = (alpha.cross(xB)).normalized();
	Eigen::Vector3d zB = xB.cross(yB);

	Eigen::Matrix3d R;
	R.col(0) = xB;
	R.col(1) = yB;
	R.col(2) = zB;

	Eigen::Quaterniond q(R);
	return q;
}

void FakeDrone::setStateFromCmd(FakeDrone::Pose& state, const FakeDrone::Command& cmd ){
	state_mutex_.lock();

	// Set odom
	state.odom.header.stamp = ros::Time::now();
	state.odom.header.frame_id = uav_origin_frame_;

	state.odom.pose.pose.position = cmd.pos_targ.position;
	state.odom.twist.twist.linear = cmd.pos_targ.velocity;

	state.odom.pose.pose.orientation.w = cmd.q.w();
	state.odom.pose.pose.orientation.x = cmd.q.x();
	state.odom.pose.pose.orientation.y = cmd.q.y();
	state.odom.pose.pose.orientation.z = cmd.q.z();

	// Set Pose
	state.pose.header.stamp = ros::Time::now();
	state.pose.header.frame_id = uav_origin_frame_;

	state.pose.pose.position = cmd.pos_targ.position;

	state.pose.pose.orientation.w = cmd.q.w();
	state.pose.pose.orientation.x = cmd.q.x();
	state.pose.pose.orientation.y = cmd.q.y();
	state.pose.pose.orientation.z = cmd.q.z();

	state_mutex_.unlock();
}

void FakeDrone::pubMavrosState()
{
	mavros_msgs::State msg;

	msg.connected = true;
	msg.armed = mavros_state_.armed;
	msg.mode = mavros_state_.custom_mode;

	mavros_state_pub_.publish(msg);
}

