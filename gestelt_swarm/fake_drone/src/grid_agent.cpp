#include <fake_drone/grid_agent.h>

GridAgent::GridAgent(ros::NodeHandle& nh, ros::NodeHandle& pnh) {

	double sim_update_freq;

	/* ROS Params */
	pnh.param<int>("drone_id", drone_id_, -1);
	pnh.param<double>("sim_update_frequency", sim_update_freq, -1.0);
	pnh.param<double>("pose_pub_frequency", pose_pub_freq_, -1.0);
	pnh.param<double>("tf_broadcast_frequency", tf_broadcast_freq_, -1.0);

	pnh.param<std::string>("origin_frame", uav_origin_frame_, "world");
	pnh.param<std::string>("base_link_frame", base_link_frame_, "base_link");

	pnh.param<double>("init_x", init_pos_(0), 0.0);
	pnh.param<double>("init_y", init_pos_(1), 0.0);
	pnh.param<double>("init_z", init_pos_(2), 0.0);

	node_name_ = "fake_drone" + std::to_string(drone_id_);
	
	/* Subscribers and publishers*/

	fe_plan_sub_ = nh.subscribe<gestelt_msgs::FrontEndPlan>(
		"fe_plan", 5, &GridAgent::frontEndPlanCB, this);

	pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>(
		"mavros/local_position/pose", 10);
	odom_pub_ = nh.advertise<nav_msgs::Odometry>(
		"mavros/local_position/odom", 10);

	/**
	 * Timers that handles drone state at each time frame 
	*/
	sim_update_timer_ = nh.createTimer(
		ros::Duration(1/sim_update_freq), 
		&GridAgent::simUpdateTimer, this, false, false);

	/**
	 * Service servers
	*/

	// Set initial position
	pose_msg_.pose.position.x = init_pos_(0);
	pose_msg_.pose.position.y = init_pos_(1);
	pose_msg_.pose.position.z = init_pos_(2);

	odom_msg_.pose.pose.position.x = init_pos_(0);
	odom_msg_.pose.pose.position.y = init_pos_(1);
	odom_msg_.pose.pose.position.z = init_pos_(2);

	odom_msg_.header.frame_id = uav_origin_frame_;
	pose_msg_.header.frame_id = uav_origin_frame_;

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

	sim_update_timer_.start();
}

GridAgent::~GridAgent()
{
	sim_update_timer_.stop();
}

/* Subscriber Callbacks*/

void GridAgent::frontEndPlanCB(const gestelt_msgs::FrontEndPlan::ConstPtr &msg)
{	
	fe_plan_msg_ = *msg;
	plan_start_exec_t_ = ros::Time::now().toSec();

	// Generate spline from plan 
	std::vector<tinyspline::real> points;

	for (int i = 0; i < fe_plan_msg_.plan.size(); i+= 10)
	{	
		// Add control point
		points.push_back(fe_plan_msg_.plan[i].position.x);
		points.push_back(fe_plan_msg_.plan[i].position.y);
		points.push_back(fe_plan_msg_.plan[i].position.z);
	}

	spline_ = std::make_shared<tinyspline::BSpline>(tinyspline::BSpline::interpolateCubicNatural(points, 3));

	plan_received_ = true;
}

/* Timer Callbacks*/

void GridAgent::simUpdateTimer(const ros::TimerEvent &)
{
	odom_msg_.header.stamp = ros::Time::now();
	pose_msg_.header.stamp = ros::Time::now();

	if (plan_received_){
		setStateFromPlan(fe_plan_msg_, plan_start_exec_t_);
	}

	if ((ros::Time::now() - last_pose_pub_time_).toSec() > (1/pose_pub_freq_))
	{
		state_mutex_.lock();
		odom_pub_.publish(odom_msg_);
		pose_pub_.publish(pose_msg_);
		state_mutex_.unlock();

		last_pose_pub_time_ = ros::Time::now();
	}

	// if ((ros::Time::now() - last_tf_broadcast_time_).toSec() > (1/tf_broadcast_freq_))
	// {
	// 	// origin to base link tf
	// 	geometry_msgs::TransformStamped o_to_bl_tf;

	// 	o_to_bl_tf.header.stamp = ros::Time::now();
	// 	o_to_bl_tf.header.frame_id = uav_origin_frame_;
	// 	o_to_bl_tf.child_frame_id = base_link_frame_;
		
	// 	o_to_bl_tf.transform.translation.x = pose_msg_.pose.position.x;
	// 	o_to_bl_tf.transform.translation.y = pose_msg_.pose.position.y;
	// 	o_to_bl_tf.transform.translation.z = pose_msg_.pose.position.z;

	// 	o_to_bl_tf.transform.rotation = pose_msg_.pose.orientation;
		
	// 	tf_broadcaster_.sendTransform(o_to_bl_tf);

	// 	last_tf_broadcast_time_ = ros::Time::now();
	// }

}

/* Checks */

/* Helper Methods */

void GridAgent::setStateFromPlan(	const gestelt_msgs::FrontEndPlan &fe_plan_msg, 
																	const double& exec_start_t)
{

	double t_now = ros::Time::now().toSec();

	if (t_now >= exec_start_t + fe_plan_msg.plan_time.back()){
		// std::cout << "t_now (" << t_now << ") exceeds end of traj time (" << fe_plan_msg.plan_time.back()<< ")" << std::endl;
		// Time exceeded end of trajectory. Trajectory published in the past

		plan_received_ = false;
		return;
	}

	// // Iterate through plan and choose state at current time
	// size_t cur_plan_idx = 0;

	// for (size_t i = 0; i < fe_plan_msg.plan_time.size(); i++){
	// 	if (t_now < exec_start_t + fe_plan_msg.plan_time[i]){ // Future point found
	// 		cur_plan_idx = i; 
	// 		break;
	// 	}
	// }

	// alpha: Arc length parameterization of spline. Formed by time ratio
	double alpha = (t_now - exec_start_t) / fe_plan_msg.plan_time.back();

	std::vector<tinyspline::real> result = spline_->eval(alpha).result();
	double x = result[0];
	double y = result[1];
	double z = result[2];

	state_mutex_.lock();

	// Set odom
	odom_msg_.header.stamp = ros::Time::now();

	// odom_msg_.pose.pose = fe_plan_msg.plan[cur_plan_idx];
	odom_msg_.pose.pose.position.x = x;
	odom_msg_.pose.pose.position.y = y;
	odom_msg_.pose.pose.position.z = z;

	// Set Pose
	pose_msg_.header.stamp = ros::Time::now();

	// pose_msg_.pose = fe_plan_msg.plan[cur_plan_idx];
	pose_msg_.pose.position.x = x;
	pose_msg_.pose.position.y = y;
	pose_msg_.pose.position.z = z;


	state_mutex_.unlock();
}
