#include <fake_map/fake_map.h>

FakeMap::FakeMap(ros::NodeHandle &nodeHandle) : _nh(nodeHandle) {
	// Laser ray tracing params
	int _hrz_laser_line_num;
	int _vtc_laser_line_num;
	double _resolution;
	double _sensing_range;
	double _hrz_laser_range_dgr;
	double _vtc_laser_range_dgr;

	double tf_listen_freq;
	double sensor_refresh_freq;

	/* ROS Params */
	_nh.param<std::string>("uav/id", _id, "");
	_nh.param<double>("uav/tf_listen_freq", tf_listen_freq, -1.0);
	_nh.param<std::string>("uav/global_frame", global_frame_, "world");
	_nh.param<std::string>("uav/origin_frame", uav_origin_frame_, "world");
	_nh.param<std::string>("uav/sensor_frame", sensor_frame_, "camera_link");

	_nh.param<double>("fake_laser/sensing_range", _sensing_range, -1.0);
	_nh.param<std::string>("fake_laser/filepath", map_filepath_, "");
	_nh.param<double>("fake_laser/refresh_rate", sensor_refresh_freq, 0.0);
	_nh.param<double>("fake_laser/resolution", _resolution, 0.0);
	_nh.param<int>("fake_laser/horizontal/laser_line_num", _hrz_laser_line_num, 0);
	_nh.param<int>("fake_laser/vertical/laser_line_num", _vtc_laser_line_num, 0);
	_nh.param<double>("fake_laser/horizontal/laser_range_dgr", _hrz_laser_range_dgr, 0.0);
	_nh.param<double>("fake_laser/vertical/laser_range_dgr", _vtc_laser_range_dgr, 0.0);

	std::string copy_id = _id; 
	std::string uav_id_char = copy_id.erase(0,5); // removes first 5 character
	uav_id = std::stoi(uav_id_char);

	/** Subscriber to quad pose */
	// pose_sub_ = _nh.subscribe<geometry_msgs::PoseStamped>(
	// 	"/uav/pose", 10, &FakeMap::poseCB, this);
		
	/** Publisher that publishes local pointcloud */
	fake_sensor_cloud_pub_ = _nh.advertise<sensor_msgs::PointCloud2>(
		"/uav/sensor_cloud", 10);

	/**
	 * Timers that handles drone state at each time frame 
	*/
	tf_listen_timer_ = _nh.createTimer(
		ros::Duration(1/tf_listen_freq), 
		&FakeMap::TFListenCB, this, false, false);

	sensor_refresh_timer_ = _nh.createTimer(
		ros::Duration(1/sensor_refresh_freq), 
		&FakeMap::sensorRefreshTimerCB, this, false, false);

	printf("[quad] %sdrone%d%s pcd path: %s\n", KGRN, uav_id, KNRM, map_filepath_.c_str());
	// try to load the file
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(
		map_filepath_, _global_map) == -1) 
	{
		printf("[quad] %sdrone%d%s no valid pcd used!\n", KRED, uav_id, KNRM);
	}

	fake_laser_.set_parameters(
		_resolution,
		_sensing_range,
		_global_map,
		_vtc_laser_range_dgr,
		_hrz_laser_range_dgr,
		_vtc_laser_line_num,
		_hrz_laser_line_num);

	printf("[quad] %sdrone%d%s created! \n", KGRN, uav_id, KNRM);

	sensor_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);


    // Transformations
    tfListener_.reset(new tf2_ros::TransformListener(tfBuffer_));

	tf_listen_timer_.start();
	sensor_refresh_timer_.start();
}

FakeMap::~FakeMap()
{
	tf_listen_timer_.stop();
	sensor_refresh_timer_.stop();
	_global_map.points.clear();
}

/* Subscriber Callbacks*/
// void FakeMap::poseCB(const geometry_msgs::PoseStamped::ConstPtr &msg)
// {
// 	pose_sensor_mutex.lock();
// 	pose_ = *msg;
// 	pose_sensor_mutex.unlock();
// }

/* Timer Callbacks*/

void FakeMap::TFListenCB(const ros::TimerEvent &)
{
	/** For visualization */
	getCamLinkTF();
}

void FakeMap::sensorRefreshTimerCB(const ros::TimerEvent &)
{
	// ros::Time start = ros::Time::now();

	// pose_sensor_mutex.lock();
	// geometry_msgs::PoseStamped pose_copy = pose_;
	// pose_sensor_mutex.unlock();

	// Eigen::Quaterniond q;
	// q.x() = pose_copy.pose.orientation.x;
	// q.y() = pose_copy.pose.orientation.y;
	// q.z() = pose_copy.pose.orientation.z;
	// q.w() = pose_copy.pose.orientation.w;
	// Eigen::Matrix3d rot(q);
	// Eigen::Vector3d pos(
		// pose_copy.pose.position.x, 
		// pose_copy.pose.position.y, 
		// pose_copy.pose.position.z);

	sensor_tf_mutex_.lock();
	geometry_msgs::TransformStamped sens_to_gbl_tf_cpy = sens_to_gbl_tf_;
	sensor_tf_mutex_.unlock();

	Eigen::Quaterniond q;
	q.x() = sens_to_gbl_tf_cpy.transform.rotation.x;
	q.y() = sens_to_gbl_tf_cpy.transform.rotation.y;
	q.z() = sens_to_gbl_tf_cpy.transform.rotation.z;
	q.w() = sens_to_gbl_tf_cpy.transform.rotation.w;
	Eigen::Matrix3d sensor_rot_(q);
	Eigen::Vector3d sensor_pos_(
		sens_to_gbl_tf_cpy.transform.translation.x, 
		sens_to_gbl_tf_cpy.transform.translation.y, 
		sens_to_gbl_tf_cpy.transform.translation.z);

	// Generate point cloud from position and orientation of sensor
	fake_laser_.render_sensed_points(sensor_pos_, sensor_rot_, *sensor_cloud_);

	// Create transformation matrix from global to sensor frame
	tf::StampedTransform transform_sens_to_gbl;
	tf::transformStampedMsgToTF(sens_to_gbl_tf_cpy, transform_sens_to_gbl);
	geometry_msgs::Transform  gbl_to_sens_tf;
	tf::transformTFToMsg(transform_sens_to_gbl.inverse(), gbl_to_sens_tf);

	Eigen::Matrix4d gbl_to_sens_tf_mat;
	gbl_to_sens_tf_mat.block<3, 3>(0, 0) = Eigen::Quaterniond(gbl_to_sens_tf.rotation.w,
											gbl_to_sens_tf.rotation.x,
											gbl_to_sens_tf.rotation.y,
											gbl_to_sens_tf.rotation.z).toRotationMatrix();
	gbl_to_sens_tf_mat(0, 3) = gbl_to_sens_tf.translation.x;
	gbl_to_sens_tf_mat(1, 3) = gbl_to_sens_tf.translation.y;
	gbl_to_sens_tf_mat(2, 3) = gbl_to_sens_tf.translation.z;
	gbl_to_sens_tf_mat(3, 3) = 1.0;

	// Transform cloud from global to camera frame
	pcl::transformPointCloud (*sensor_cloud_, *sensor_cloud_, gbl_to_sens_tf_mat);

	// Publish cloud 
	sensor_msgs::PointCloud2 sensor_cloud_msg;
    pcl::toROSMsg(*sensor_cloud_, sensor_cloud_msg);

    sensor_cloud_msg.header.frame_id = sensor_frame_;
    sensor_cloud_msg.header.stamp = ros::Time::now();

    fake_sensor_cloud_pub_.publish(sensor_cloud_msg);
	
	// printf("[quad] %sdrone%d%s mapping time %.3lfms\n", 
	// 	KGRN, uav_id, KNRM, (ros::Time::now() - start).toSec() * 1000);
}

/* TF methods */

void FakeMap::getCamLinkTF()
{
	sensor_tf_mutex_.lock();
    bool lookup_success = false;
	try
    {
		// Get transform from sensor_frame to global frame
      	sens_to_gbl_tf_ = tfBuffer_.lookupTransform(global_frame_, sensor_frame_, ros::Time(0));
    }
    catch (const tf2::TransformException &ex)
    {
      ROS_ERROR_STREAM(
          "[Fake map]: Error in lookupTransform of " << sensor_frame_ << " in " << global_frame_);
      ROS_WARN("%s",ex.what());
	  lookup_success = false;
    }
	got_tf_ = lookup_success;

	sensor_tf_mutex_.unlock();
}
