#include <fake_map/fake_sensor.h>

FakeSensor::FakeSensor(ros::NodeHandle &nh, ros::NodeHandle &pnh){
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
	bool input_pcd_file{false}; 
	std::string map_pcd_topic;
	pnh.param<bool>("fake_map/use_pcd_file", input_pcd_file, false);
	pnh.param<std::string>("fake_map/filepath", map_filepath_, "");
	pnh.param<std::string>("fake_map/input_topic", map_pcd_topic, "");

	pnh.param<std::string>("global_frame", global_frame_, "world");
	pnh.param<int>("drone_id", uav_id_, -1);
	pnh.param<std::string>("uav/origin_frame", uav_origin_frame_, "map");
	pnh.param<std::string>("uav/sensor_frame", sensor_frame_, "cam_link");
	pnh.param<double>("uav/tf_listen_freq", tf_listen_freq, -1.0);

	/* Fake laser params*/
	pnh.param<double>("fake_laser/sensor_range", _sensing_range, -1.0);
	pnh.param<double>("fake_laser/sensor_refresh_frequency", sensor_refresh_freq, -1.0);
	pnh.param<double>("fake_laser/resolution", _resolution, 0.0);
	pnh.param<int>("fake_laser/horizontal/laser_line_num", _hrz_laser_line_num, 0);
	pnh.param<int>("fake_laser/vertical/laser_line_num", _vtc_laser_line_num, 0);
	pnh.param<double>("fake_laser/horizontal/laser_range_dgr", _hrz_laser_range_dgr, 0.0);
	pnh.param<double>("fake_laser/vertical/laser_range_dgr", _vtc_laser_range_dgr, 0.0);

	/* Downsampler params*/
	pnh.param<bool>("downsampler/active", downsampler_active_, true);
	pnh.param<double>("downsampler/voxel_size", ds_voxel_size_, 0.1);

	/** Subscriber to quad pose */
	// pose_sub_ = pnh.subscribe<geometry_msgs::PoseStamped>(
	// 	"/uav/pose", 10, &FakeSensor::poseCB, this);
		
	/** Publisher that publishes local pointcloud */
	fake_sensor_cloud_pub_ = pnh.advertise<sensor_msgs::PointCloud2>(
		"/uav/sensor_cloud", 5);

	/**
	 * Timers that handles drone state at each time frame 
	*/
	tf_listen_timer_ = pnh.createTimer(
		ros::Duration(1.0/tf_listen_freq), 
		&FakeSensor::TFListenCB, this, false, false);

	sensor_update_timer_ = pnh.createTimer(
		ros::Duration(1.0/sensor_refresh_freq), 
		&FakeSensor::sensorUpdateTimerCB, this, false, false);

	if (input_pcd_file){
		printf("[fake_sensor] %sdrone%d%s pcd path: %s\n", KGRN, uav_id_, KNRM, map_filepath_.c_str());
		// try to load the file
		if (pcl::io::loadPCDFile<pcl::PointXYZ>(
			map_filepath_, global_map_) == -1) 
		{
			printf("[fake_sensor] %sdrone%d%s no valid pcd given to input! Shutting down.\n", KRED, uav_id_, KNRM);
			ros::shutdown();
		}
	}
	else {
		sensor_msgs::PointCloud2 pc_msg;
		try {	
			// Wait for 20s on point cloud topic
			printf("[fake_sensor] %sdrone%d%s Waiting for point cloud on topic %s\n", KGRN, uav_id_, KNRM, map_pcd_topic.c_str());
			pc_msg = *(ros::topic::waitForMessage<sensor_msgs::PointCloud2>(map_pcd_topic,ros::Duration(20)));
			pcl::fromROSMsg(pc_msg, global_map_);
		}
		catch (...)
		{
			ROS_ERROR("[fake_sensor] drone%d: No point cloud topic %s received. Shutting down.", uav_id_, map_pcd_topic.c_str());
			ros::shutdown();
		}
	}

	fake_laser_.set_parameters(
		_resolution,
		_sensing_range,
		global_map_,
		_vtc_laser_range_dgr,
		_hrz_laser_range_dgr,
		_vtc_laser_line_num,
		_hrz_laser_line_num);

	reset();

	tf_listen_timer_.start();
	sensor_update_timer_.start();

	// printf("[fake_sensor] %sdrone%d%s fake sensor created! \n", KGRN, uav_id_, KNRM);
}

void FakeSensor::reset()
{
	sensor_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);

	// Transformations
	tfListener_.reset(new tf2_ros::TransformListener(tfBuffer_));

	if (downsampler_active_){
		vox_grid_ = std::make_shared<pcl::VoxelGrid<pcl::PointXYZ>>();
	}
}

FakeSensor::~FakeSensor()
{
	tf_listen_timer_.stop();
	sensor_update_timer_.stop();
	global_map_.points.clear();
}

/* Subscriber Callbacks*/

// We dont use pose because we will have to perform the transformation ourselves
// void FakeSensor::poseCB(const geometry_msgs::PoseStamped::ConstPtr &msg)
// {
// 	pose_sensor_mutex_.lock();
// 	pose_ = *msg;
// 	pose_sensor_mutex_.unlock();
// }

/* Timer Callbacks*/



void FakeSensor::TFListenCB(const ros::TimerEvent &)
{
	try
    {
		std::unique_lock<std::mutex> lck(sensor_tf_mutex_, std::try_to_lock);
		if (lck.owns_lock()){
			// Get transform from sensor_frame to global frame
			sens_to_gbl_tf_ = tfBuffer_.lookupTransform(global_frame_, sensor_frame_, ros::Time(0));
			cam_tf_valid_ = true;
		}
    }
    catch (const tf2::TransformException &ex)
    {
		ROS_ERROR_THROTTLE(1,
			"[Fake sensor]: Error in lookupTransform of %s in %s", sensor_frame_.c_str(), global_frame_.c_str());
		ROS_WARN_THROTTLE(1, "%s",ex.what());
		cam_tf_valid_ = false;
		return;
    }
}

void FakeSensor::sensorUpdateTimerCB(const ros::TimerEvent &)
{
	if (!cam_tf_valid_){
		return;
	}

	// ros::Time start = ros::Time::now();

	// pose_sensor_mutex_.lock();
	// geometry_msgs::PoseStamped pose_copy = pose_;
	// pose_sensor_mutex_.unlock();

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

	geometry_msgs::TransformStamped sens_to_gbl_tf;
	std::unique_lock<std::mutex> lck(sensor_tf_mutex_, std::try_to_lock);
	if (lck.owns_lock()){
		sens_to_gbl_tf = sens_to_gbl_tf_;
	}
	else {
		return;
	}

	Eigen::Quaterniond q;
	q.x() = sens_to_gbl_tf.transform.rotation.x;
	q.y() = sens_to_gbl_tf.transform.rotation.y;
	q.z() = sens_to_gbl_tf.transform.rotation.z;
	q.w() = sens_to_gbl_tf.transform.rotation.w;
	Eigen::Matrix3d sensor_rot_(q);
	Eigen::Vector3d sensor_pos_(
		sens_to_gbl_tf.transform.translation.x, 
		sens_to_gbl_tf.transform.translation.y, 
		sens_to_gbl_tf.transform.translation.z);

	if (sensor_rot_.array().isNaN().any() || sensor_pos_.array().isNaN().any()){
		return;
	}

	// Generate point cloud from position and orientation of sensor
	fake_laser_.render_sensed_points(sensor_pos_, sensor_rot_, *sensor_cloud_);

	// Create transformation matrix from global to sensor frame
	tf::StampedTransform transform_sens_to_gbl;
	tf::transformStampedMsgToTF(sens_to_gbl_tf, transform_sens_to_gbl);
	geometry_msgs::Transform  gbl_to_sens_tf;
	tf::transformTFToMsg(transform_sens_to_gbl.inverse(), gbl_to_sens_tf);
	
	// Create transformation matrix from global to sensor frame
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

	// Downsample cloud
	if (downsampler_active_){
		vox_grid_->setInputCloud(sensor_cloud_);
		vox_grid_->setLeafSize(ds_voxel_size_, ds_voxel_size_, ds_voxel_size_);
		vox_grid_->filter(*sensor_cloud_);
	}

	// Publish cloud 
	sensor_msgs::PointCloud2 sensor_cloud_msg;
	pcl::toROSMsg(*sensor_cloud_, sensor_cloud_msg);

	sensor_cloud_msg.header.frame_id = sensor_frame_;
	sensor_cloud_msg.header.stamp = ros::Time::now();

    if (fake_sensor_cloud_pub_.getNumSubscribers() > 0) {
      fake_sensor_cloud_pub_.publish(sensor_cloud_msg);
    }


	// printf("[fake_sensor] %sdrone%d%s mapping time %.3lfms\n", 
	// 	KGRN, uav_id_, KNRM, (ros::Time::now() - start).toSec() * 1000);
}
