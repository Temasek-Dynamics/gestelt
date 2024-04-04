#include "grid_map/grid_map.h"

/** Initialization methods */

void GridMap::initMap(pcl::PointCloud<pcl::PointXYZ>::Ptr pcd, const Eigen::Vector3d& map_size, const double& inflation, const double& resolution)
{
  // md_.has_pose_ = true;
  // mp_.global_map_size_ = map_size;

  // mp_.inflation_ = inflation;
  // mp_.uav_origin_frame_ = "world";
  // mp_.global_map_origin_ = Eigen::Vector3d(
  //   -mp_.global_map_size_(0) / 2.0, 
  //   -mp_.global_map_size_(1) / 2.0, 
  //   0.0);
  // mp_.resolution_ = resolution;

  // mp_.global_map_num_voxels_ = (mp_.global_map_size_.cwiseProduct(Eigen::Vector3d::Constant(1/mp_.resolution_))).cast<int>();

  // reset(mp_.resolution_);

  // pcdToMap(pcd);
}

void GridMap::initMapROS(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{  
  readROSParams(nh, pnh);

  reset(mp_.resolution_);

  Eigen::Matrix3d rot_mat = (Eigen::AngleAxisd((M_PI/180.0) * md_.cam2body_rpy_deg(0), Eigen::Vector3d::UnitX())
                              * Eigen::AngleAxisd((M_PI/180.0) * md_.cam2body_rpy_deg(1), Eigen::Vector3d::UnitY())
                              * Eigen::AngleAxisd((M_PI/180.0) * md_.cam2body_rpy_deg(2), Eigen::Vector3d::UnitZ())).toRotationMatrix();

  md_.cam2body_.block<3, 3>(0, 0) =  (Eigen::AngleAxisd((M_PI/180.0) * md_.cam2body_rpy_deg(0), Eigen::Vector3d::UnitX())
                                      * Eigen::AngleAxisd((M_PI/180.0) * md_.cam2body_rpy_deg(1), Eigen::Vector3d::UnitY())
                                      * Eigen::AngleAxisd((M_PI/180.0) * md_.cam2body_rpy_deg(2), Eigen::Vector3d::UnitZ())).toRotationMatrix();
  
  // Global map origin is at a corner of the global map i.e. (-W/2, -L/2, 0)
  mp_.global_map_origin_ = Eigen::Vector3d(
    -mp_.global_map_size_(0) / 2.0, 
    -mp_.global_map_size_(1) / 2.0, 
    mp_.ground_height_);
  // Local map origin is at a corner of the local map i.e. (uav_pos_x-local_W/2, uav_pos_y-local_L/2, 0)
  // Local map origin_ is relative to the current position of the robot
  mp_.local_map_origin_ = Eigen::Vector3d(
    -mp_.local_map_size_(0) / 2.0, 
    -mp_.local_map_size_(1) / 2.0, 
    mp_.ground_height_);
  //local_map_origin_rel_uav_: local map origin relative to current position of the drone
  mp_.local_map_origin_rel_uav_ = Eigen::Vector3d(
    -mp_.local_map_size_(0) / 2.0, 
    -mp_.local_map_size_(1) / 2.0, 
    mp_.ground_height_);

  mp_.local_map_max_ = Eigen::Vector3d(
    mp_.local_map_size_(0) / 2.0, 
    mp_.local_map_size_(1) / 2.0, 
    mp_.ground_height_ + mp_.local_map_size_(2));

  mp_.global_map_num_voxels_ = (mp_.global_map_size_.cwiseProduct(Eigen::Vector3d::Constant(1/mp_.resolution_))).cast<int>();
  mp_.local_map_num_voxels_ = (mp_.local_map_size_.cwiseProduct(Eigen::Vector3d::Constant(1/mp_.resolution_))).cast<int>();

  if (dbg_input_entire_map_){
    ROS_INFO("[%s] DEBUG: INPUT ENTIRE MAP", node_name_.c_str());

    md_.has_pose_ = true;
    
		sensor_msgs::PointCloud2 pc_msg;
		try {	
      ROS_ERROR("[%s] Waiting for point cloud on topic %s", node_name_.c_str(), entire_pcd_map_topic_.c_str());
			pc_msg = *(ros::topic::waitForMessage<sensor_msgs::PointCloud2>(entire_pcd_map_topic_, ros::Duration(10)));
		}
		catch (...)
		{
			ROS_ERROR("[%s] No point cloud topic %s received. Shutting down.", node_name_.c_str(), entire_pcd_map_topic_.c_str());
			ros::shutdown();
		}
    pcdMsgToMap(pc_msg);

    // Initialize publisher for occupancy map
    occ_map_pub_ = nh.advertise<sensor_msgs::PointCloud2>("grid_map/occupancy", 10);
    // Publisher for collision visualizations
    collision_viz_pub_ = nh.advertise<visualization_msgs::Marker>("grid_map/collision_viz", 10);

    // Initialize subscriber
    if (check_collisions_){
      odom_col_check_sub_ = nh.subscribe<nav_msgs::Odometry>(
        "odom", 1, &GridMap::odomColCheckCB, this);
      check_collisions_timer_ = 
        nh.createTimer(ros::Duration(0.025), &GridMap::checkCollisionsTimerCB, this);
      pnh.param("grid_map/collision_check/warn_radius", col_warn_radius_, -1.0); // 0.25
      pnh.param("grid_map/collision_check/fatal_radius", col_fatal_radius_, -1.0); // 0.126
    }

    /* Initialize ROS Timers */
    vis_timer_ = nh.createTimer(ros::Duration(0.1), &GridMap::visTimerCB, this);
    
  }
  else {
    initROSPubSubTimers(nh, pnh);
  }

  // ROS_INFO("[%s] Map origin (%f, %f, %f)", node_name_.c_str(), 
  //   mp_.global_map_origin_(0), mp_.global_map_origin_(1), mp_.global_map_origin_(2));
  // ROS_INFO("[%s] Global map size (%f, %f, %f)", node_name_.c_str(), 
  //   mp_.global_map_size_(0), mp_.global_map_size_(1), mp_.global_map_size_(2));
  // // ROS_INFO("[%s] Local map size (%f, %f, %f)", node_name_.c_str(), 
  // //   mp_.local_map_size_(0), mp_.local_map_size_(1), mp_.local_map_size_(2));
  // ROS_INFO("[%s] Resolution %f m, Inflation %f m", node_name_.c_str(), 
    // mp_.resolution_, mp_.inflation_);

  
}

void GridMap::initROSPubSubTimers(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
  /* Initialize ROS Subscribers, publishers */
  camera_info_sub_ = nh.subscribe<sensor_msgs::CameraInfo>(
    "grid_map/camera_info", 1, &GridMap::cameraInfoCallback, this);

  // From sensor type, determine what type of sensor message to subscribe to
  if (mp_.sensor_type_ == SensorType::SENSOR_CLOUD)
  {
    ROS_INFO("[%s] Point clouds as input", node_name_.c_str());
    cloud_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(
      nh, "grid_map/cloud", 5, ros::TransportHints().tcpNoDelay()));
  }
  else if (mp_.sensor_type_ == SensorType::SENSOR_DEPTH) {
    ROS_INFO("[%s] Depth image as input", node_name_.c_str());
    depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(
      nh, "grid_map/depth", 5, ros::TransportHints().tcpNoDelay()));
  }
  else {
    ROS_ERROR("[%s] Unknown SENSOR_TYPE", node_name_.c_str());
    ros::shutdown();
  }

  // From pose type, determine what type of localization message to subscribe to
  if (mp_.pose_type_ == PoseType::POSE_STAMPED) // Pose
  {
    ROS_INFO("[%s] Pose as camera pose input", node_name_.c_str());
    pose_sub_.reset(new message_filters::Subscriber<geometry_msgs::PoseStamped>(
      nh, "grid_map/pose", 5, ros::TransportHints().udp()));
  }
  else if (mp_.pose_type_ == PoseType::ODOMETRY) // Odom
  {
    ROS_INFO("[%s] Odometry as camera pose input", node_name_.c_str());
    odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(
      nh, "grid_map/odom", 5, ros::TransportHints().udp()));
  }
  else if (mp_.pose_type_ == PoseType::TF) // TF
  {
    ROS_INFO("[%s] TF as camera pose input", node_name_.c_str());
    tfListener_.reset(new tf2_ros::TransformListener(tfBuffer_));
    tf_cloud_filter_.reset(new tf2_ros::MessageFilter<sensor_msgs::PointCloud2>(*cloud_sub_, tfBuffer_, mp_.uav_origin_frame_, 5, 0));
  }
  else { // UNKNOWN
    ROS_ERROR("[%s] Unknown POSE_TYPE", node_name_.c_str());
    ros::shutdown();
  }

  // From pose and sensor type, determine the pose/sensor pairing 
  if (mp_.pose_type_ == PoseType::POSE_STAMPED) { // Pose
    if (mp_.sensor_type_ == SensorType::SENSOR_CLOUD) { // Point cloud
      sync_cloud_pose_.reset(new message_filters::Synchronizer<SyncPolicyCloudPose>(
          SyncPolicyCloudPose(5), *cloud_sub_, *pose_sub_));
      sync_cloud_pose_->registerCallback(boost::bind(&GridMap::cloudPoseCB, this, _1, _2));
    }
    else { // Depth image
      sync_image_pose_.reset(new message_filters::Synchronizer<SyncPolicyImagePose>(
          SyncPolicyImagePose(5), *depth_sub_, *pose_sub_));
      sync_image_pose_->registerCallback(boost::bind(&GridMap::depthPoseCB, this, _1, _2));
    }
  }
  else if (mp_.pose_type_ == PoseType::ODOMETRY) { // Odom
    if (mp_.sensor_type_ == SensorType::SENSOR_CLOUD) { // Point cloud
      sync_cloud_odom_.reset(new message_filters::Synchronizer<SyncPolicyCloudOdom>(
          SyncPolicyCloudOdom(5), *cloud_sub_, *odom_sub_));
      sync_cloud_odom_->registerCallback(boost::bind(&GridMap::cloudOdomCB, this, _1, _2));
    }
    else { // Depth image
      sync_image_odom_.reset(new message_filters::Synchronizer<SyncPolicyImageOdom>(
          SyncPolicyImageOdom(5), *depth_sub_, *odom_sub_));
      sync_image_odom_->registerCallback(boost::bind(&GridMap::depthOdomCB, this, _1, _2));
    }
  }
  else if (mp_.pose_type_ == PoseType::TF) { // TF
    tf_cloud_filter_->registerCallback( boost::bind(&GridMap::cloudTFCB, this, _1) );
  }

  occ_map_pub_ = nh.advertise<sensor_msgs::PointCloud2>("grid_map/occupancy", 10);

  /* Initialize ROS Timers */
  vis_timer_ = nh.createTimer(ros::Duration(0.1), &GridMap::visTimerCB, this);
}

void GridMap::readROSParams(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
  /* Map parameters */
  pnh.param("grid_map/debug_input_entire_map", dbg_input_entire_map_, false);
  pnh.param("grid_map/entire_pcd_map_topic", entire_pcd_map_topic_, std::string("/fake_map"));

  pnh.param("grid_map/ground_height", mp_.ground_height_, 0.0);
  pnh.param("grid_map/map_size_x", mp_.global_map_size_(0), -1.0);
  pnh.param("grid_map/map_size_y", mp_.global_map_size_(1), -1.0);
  pnh.param("grid_map/map_size_z", mp_.global_map_size_(2), -1.0);

  pnh.param("grid_map/local_map_size_x", mp_.local_map_size_(0), -1.0);
  pnh.param("grid_map/local_map_size_y", mp_.local_map_size_(1), -1.0);
  pnh.param("grid_map/local_map_size_z", mp_.local_map_size_(2), -1.0);

  pnh.param("grid_map/keep_global_map", mp_.keep_global_map_, false);

  pnh.param("grid_map/occ_grid/resolution", mp_.resolution_, -1.0);
  pnh.param("grid_map/occ_grid/inflation", mp_.inflation_, -1.0);

  mp_.inf_num_voxels_ = std::ceil(mp_.inflation_/mp_.resolution_);

  /* Point cloud filter */
  pnh.param("grid_map/filter/depth_stride", mp_.depth_stride_, 4);
  pnh.param("grid_map/filter/downsample_cloud", mp_.downsample_cloud_, false);
  pnh.param("grid_map/filter/voxel_size", mp_.voxel_size_, -1.0);

  /* Sensor inputs */
  pnh.param("grid_map/pose_type", mp_.pose_type_, 1);
  pnh.param("grid_map/sensor_type", mp_.sensor_type_, 1);

  /* Camera parameters  */
  pnh.param("grid_map/k_depth_scaling_factor", mp_.k_depth_scaling_factor_, 1000.0);

  pnh.param("grid_map/cam_frame", mp_.cam_frame_, std::string("cam_link"));
  pnh.param("grid_map/global_frame", mp_.global_frame_, std::string("world"));
  pnh.param("grid_map/uav_origin_frame", mp_.uav_origin_frame_, std::string("world"));

  pnh.param("grid_map/sensor/max_range", mp_.max_range, -1.0);

  /* Camera extrinsic parameters  */
  pnh.param("grid_map/camera_to_body/roll", md_.cam2body_rpy_deg(0), 0.0);
  pnh.param("grid_map/camera_to_body/pitch", md_.cam2body_rpy_deg(1), 0.0);
  pnh.param("grid_map/camera_to_body/yaw", md_.cam2body_rpy_deg(2), 0.0);
  pnh.param("grid_map/camera_to_body/t_x", md_.cam2body_.col(3)(0), 0.0);
  pnh.param("grid_map/camera_to_body/t_y", md_.cam2body_.col(3)(1), 0.0);
  pnh.param("grid_map/camera_to_body/t_z", md_.cam2body_.col(3)(2), 0.0);
}

void GridMap::reset(const double& resolution){
  // Initialize data structures for occupancy map and point clouds
  // vox_grid_filter_.setLeafSize(mp_.voxel_size_, mp_.voxel_size_, mp_.voxel_size_);

  // Set up Bonxai data structure
  bonxai_map_ = std::make_unique<BonxaiT>(resolution);

  // Set up kdtree for generating safe flight corridors
  kdtree_ = std::make_shared<KD_TREE<pcl::PointXYZ>>(0.5, 0.6, 0.1);

}

/** Timer callbacks */

void GridMap::visTimerCB(const ros::TimerEvent & /*event*/)
{
  publishOccMap();

  // ROS_INFO_THROTTLE(1.0, "No. of Point clouds: %ld", global_map_in_origin_->points.size());
  // ROS_INFO_THROTTLE(1.0, "Octree memory usage: %ld kilobytes", octree_->memoryUsage()/1000);
  // ROS_INFO_STREAM_THROTTLE(1.0, "Octree Bounding Box: " << octree_->getBBXMin() << ", " << octree_->getBBXMax());
}

void GridMap::checkCollisionsTimerCB(const ros::TimerEvent & /*event*/)
{
  // Get nearest obstacle position
  Eigen::Vector3d occ_nearest;
  double dist_to_obs;
  if (!getNearestOccupiedCell(md_.body2origin_.block<3,1>(0,3), occ_nearest, dist_to_obs)){
    return;
  }

  // Publish collision sphere visualizations.
  if (dist_to_obs <= col_warn_radius_){
    publishCollisionSphere(occ_nearest, dist_to_obs, col_fatal_radius_, col_warn_radius_);
  }
}

/** Subscriber callbacks */

void GridMap::odomColCheckCB(const nav_msgs::OdometryConstPtr &msg_odom) 
{
  getCamToGlobalPose(msg_odom->pose.pose);
}

void GridMap::depthOdomCB( const sensor_msgs::ImageConstPtr &msg_img, 
                          const nav_msgs::OdometryConstPtr &msg_odom) 
{
  getCamToGlobalPose(msg_odom->pose.pose);
  depthToCloudMap(msg_img);
}

void GridMap::depthPoseCB( const sensor_msgs::ImageConstPtr &msg_img,
                            const geometry_msgs::PoseStampedConstPtr &msg_pose)
{
  getCamToGlobalPose(msg_pose->pose);
  depthToCloudMap(msg_img);
}

void GridMap::cloudOdomCB( const sensor_msgs::PointCloud2ConstPtr &msg_pc, 
                            const nav_msgs::OdometryConstPtr &msg_odom)
{
  getCamToGlobalPose(msg_odom->pose.pose);
  pcdMsgToMap(*msg_pc);
}

void GridMap::cloudPoseCB( const sensor_msgs::PointCloud2ConstPtr &msg_pc,
                            const geometry_msgs::PoseStampedConstPtr &msg_pose)
{
  getCamToGlobalPose(msg_pose->pose);
  pcdMsgToMap(*msg_pc);
}

void GridMap::cloudTFCB( const sensor_msgs::PointCloud2ConstPtr &msg_pc) 
{
  geometry_msgs::TransformStamped cam_to_origin_tf;

  sensor_msgs::PointCloud2 pc_out;
  try
  {
    // Get camera to origin frame transform
    cam_to_origin_tf = tfBuffer_.lookupTransform(mp_.uav_origin_frame_, mp_.cam_frame_, ros::Time(0));
  }
  catch (const tf2::TransformException &ex)
  {
    ROS_ERROR_THROTTLE(1, 
        "[Gridmap]: Error in lookupTransform of %s in %s", mp_.cam_frame_.c_str(), mp_.uav_origin_frame_.c_str());
    ROS_WARN_THROTTLE(1, "%s",ex.what());
    md_.has_pose_ = false;
    return;
  }

  // Body of uav to global frame
  md_.cam2origin_.block<3, 3>(0, 0) = Eigen::Quaterniond(cam_to_origin_tf.transform.rotation.w,
                                        cam_to_origin_tf.transform.rotation.x,
                                        cam_to_origin_tf.transform.rotation.y,
                                        cam_to_origin_tf.transform.rotation.z).toRotationMatrix();
  md_.cam2origin_.col(3) << cam_to_origin_tf.transform.translation.x, cam_to_origin_tf.transform.translation.y, cam_to_origin_tf.transform.translation.z;

  md_.has_pose_ = true;

  pcdMsgToMap(*msg_pc);
}

void GridMap::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg) 
{
  camera_info_sub_.shutdown();

  // Intrinsic camera matrix for the raw (distorted) images.
  //     [fx  0 cx]
  // K = [ 0 fy cy]
  //     [ 0  0  1]
  mp_.fx_ = msg->K[0];
  mp_.fy_ = msg->K[4];
  mp_.cx_ = msg->K[2];
  mp_.cy_ = msg->K[5];

  mp_.fx_inv_ = 1.0 / msg->K[0];
  mp_.fy_inv_ = 1.0 / msg->K[4];

}

/* Gridmap conversion methods */

void GridMap::updateLocalMap(){
  if (!isPoseValid()){
    return;
  }
  // #define ENV_BUILDER_OCC 100
  // #define ENV_BUILDER_FREE 0
  // #define ENV_BUILDER_UNK -1
  
  // std::cout << "mp_.local_map_origin_: " << mp_.local_map_origin_ .transpose() << std::endl; 
  // std::cout << "mp_.local_map_max_: " << mp_.local_map_max_ .transpose() << std::endl; 

  // Update local map origin based on current UAV position
  mp_.local_map_origin_ = Eigen::Vector3d(
    md_.body2origin_.block<3,1>(0,3)(0) - (mp_.local_map_size_(0) / 2.0), 
    md_.body2origin_.block<3,1>(0,3)(1) - (mp_.local_map_size_(1) / 2.0), 
    mp_.ground_height_);
  // Update local map max position based on current UAV position
  mp_.local_map_max_ = Eigen::Vector3d(
    md_.body2origin_.block<3,1>(0,3)(0) + (mp_.local_map_size_(0) / 2.0), 
    md_.body2origin_.block<3,1>(0,3)(1) + (mp_.local_map_size_(1) / 2.0), 
    mp_.ground_height_ + mp_.local_map_size_(2) );

  // In voxel space, everything is relative to the mp_.local_map_origin_
  // local_map_data_: relative to local_map_origin_
  local_map_data_.clear();
  local_map_data_.resize( mp_.local_map_num_voxels_(0) 
                          * mp_.local_map_num_voxels_(1) 
                          * mp_.local_map_num_voxels_(2), 0);
  
  // Get all occupied coordinates 
  std::vector<Bonxai::CoordT> occ_coords;
  bonxai_map_->getOccupiedVoxels(occ_coords);

  for (auto& coord : occ_coords) // For each occupied coordinate
  {
    // obs_gbl_pos: global obstacle pos
    // Check if the obstacle within local map bounds
    Bonxai::Point3D obs_gbl_pos_pt3d = bonxai_map_->grid().coordToPos(coord);
    // Eigen::Vector3d obs_gbl_pos = Eigen::Vector3d(obs_gbl_pos_pt3d.x, obs_gbl_pos_pt3d.y, obs_gbl_pos_pt3d.z)
    //                               - md_.body2origin_.block<3,1>(0,3);
    Eigen::Vector3d obs_gbl_pos(obs_gbl_pos_pt3d.x, obs_gbl_pos_pt3d.y, obs_gbl_pos_pt3d.z);
    if (!isInLocalMap(obs_gbl_pos)){
      continue;
    }

    // Inflate voxel

    // Convert to voxel index. This is relative to mp_.local_map_origin_.
    Eigen::Vector3i vox_idx_3d = ((obs_gbl_pos - getLocalOrigin()) / getRes() - Eigen::Vector3d::Constant(0.5) ).cast<int>() ; 

    // Inflate voxel by inflation distance
    for(int x = vox_idx_3d(0) - mp_.inf_num_voxels_; x <= vox_idx_3d(0) + mp_.inf_num_voxels_; x++)
    {
      for(int y = vox_idx_3d(1) - mp_.inf_num_voxels_; y <= vox_idx_3d(1) + mp_.inf_num_voxels_; y++)
      {
        for(int z = vox_idx_3d(2) - mp_.inf_num_voxels_; z <= vox_idx_3d(2) + mp_.inf_num_voxels_; z++)
        {
          size_t vox_idx_1d =   x
                              + y * mp_.local_map_num_voxels_(0) 
                              + z * mp_.local_map_num_voxels_(0) * mp_.local_map_num_voxels_(1);

          if (vox_idx_1d >= local_map_data_.size() || vox_idx_1d < 0) {
            continue; // Exceeded map size
          }

          local_map_data_[vox_idx_1d] = 100;
        }
      }
    }

  }
}

void GridMap::getCamToGlobalPose(const geometry_msgs::Pose &pose)
{
  // Transform camera frame to that of the uav
  Eigen::Quaterniond body_q = Eigen::Quaterniond(pose.orientation.w,
                                                pose.orientation.x,
                                                pose.orientation.y,
                                                pose.orientation.z);
  // UAV body to global frame
  md_.body2origin_.block<3, 3>(0, 0) = body_q.toRotationMatrix();
  // md_.body2origin_.col(3) << pose.position.x, pose.position.y, pose.position.z, 1.0;
  md_.body2origin_.block<3,1>(0,3) << pose.position.x, pose.position.y, pose.position.z;

  // Converts camera to UAV origin frame
  md_.cam2origin_ = md_.body2origin_ * md_.cam2body_;

  md_.has_pose_ = true;
}

void GridMap::pcdMsgToMap(const sensor_msgs::PointCloud2 &msg)
{
  /* Bonxai*/

  // Input Point cloud is assumed to be in frame id of the sensor
  if (!isPoseValid()){
    ROS_ERROR("invalid pose");
    return;
  }

  // Remove anything outside of local map bounds
  // updateLocalMap();

  if (msg.data.empty()){
    ROS_WARN_THROTTLE(1.0, "[grid_map]: Empty point cloud received");
    // return;
  }

  std::cout << "before fromROSMsg" << std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcd;
  pcd.reset(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::fromROSMsg(msg, *pcd);

  pcdToMap(pcd);
}

void GridMap::pcdToMap(pcl::PointCloud<pcl::PointXYZ>::Ptr pcd)
{
  // Transform point cloud from camera frame to uav origin frame (the global reference frame)
  pcl::transformPointCloud(*pcd, *pcd, md_.cam2origin_);

  // Getting the Translation from the sensor to the Global Reference Frame
  const pcl::PointXYZ sensor_origin(md_.cam2origin_(0, 3), md_.cam2origin_(1, 3), md_.cam2origin_(2, 3));

  ROS_INFO("[grid_map] Size of point clouds: %ld", pcd->points.size());

  // Save point cloud to global map origin
  std::cout << "before global_map_in_origin_ = pcd" << std::endl;

  global_map_in_origin_ = pcd;
  global_map_in_origin_->header.frame_id = mp_.uav_origin_frame_;

  auto bonxai_start = std::chrono::high_resolution_clock::now();
  bonxai_map_->insertPointCloud(global_map_in_origin_->points, sensor_origin, 30.0);
  auto bonxai_end = std::chrono::high_resolution_clock::now();

  auto kdtree_set_input_cloud_start = std::chrono::high_resolution_clock::now();
  kdtree_->Build((*global_map_in_origin_).points);

  auto kdtree_set_input_cloud_end = std::chrono::high_resolution_clock::now();


  ROS_INFO("[grid_map] kdtree_set_input_cloud duration: %f", std::chrono::duration_cast<std::chrono::duration<double>>(
        kdtree_set_input_cloud_end - kdtree_set_input_cloud_start).count() * 1000.0);
  ROS_INFO("[grid_map] bonxai duration: %f", std::chrono::duration_cast<std::chrono::duration<double>>(
        bonxai_end - bonxai_start).count() * 1000.0);

  ROS_INFO("[grid_map] Completed pcdToMap");
}

/* Checks */

bool GridMap::isPoseValid() {
  if (!md_.has_pose_){
    ROS_ERROR("[%s] No pose/odom received", node_name_.c_str());
    return false;
  }

  if (isnan(md_.cam2origin_.col(3)(0)) || isnan(md_.cam2origin_.col(3)(1)) || isnan(md_.cam2origin_.col(3)(2))){
    ROS_ERROR("[%s] Camera pose has NAN value", node_name_.c_str());
    return false;
  }

  if (!isInGlobalMap(md_.cam2origin_.block<3,1>(0,3)))
  {
    ROS_ERROR("[%s] Camera pose (%.2f, %.2f, %.2f) is not within map boundary", 
      node_name_.c_str(), md_.cam2origin_.col(3)(0), md_.cam2origin_.col(3)(1), md_.cam2origin_.col(3)(2));
    return false;
  }

  return true;
}

/** Publishers */

void GridMap::publishOccMap()
{
  if (occ_map_pub_.getNumSubscribers() == 0){
    return;
  }

  /* Bonxai */

  thread_local std::vector<Eigen::Vector3d> bonxai_result;
  bonxai_result.clear();
  bonxai_map_->getOccupiedVoxels(bonxai_result);

  if (bonxai_result.size() <= 1)
  {
    // ROS_WARN("[grid_map] Nothing to publish, bonxai is empty");
    return;
  }

  thread_local pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl_cloud.clear();

  for (const auto& voxel : bonxai_result)
  {
    pcl_cloud.push_back(pcl::PointXYZ(voxel.x(), voxel.y(), voxel.z()));
  }
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(pcl_cloud, cloud_msg);

  cloud_msg.header.frame_id = mp_.uav_origin_frame_;
  cloud_msg.header.stamp = ros::Time::now();
  occ_map_pub_.publish(cloud_msg);
  // ROS_INFO("Published occupancy grid with %ld voxels", pcl_cloud.points.size());
}

void GridMap::publishCollisionSphere(
  const Eigen::Vector3d &pos, const double& dist_to_obs, 
  const double& fatal_radius, const double& warn_radius)
{
  static int col_viz_id = 0;

  visualization_msgs::Marker sphere;
  sphere.header.frame_id = mp_.uav_origin_frame_;
  sphere.header.stamp = ros::Time::now();
  sphere.type = visualization_msgs::Marker::SPHERE;
  sphere.action = visualization_msgs::Marker::ADD;
  sphere.ns = "collision_viz";
  sphere.pose.orientation.w = 1.0;
  sphere.id = col_viz_id++;

  // Make the alpha and red color value scale from 0.0 to 1.0 depending on the distance to the obstacle. 
  // With the upper limit being the warn_radius, and the lower limit being the fatal_radius
  double fatal_ratio = std::clamp((warn_radius - dist_to_obs)/(warn_radius - fatal_radius), 0.0, 1.001);
  
  if (fatal_ratio >= 1.0){
    sphere.color.r = 1.0;
    sphere.color.g = 0.0;
    sphere.color.b = 0.0; 
    sphere.color.a = 0.8;
  }
  else {
    // Goes from blue to purple
    sphere.color.r = fatal_ratio*(1.0);
    sphere.color.g = 0.0;
    sphere.color.b = 1.0; // If fatal, make blue value 0.0, so sphere is entire red.
    sphere.color.a = 0.3 + (fatal_ratio*(0.8-0.3));
  }

  double scale = fatal_ratio < 1.0 ? 0.35 : 0.6;

  sphere.scale.x = scale;
  sphere.scale.y = scale;
  sphere.scale.z = scale;
  sphere.pose.position.x = pos(0);
  sphere.pose.position.y = pos(1);
  sphere.pose.position.z = pos(2);

  collision_viz_pub_.publish(sphere);
}

