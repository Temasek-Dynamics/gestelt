#include "grid_map/grid_map.h"

/** Initialization methods */

void GridMap::initMap(pcl::PointCloud<pcl::PointXYZ>::Ptr pcd, const Eigen::Vector3d& map_size, const double& inflation, const double& resolution)
{
  md_.has_pose_ = true;
  mp_.global_map_size_ = map_size;
  mp_.inflation_ = inflation;
  mp_.uav_origin_frame_ = "world";
  mp_.map_origin_ = Eigen::Vector3d(mp_.global_map_size_(0) / 2.0, mp_.global_map_size_(1) / 2.0, 0.0);
  mp_.resolution_ = resolution;

  reset(mp_.resolution_);

  pcdToMap(pcd);
}

void GridMap::initMapROS(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{  
  std::cout << "initMapROS" << std::endl;
  readROSParams(nh, pnh);

  std::cout << "read ros params" << std::endl;

  reset(mp_.resolution_);

  Eigen::Matrix3d rot_mat = (Eigen::AngleAxisd((M_PI/180.0) * md_.cam2body_rpy_deg(0), Eigen::Vector3d::UnitX())
                              * Eigen::AngleAxisd((M_PI/180.0) * md_.cam2body_rpy_deg(1), Eigen::Vector3d::UnitY())
                              * Eigen::AngleAxisd((M_PI/180.0) * md_.cam2body_rpy_deg(2), Eigen::Vector3d::UnitZ())).toRotationMatrix();

  md_.cam2body_.block<3, 3>(0, 0) =  (Eigen::AngleAxisd((M_PI/180.0) * md_.cam2body_rpy_deg(0), Eigen::Vector3d::UnitX())
                                      * Eigen::AngleAxisd((M_PI/180.0) * md_.cam2body_rpy_deg(1), Eigen::Vector3d::UnitY())
                                      * Eigen::AngleAxisd((M_PI/180.0) * md_.cam2body_rpy_deg(2), Eigen::Vector3d::UnitZ())).toRotationMatrix();
  mp_.map_origin_ = Eigen::Vector3d(-mp_.global_map_size_(0) / 2.0, -mp_.global_map_size_(1) / 2.0, mp_.ground_height_);

  std::cout << "before dbg_input_entire_map_" << std::endl;

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

    /* Initialize ROS Timers */
    vis_timer_ = nh.createTimer(ros::Duration(0.1), &GridMap::visTimerCB, this);
  }
  else {
    initROSPubSubTimers(nh, pnh);
  }

  ROS_INFO("[%s] Map origin (%f, %f, %f)", node_name_.c_str(), 
    mp_.map_origin_(0), mp_.map_origin_(1), mp_.map_origin_(2));
  ROS_INFO("[%s] Global map size (%f, %f, %f)", node_name_.c_str(), 
    mp_.global_map_size_(0), mp_.global_map_size_(1), mp_.global_map_size_(2));
  // ROS_INFO("[%s] Local map size (%f, %f, %f)", node_name_.c_str(), 
  //   mp_.local_map_size_(0), mp_.local_map_size_(1), mp_.local_map_size_(2));
  ROS_INFO("[%s] Resolution %f m, Inflation %f m", node_name_.c_str(), 
    mp_.resolution_, mp_.inflation_);
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

  // Set up octree data structure
  // octree_ = std::make_shared<octomap::OcTree>(mp_.resolution_);
  // octomap::point3d bbx_min(-mp_.global_map_size_(0), -mp_.global_map_size_(1), -mp_.global_map_size_(2));
  // octomap::point3d bbx_max(mp_.global_map_size_(0), mp_.global_map_size_(1), mp_.global_map_size_(2));
  // octree_->setBBXMin(bbx_min);
  // octree_->setBBXMax(bbx_max);
  // octree_->useBBXLimit(true);

  // Set up Bonxai data structure
  bonxai_map_ = std::make_unique<BonxaiT>(resolution);

  // Set up kdtree for generating safe flight corridors
  kdtree_ = std::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ>>(); // KD-Tree 
}

/** Timer callbacks */

void GridMap::visTimerCB(const ros::TimerEvent & /*event*/)
{
  publishMap();
  // ROS_INFO_THROTTLE(1.0, "No. of Point clouds: %ld", global_map_in_origin_->points.size());
  // ROS_INFO_THROTTLE(1.0, "Octree memory usage: %ld kilobytes", octree_->memoryUsage()/1000);
  // ROS_INFO_STREAM_THROTTLE(1.0, "Octree Bounding Box: " << octree_->getBBXMin() << ", " << octree_->getBBXMax());
}


/** Subscriber callbacks */

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
  
  // Update bounds of local map 
  md_.local_map_min_ << -mp_.local_map_size_(0) + md_.cam2origin_.col(3)(0), 
                        -mp_.local_map_size_(1) + md_.cam2origin_.col(3)(1),
                        -mp_.local_map_size_(2) + md_.cam2origin_.col(3)(2);

  md_.local_map_max_ << mp_.local_map_size_(0) + md_.cam2origin_.col(3)(0), 
                        mp_.local_map_size_(1) + md_.cam2origin_.col(3)(1),
                        mp_.local_map_size_(2) + md_.cam2origin_.col(3)(2);

  // Update bounds of octomap
  // if (!mp_.keep_global_map_){
    // octomap::point3d bbx_min(md_.local_map_min_(0), md_.local_map_min_(1), md_.local_map_min_(2));
    // octomap::point3d bbx_max(md_.local_map_max_(0), md_.local_map_max_(1), md_.local_map_max_(2));
    // octree_->setBBXMin(bbx_min);
    // octree_->setBBXMax(bbx_max);
  // }
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
  md_.body2origin_.col(3) << pose.position.x, pose.position.y, pose.position.z;

  // Converts camera to UAV origin frame
  md_.cam2origin_ = md_.body2origin_ * md_.cam2body_;

  md_.has_pose_ = true;
}

void GridMap::pcdMsgToMap(const sensor_msgs::PointCloud2 &msg)
{
  /* Octomap*/

  // // Input Point cloud is assumed to be in frame id of the sensor
  // if (!isPoseValid()){
  //   ROS_INFO("invalid pose");
  //   return;
  // }

  // // Remove anything outside of local map bounds
  // updateLocalMap();

  // ROS_INFO("[grid_map] Size of data: %ld", msg.data.size());

  // if (msg.data.empty()){
  //   ROS_WARN_THROTTLE(1.0, "[grid_map]: Empty point cloud received");
  //   // return;
  // }


  // pcl::fromROSMsg(msg, *global_map_in_origin_);

  // // Transform point cloud from camera frame to uav origin frame
  // pcl::transformPointCloud (*global_map_in_origin_, *global_map_in_origin_, md_.cam2origin_);

  // ROS_INFO("After transformPointCloud");

  // // Downsample point cloud
  // // if (mp_.downsample_cloud_){
  // //   vox_grid_filter_.setInputCloud(cloud);
  // //   vox_grid_filter_.filter(*cloud);
  // // }

  // ROS_INFO("Before pclToOctomapPC");

  // octomap::Pointcloud octomap_cloud;
  // pclToOctomapPC(global_map_in_origin_, octomap_cloud);

  // ROS_INFO("[grid_map] Size of octree data: %ld", octomap_cloud.size());

  // ROS_INFO("After pclToOctomapPC");

  // // octomap::point3d sensor_origin(md_.cam2origin_(0, 3), md_.cam2origin_(1, 3), md_.cam2origin_(2, 3));
  // // octree_->insertPointCloud(octomap_cloud, sensor_origin, mp_.max_range);

  // octomap::point3d sensor_origin(0.0, 0.0, 0.0);
  // octree_->insertPointCloud(octomap_cloud, sensor_origin);

  // ROS_INFO("After insertPointCloud");

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
  kdtree_->setInputCloud(global_map_in_origin_);
  auto kdtree_set_input_cloud_end = std::chrono::high_resolution_clock::now();

  std::cout << "kdtree_set_input_cloud duration: " << std::chrono::duration_cast<std::chrono::duration<double>>(
        kdtree_set_input_cloud_end - kdtree_set_input_cloud_start).count() << std::endl;
  std::cout << "bonxai duration: " << std::chrono::duration_cast<std::chrono::duration<double>>(
        bonxai_end - bonxai_start).count() << std::endl;

  ROS_INFO("[grid_map] Completed pcdToMap");
}

void GridMap::pclToOctomapPC(const pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud , octomap::Pointcloud& octomap_cloud) {
  // Iterate through all the points in the point cloud 
  for (size_t i = 0; i < pcl_cloud->points.size(); i++){
    octomap::point3d endpoint(pcl_cloud->points[i].x, pcl_cloud->points[i].y, pcl_cloud->points[i].z);
    octomap_cloud.push_back(endpoint);
  }
}

void GridMap::octreeToPclPC(std::shared_ptr<octomap::OcTree> tree, pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud) {
  std::vector<octomap::point3d> pcl;
  pcl_cloud->points.clear();
  for (octomap::OcTree::iterator it = tree->begin(); it != tree->end(); ++it)
  {
    if (it == NULL){
      continue;
    }
    if(tree->isNodeOccupied(*it))
    {
      octomap::point3d occ_point = it.getCoordinate();
      pcl::PointXYZ pcl_occ_point(occ_point(0), occ_point(1), occ_point(2));
      pcl_cloud->push_back(pcl_occ_point);
    }
  }
  
  pcl_cloud->height = 1;
  pcl_cloud->width = pcl_cloud->points.size();
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

void GridMap::publishMap()
{
  if (occ_map_pub_.getNumSubscribers() == 0){
    return;
  }

  /* Octree */

  // thread_local pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  // pcl_cloud.clear();

  // octreeToPclPC(octree_, pcl_cloud);

  // sensor_msgs::PointCloud2 cloud_msg;
  // pcl::toROSMsg(*pcl_cloud, cloud_msg);
  // cloud_msg.header.frame_id = mp_.uav_origin_frame_;
  // cloud_msg.header.stamp = ros::Time::now();

  // occ_map_pub_.publish(cloud_msg);

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

/** Gridmap operations */

bool GridMap::getOccupancy(const Eigen::Vector3d &pos)
{

  /* Octree */

  // // If not in map or not in octree bounding box. return -1 
  // if (!isInGlobalMap(pos)){
  //   return -1;
  // }

  // octomap::point3d octo_pos(pos(0), pos(1), pos(2));
  // octomap::OcTreeNode* node = octree_->search(octo_pos);
  // if (node == NULL){
  //   // Return 0 if node is not occupied
  //   return 0;
  // }
  // if (octree_->isNodeOccupied(node)){
  //   return 1;
  // }

  // // Return 0 if node is not occupied
  // return 0;

  /* Bonxai */

  // If not in map or not in octree bounding box. return -1 
  if (!isInGlobalMap(pos)){
    return true;
  }

  Bonxai::CoordT coord = bonxai_map_->grid().posToCoord(pos(0), pos(1), pos(2));

  return bonxai_map_->isOccupied(coord);


}

bool GridMap::getInflateOccupancy(const Eigen::Vector3d &pos)
{
  /* Octree */

  // if (!isInGlobalMap(pos)){
  //   return -1;
  // }

  // // Search inflated space of given position
  // for(float x = pos(0) - mp_.inflation_; x <= pos(0) + mp_.inflation_; x += mp_.resolution_){
  //   for(float y = pos(1) - mp_.inflation_; y <= pos(1) + mp_.inflation_; y += mp_.resolution_){
  //     for(float z = pos(2) - mp_.inflation_; z <= pos(2) + mp_.inflation_; z += mp_.resolution_){
  //       octomap::OcTreeNode* node = octree_->search(x, y, z);
  //       if (node == NULL){
  //         // Return 0 if node is not occupied
  //         return 0;
  //       }
  //       if (octree_->isNodeOccupied(node)){
  //         return 1;
  //       }
  //     }
  //   }
  // }

  // // Return 0 if node is not occupied
  // return 0;

  /* Bonxai */

  // if (!isInGlobalMap(pos)){
  //   return true;
  // }

  // for(float x = pos(0) - mp_.inflation_; x <= pos(0) + mp_.inflation_; x += mp_.resolution_)
  // {
  //   for(float y = pos(1) - mp_.inflation_; y <= pos(1) + mp_.inflation_; y += mp_.resolution_)
  //   {
  //     for(float z = pos(2) - mp_.inflation_; z <= pos(2) + mp_.inflation_; z += mp_.resolution_)
  //     {
  //       Bonxai::CoordT coord = bonxai_map_->grid().posToCoord(x, y, z);
  //       if (bonxai_map_->isOccupied(coord)){
  //         return true;
  //       }
  //     }
  //   }
  // }
  // Return 0 if node is not occupied
  // return false;

  /* Using KDTree to check for inflation */

  if (!isInGlobalMap(pos)){
    return true;
  }

  return withinObsRadius(pos, mp_.inflation_);
}

bool GridMap::isInGlobalMap(const Eigen::Vector3d &pos)
{
  if (pos(0) <= -mp_.global_map_size_(0)/2 || pos(0) >= mp_.global_map_size_(0)/2
    || pos(1) <= -mp_.global_map_size_(1)/2 || pos(1) >= mp_.global_map_size_(1)/2
    || pos(2) <= -mp_.global_map_size_(2)/2 || pos(2) >= mp_.global_map_size_(2)/2)
  {
    return false;
  }

  return true;
}

bool GridMap::isInLocalMap(const Eigen::Vector3d &pos)
{
  if (pos(0) <= md_.local_map_min_(0) || pos(0) >= md_.local_map_max_(0)
    || pos(1) <= md_.local_map_min_(1) || pos(1) >= md_.local_map_max_(1)
    || pos(2) <= md_.local_map_min_(2) || pos(2) >= md_.local_map_max_(2))
  {
    return false;
  }

  return true;
}

bool GridMap::withinObsRadius(const Eigen::Vector3d &pos, const double& radius)
{
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

  pcl::PointXYZ searchPoint(pos(0), pos(1), pos(2));

  return kdtree_->radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ;
}

bool GridMap::getNearestOccupiedCell(const Eigen::Vector3d &pos, Eigen::Vector3d& occ_nearest, double& radius){
  int K = 1;

  std::vector<int> pointIdxKNNSearch(K);
  std::vector<float> pointKNNSquaredDistance(K);

  pcl::PointXYZ searchPoint(pos(0), pos(1), pos(2));

  if ( kdtree_->nearestKSearch (searchPoint, K, pointIdxKNNSearch, pointKNNSquaredDistance) > 0 )
  {
    occ_nearest = Eigen::Vector3d{(*global_map_in_origin_)[ pointIdxKNNSearch[0]].x, 
                                  (*global_map_in_origin_)[ pointIdxKNNSearch[0]].y, 
                                  (*global_map_in_origin_)[ pointIdxKNNSearch[0]].z};
    radius = sqrt(pointKNNSquaredDistance[0]);

    return true;
  }

  return false;
}


void GridMap::depthToCloudMap(const sensor_msgs::ImageConstPtr &msg)
{
  // // TODO: Still necessary?
  // if (!isPoseValid()){
  //   return;
  // }

  // /* get depth image */
  // cv_bridge::CvImagePtr cv_ptr;
  // cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);

  // if (msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  // {
  //   (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
  // }
  // cv_ptr->image.copyTo(md_.depth_image_);

  // // pcl header.stamp must have the timestamp in microseconds or TF will not
  // // be able to resolve the transformation
  // local_map_in_origin_->header.stamp = msg->header.stamp.toNSec() * 1e-3;

  // local_map_in_origin_->height = 1;
  // local_map_in_origin_->width = (md_.depth_image_.cols * md_.depth_image_.rows) / (mp_.depth_stride_ * mp_.depth_stride_);

  // local_map_in_origin_->points.resize(local_map_in_origin_->width);
  // local_map_in_origin_->is_dense = true;

  // uint16_t *row_ptr;

  // int cloud_idx = 0;

  // for (int v = 0; v < md_.depth_image_.rows; v+=mp_.depth_stride_){
  //   row_ptr = md_.depth_image_.ptr<uint16_t>(v);
  //   for (int u = 0; u < md_.depth_image_.cols; u+=mp_.depth_stride_, row_ptr+=mp_.depth_stride_, cloud_idx++){

  //     pcl::PointXYZ& pt = local_map_in_origin_->points[cloud_idx];

  //     float bad_point = std::numeric_limits<float>::quiet_NaN ();

  //     if (*row_ptr == 0 || *row_ptr == bad_point || *row_ptr == 255){
  //       pt.x = pt.y = pt.z = bad_point;
  //       // local_map_in_origin_->is_dense = false;
  //     }
  //     else{
  //       pt.z = (*row_ptr) / 1000.0;
  //       pt.x = (u - mp_.cx_) * pt.z * mp_.fx_inv_;
  //       pt.y = (v - mp_.cy_) * pt.z * mp_.fy_inv_;
  //     }
    
  //   }
  // }

  // // Downsample point cloud
  // if (mp_.downsample_cloud_){
  //   vox_grid_filter_.setInputCloud(local_map_in_origin_);
  //   vox_grid_filter_.filter(*local_map_in_origin_);
  // }

  // pcl::transformPointCloud(*local_map_in_origin_, *local_map_in_origin_, md_.cam2origin_);
}

