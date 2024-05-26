#include "grid_map/grid_map.h"

/** Initialization methods */

void GridMap::initMapROS(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{  
  readROSParams(nh, pnh);

  reset(mp_.resolution_);

  if (check_collisions_){ // True if we want to publish collision visualizations between the agent and static obstacles
    odom_col_check_sub_ = nh.subscribe<nav_msgs::Odometry>(
      "odom", 1, &GridMap::odomColCheckCB, this);
    // Publisher for collision visualizations
    collision_viz_pub_ = nh.advertise<visualization_msgs::Marker>("grid_map/collision_viz", 10);
    check_collisions_timer_ = 
      nh.createTimer(ros::Duration(0.025), &GridMap::checkCollisionsTimerCB, this);
    pnh.param("grid_map/collision_check/warn_radius", col_warn_radius_, -1.0); // 0.25
    pnh.param("grid_map/collision_check/fatal_radius", col_fatal_radius_, -1.0); // 0.126
  }

  if (dbg_input_entire_map_){
    ROS_INFO("[%s] DEBUG: INPUT ENTIRE MAP", node_name_.c_str());

    md_.has_pose_ = true;
    
    // Wait for entire point cloud map
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

    occ_map_pub_ = nh.advertise<sensor_msgs::PointCloud2>("grid_map/occupancy", 10);
    local_map_poly_pub_ = nh.advertise<geometry_msgs::PolygonStamped>("local_map/bounds", 5);
    // update_local_map_timer_ = nh.createTimer(ros::Duration(1.0/update_local_map_freq_), &GridMap::updateLocalMapTimerCB, this);
    updateLocalMap();

    return;
  }
  else {
    initROSPubSubTimers(nh, pnh);
  }

  // Initialize publisher for occupancy map
  occ_map_pub_ = nh.advertise<sensor_msgs::PointCloud2>("grid_map/occupancy", 10);
  local_map_poly_pub_ = nh.advertise<geometry_msgs::PolygonStamped>("local_map/bounds", 5);

  /* Initialize ROS Timers */
  // vis_occ_timer_ = nh.createTimer(ros::Duration(1.0/viz_occ_map_freq_), &GridMap::visTimerCB, this);
  update_local_map_timer_ = nh.createTimer(ros::Duration(1.0/update_local_map_freq_), &GridMap::updateLocalMapTimerCB, this);
  // build_kd_tree_timer_ = nh.createTimer(ros::Duration(1.0/build_kd_tree_freq_), &GridMap::buildKDTreeTimerCB, this);

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
  cloud_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(
    nh, "grid_map/cloud", 5, ros::TransportHints().tcpNoDelay()));

  // From pose type, determine what type of localization message to subscribe to
  if (mp_.pose_type_ == PoseType::POSE_STAMPED) // Pose
  {
    ROS_INFO("[%s] Pose as camera pose input", node_name_.c_str());
    pose_sub_.reset(new message_filters::Subscriber<geometry_msgs::PoseStamped>(
      nh, "grid_map/pose", 5, ros::TransportHints().udp()));
    sync_cloud_pose_.reset(new message_filters::Synchronizer<SyncPolicyCloudPose>(
        SyncPolicyCloudPose(5), *cloud_sub_, *pose_sub_));
    sync_cloud_pose_->registerCallback(boost::bind(&GridMap::cloudPoseCB, this, _1, _2));
  }
  else if (mp_.pose_type_ == PoseType::ODOMETRY) // Odom
  {
    ROS_INFO("[%s] Odometry as camera pose input", node_name_.c_str());
    odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(
      nh, "grid_map/odom", 5, ros::TransportHints().udp()));

    sync_cloud_odom_.reset(new message_filters::Synchronizer<SyncPolicyCloudOdom>(
        SyncPolicyCloudOdom(5), *cloud_sub_, *odom_sub_));
    sync_cloud_odom_->registerCallback(boost::bind(&GridMap::cloudOdomCB, this, _1, _2));
  }
  else if (mp_.pose_type_ == PoseType::TF) // TF
  {
    ROS_INFO("[%s] TF as camera pose input", node_name_.c_str());
    tfListener_.reset(new tf2_ros::TransformListener(tfBuffer_));
    tf_cloud_filter_.reset(new tf2_ros::MessageFilter<sensor_msgs::PointCloud2>(*cloud_sub_, tfBuffer_, mp_.uav_origin_frame_, 5, 0));

    tf_cloud_filter_->registerCallback( boost::bind(&GridMap::cloudTFCB, this, _1) );
  }
  else { // UNKNOWN
    ROS_ERROR("[%s] Unknown POSE_TYPE", node_name_.c_str());
    ros::shutdown();
  }

}

void GridMap::readROSParams(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
  /* Map parameters */
  pnh.param("grid_map/debug_input_entire_map", dbg_input_entire_map_, false);
  pnh.param("grid_map/entire_pcd_map_topic", entire_pcd_map_topic_, std::string("/fake_map"));

  pnh.param("grid_map/global_map/size_x", mp_.global_map_size_(0), -1.0);
  pnh.param("grid_map/global_map/size_y", mp_.global_map_size_(1), -1.0);
  pnh.param("grid_map/global_map/size_z", mp_.global_map_size_(2), -1.0);

  pnh.param("grid_map/local_map/size_x", mp_.local_map_size_(0), -1.0);
  pnh.param("grid_map/local_map/size_y", mp_.local_map_size_(1), -1.0);
  pnh.param("grid_map/local_map/size_z", mp_.local_map_size_(2), -1.0);

  pnh.param("grid_map/local_map/update_frequency", update_local_map_freq_, -1.0);
  pnh.param("grid_map/local_map/viz_frequency", viz_occ_map_freq_, -1.0);
  // pnh.param("grid_map/local_map/build_kdtree_frequency", build_kd_tree_freq_, -1.0);

  // pnh.param("grid_map/keep_global_map", mp_.keep_global_map_, false);

  pnh.param("grid_map/occ_map/resolution", mp_.resolution_, -1.0);
  pnh.param("grid_map/occ_map/inflation", mp_.inflation_, -1.0);
  pnh.param("grid_map/occ_map/max_range", mp_.max_range, -1.0);
  pnh.param("grid_map/occ_map/ground_height", mp_.ground_height_, 0.0);

  mp_.inf_num_voxels_ = std::ceil(mp_.inflation_/mp_.resolution_);

  /* Point cloud filter */
  // pnh.param("grid_map/filter/depth_stride", mp_.depth_stride_, 4);
  // pnh.param("grid_map/filter/downsample_cloud", mp_.downsample_cloud_, false);
  // pnh.param("grid_map/filter/voxel_size", mp_.voxel_size_, -1.0);

  /* Sensor inputs */
  pnh.param("grid_map/pose_type", mp_.pose_type_, 1);

  pnh.param("grid_map/cam_frame", mp_.cam_frame_, std::string("cam_link"));
  pnh.param("grid_map/global_frame", mp_.global_frame_, std::string("world"));
  pnh.param("grid_map/uav_origin_frame", mp_.uav_origin_frame_, std::string("map"));

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
  // KD_TREE(float delete_param = 0.5, float balance_param = 0.6 , float box_length = 0.2);
  kdtree_ = std::make_shared<KD_TREE<pcl::PointXYZ>>(0.5, 0.6, 0.1);

  global_map_in_origin_.reset(new pcl::PointCloud<pcl::PointXYZ>());

  md_.last_sensor_msg_time = ros::Time::now().toSec();

  // Camera to body transform
  md_.cam2body_.block<3, 3>(0, 0) =  (Eigen::AngleAxisd((M_PI/180.0) * md_.cam2body_rpy_deg(2), Eigen::Vector3d::UnitZ())
                                      * Eigen::AngleAxisd((M_PI/180.0) * md_.cam2body_rpy_deg(1), Eigen::Vector3d::UnitY())
                                      * Eigen::AngleAxisd((M_PI/180.0) * md_.cam2body_rpy_deg(0), Eigen::Vector3d::UnitX())).toRotationMatrix();

  // Global map origin is FIXED at a corner of the global map i.e. (-W/2, -L/2, 0)
  mp_.global_map_origin_ = Eigen::Vector3d(
    -mp_.global_map_size_(0) / 2.0, 
    -mp_.global_map_size_(1) / 2.0, 
    mp_.ground_height_);

  /* LOCAL MAP COORDINATES ARE NOT FIXED! */
  // Local map min is at a corner of the local map i.e. (uav_pos_x - local_sz_x /2, uav_pos_y - local_sz_y /2, 0), relative to the current position of the robot
  mp_.local_map_origin_ = Eigen::Vector3d(
    -mp_.local_map_size_(0) / 2.0, 
    -mp_.local_map_size_(1) / 2.0, 
    mp_.ground_height_);

  // Local map max is at the other corner of the local map i.e. (uav_pos_x + local_sz_x /2, uav_pos_y + local_sz_y/2, 0), relative to the current position of the robot
  mp_.local_map_max_ = Eigen::Vector3d(
    mp_.local_map_size_(0) / 2.0, 
    mp_.local_map_size_(1) / 2.0, 
    mp_.ground_height_ + mp_.local_map_size_(2));

  mp_.global_map_num_voxels_ = (mp_.global_map_size_.cwiseProduct(Eigen::Vector3d::Constant(1/mp_.resolution_))).cast<int>();
  mp_.local_map_num_voxels_ = (mp_.local_map_size_.cwiseProduct(Eigen::Vector3d::Constant(1/mp_.resolution_))).cast<int>();

}

/** Timer callbacks */

void GridMap::visTimerCB(const ros::TimerEvent & /*event*/)
{
  // publishOccMap();
  // ROS_INFO_THROTTLE(1.0, "No. of Point clouds: %ld", global_map_in_origin_->points.size());
  // ROS_INFO_THROTTLE(1.0, "Octree memory usage: %ld kilobytes", octree_->memoryUsage()/1000);
  // ROS_INFO_STREAM_THROTTLE(1.0, "Octree Bounding Box: " << octree_->getBBXMin() << ", " << octree_->getBBXMax());
}

// void GridMap::buildKDTreeTimerCB(const ros::TimerEvent & /*event*/)
// {

// }

void GridMap::updateLocalMapTimerCB(const ros::TimerEvent & /*event*/)
{
  updateLocalMap();
}

void GridMap::checkCollisionsTimerCB(const ros::TimerEvent & /*event*/)
{
  if (!isPoseValid()){
    return;
  }

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

void GridMap::cloudOdomCB( const sensor_msgs::PointCloud2ConstPtr &msg_pc, 
                            const nav_msgs::OdometryConstPtr &msg_odom)
{
  getCamToGlobalPose(msg_odom->pose.pose);
  pcdMsgToMap(*msg_pc);
  md_.last_sensor_msg_time = ros::Time::now().toSec();
}

void GridMap::cloudPoseCB( const sensor_msgs::PointCloud2ConstPtr &msg_pc,
                            const geometry_msgs::PoseStampedConstPtr &msg_pose)
{
  getCamToGlobalPose(msg_pose->pose);
  pcdMsgToMap(*msg_pc);
  md_.last_sensor_msg_time = ros::Time::now().toSec();
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

  md_.last_sensor_msg_time = ros::Time::now().toSec();
}

/* Gridmap conversion methods */

void GridMap::updateLocalMap(){
  if (!isPoseValid()){
    return;
  }

  // OCCUPIED VALUE: 100
  // FREE VALUE: 0
  // UNKNOWN VALUE: -1

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

  if (occ_coords.size() <= 1){
    // ROS_WARN("[grid_map] Nothing to publish, bonxai is empty");
    return;
  }

  // std::unique_lock<std::mutex> lck(occ_map_pts_mutex_, std::try_to_lock);
  // if (!lck.owns_lock()){
  //   return;
  // }

  occ_map_pts_.clear();

  for (auto& coord : occ_coords) // For each occupied coordinate
  {
    // obs_gbl_pos: global obstacle pos
    // Check if the obstacle within local map bounds
    Bonxai::Point3D obs_gbl_pos_pt3d = bonxai_map_->grid().coordToPos(coord);
    // Eigen::Vector3d obs_gbl_pos = Eigen::Vector3d(obs_gbl_pos_pt3d.x, obs_gbl_pos_pt3d.y, obs_gbl_pos_pt3d.z)
    //                               - md_.body2origin_.block<3,1>(0,3);
    Eigen::Vector3d obs_gbl_pos(obs_gbl_pos_pt3d.x, obs_gbl_pos_pt3d.y, obs_gbl_pos_pt3d.z);
    if (!isInLocalMap(obs_gbl_pos)){ // Point is outside the local map
      continue;
    }

    occ_map_pts_.push_back(pcl::PointXYZ(obs_gbl_pos_pt3d.x, obs_gbl_pos_pt3d.y, obs_gbl_pos_pt3d.z));

    // Convert to voxel index. This is relative to mp_.local_map_origin_
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

          // Assign as occupied voxel
          local_map_data_[vox_idx_1d] = 100;
        }
      }
    }
  }

  // tm_kdtree_build_.start();
  kdtree_->Build(occ_map_pts_.points);
  // tm_kdtree_build_.stop(false);

  publishLocalMapBounds();

  publishOccMap(occ_map_pts_);
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
    return;
  }

  if (msg.data.empty()){
    // ROS_WARN_THROTTLE(1.0, "[grid_map]: Empty point cloud received");
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr pcd;
  pcd.reset(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::fromROSMsg(msg, *pcd);

  pcdToMap(pcd);


}

void GridMap::pcdToMap(pcl::PointCloud<pcl::PointXYZ>::Ptr pcd)
{
  // Transform point cloud from camera frame to uav origin frame (the global reference frame)
  pcl::transformPointCloud(*pcd, *global_map_in_origin_, md_.cam2origin_);
  global_map_in_origin_->header.frame_id = mp_.uav_origin_frame_;

  // Getting the Translation from the sensor to the Global Reference Frame
  const pcl::PointXYZ sensor_origin(md_.cam2origin_(0, 3), md_.cam2origin_(1, 3), md_.cam2origin_(2, 3));

  // tm_bonxai_insert_.start();
  bonxai_map_->insertPointCloud(global_map_in_origin_->points, sensor_origin, mp_.max_range);
  // tm_bonxai_insert_.stop(false);

  if (dbg_input_entire_map_){
    kdtree_->Build((*global_map_in_origin_).points);
  }

}

/* Checks */

bool GridMap::isPoseValid() {
  if (dbg_input_entire_map_){ // If in debug mode, no need to check for valid camera pose
    return true;
  }

  if (!md_.has_pose_){
    ROS_ERROR_THROTTLE(1.0, "[%s] No pose/odom received", node_name_.c_str());
    return false;
  }

  if (isTimeout(md_.last_sensor_msg_time, 0.25)){
    ROS_ERROR_THROTTLE(1.0, "[%s] Sensor message timeout exceeded 0.25s: (%f, %f)", node_name_.c_str(), md_.last_sensor_msg_time, ros::Time::now().toSec());
    return false;
  }

	if (md_.cam2origin_.array().isNaN().any()){
    ROS_ERROR("[%s] Camera pose has NAN value", node_name_.c_str());
    return false;
	}

  if (!isInGlobalMap(md_.cam2origin_.block<3,1>(0,3)))
  {
    ROS_ERROR("[%s] Camera pose (%.2f, %.2f, %.2f) is not within global map boundary", 
      node_name_.c_str(), md_.cam2origin_.col(3)(0), md_.cam2origin_.col(3)(1), md_.cam2origin_.col(3)(2));
    return false;
  }

  return true;
}

/** Publishers */

void GridMap::publishOccMap(const pcl::PointCloud<pcl::PointXYZ>& occ_map_pts)
{
  if (occ_map_pub_.getNumSubscribers() > 0){
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(occ_map_pts, cloud_msg);

    cloud_msg.header.frame_id = mp_.uav_origin_frame_;
    cloud_msg.header.stamp = ros::Time::now();
    occ_map_pub_.publish(cloud_msg);
    // ROS_INFO("Published occupancy grid with %ld voxels", occ_map_pts_.points.size());
  }

  // std::unique_lock<std::mutex> lck(occ_map_pts_mutex_, std::try_to_lock);
  // if (!lck.owns_lock()){
  //   return;
  // }
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

void GridMap::publishLocalMapBounds()
{
  geometry_msgs::PolygonStamped local_map_poly;
  local_map_poly.header.frame_id = mp_.uav_origin_frame_;
  local_map_poly.header.stamp = ros::Time::now();

  geometry_msgs::Point32 min_corner, corner_0, corner_1, max_corner;
  min_corner.x = mp_.local_map_origin_(0);
  min_corner.y = mp_.local_map_origin_(1);

  corner_0.x = mp_.local_map_max_(0);
  corner_0.y = mp_.local_map_origin_(1);

  corner_1.x = mp_.local_map_origin_(0);
  corner_1.y = mp_.local_map_max_(1);

  max_corner.x = mp_.local_map_max_(0);
  max_corner.y = mp_.local_map_max_(1);

  min_corner.z = corner_0.z = corner_1.z = max_corner.z = mp_.local_map_origin_(2);;

  local_map_poly.polygon.points.push_back(min_corner);
  local_map_poly.polygon.points.push_back(corner_0);
  local_map_poly.polygon.points.push_back(max_corner);
  local_map_poly.polygon.points.push_back(corner_1);

  local_map_poly_pub_.publish(local_map_poly);
}