#include "grid_map/grid_map.h"

void GridMap::initMap(ros::NodeHandle &nh)
{
  node_ = nh;
  node_name_ = "grid_map";

  /* Map parameters */
  double x_size, y_size, z_size;
  node_.param("grid_map/ground_height", mp_.ground_height_, 0.0);
  node_.param("grid_map/map_size_x", x_size, -1.0);
  node_.param("grid_map/map_size_y", y_size, -1.0);
  node_.param("grid_map/map_size_z", z_size, -1.0);

  node_.param("grid_map/occ_grid/resolution", mp_.occ_resolution_, -1.0);
  node_.param("grid_map/occ_grid/inflation", mp_.occ_inflation_, -1.0);

  /* Point cloud filter */
  node_.param("grid_map/filter/depth_stride", mp_.depth_stride_, 4);
  node_.param("grid_map/filter/downsample_cloud", mp_.downsample_cloud_, false);
  node_.param("grid_map/filter/voxel_size", mp_.voxel_size_, -1.0);

  /* Sensor inputs */
  node_.param("grid_map/pose_type", mp_.pose_type_, 1);
  node_.param("grid_map/sensor_type", mp_.sensor_type_, 1);

  /* Camera parameters  */
  node_.param("grid_map/k_depth_scaling_factor", mp_.k_depth_scaling_factor_, 1000.0);
  double cam2body_roll, cam2body_pitch, cam2body_yaw, cam2body_t_x, cam2body_t_y, cam2body_t_z;

  node_.param("grid_map/cam_frame", mp_.cam_frame_, std::string("cam_link"));
  node_.param("grid_map/global_frame", mp_.global_frame_, std::string("world"));
  node_.param("grid_map/uav_origin_frame", mp_.uav_origin_frame_, std::string("world"));

  /* Camera extrinsic parameters  */
  node_.param("grid_map/camera_to_body/roll", cam2body_roll, 0.0);
  node_.param("grid_map/camera_to_body/pitch", cam2body_pitch, 0.0);
  node_.param("grid_map/camera_to_body/yaw", cam2body_yaw, 0.0);
  node_.param("grid_map/camera_to_body/t_x", cam2body_t_x, 0.0);
  node_.param("grid_map/camera_to_body/t_y", cam2body_t_y, 0.0);
  node_.param("grid_map/camera_to_body/t_z", cam2body_t_z, 0.0);
  int queue_size;
  node_.param("grid_map/sensor_queue_size", queue_size, 5);

  /* Initialize ROS Subscribers, publishers */

  camera_info_sub_ = node_.subscribe<sensor_msgs::CameraInfo>(
    "grid_map/camera_info", 1, &GridMap::cameraInfoCallback, this);

  occ_map_pub_ = node_.advertise<sensor_msgs::PointCloud2>("grid_map/occupancy", 10);

  /* Initialize ROS Timers */
  vis_timer_ = node_.createTimer(ros::Duration(0.1), &GridMap::visTimerCB, this);
  get_tf_timer_ = node_.createTimer(ros::Duration(0.02), &GridMap::getTFTimerCB, this, false, false);

  if (mp_.sensor_type_ == SensorType::SENSOR_CLOUD)
  {
    ROS_INFO("[%s] Point clouds as input", node_name_.c_str());
    cloud_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(
      node_, "grid_map/cloud", queue_size, ros::TransportHints().tcpNoDelay()));
  }
  else if (mp_.sensor_type_ == SensorType::SENSOR_DEPTH) {
    ROS_INFO("[%s] Depth image as input", node_name_.c_str());
    depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(
      node_, "grid_map/depth", queue_size, ros::TransportHints().tcpNoDelay()));
  }
  else if (mp_.sensor_type_ == SensorType::SENSOR_CLOUD_ONLY) {
    ROS_INFO("[%s] Point clouds (TF) as input", node_name_.c_str());
    cloud_only_sub_ = node_.subscribe<sensor_msgs::PointCloud2>(
      "grid_map/cloud", 10, &GridMap::cloudOnlyCB, this);
  }
  else {
    ROS_ERROR("[%s] Unknown SENSOR_TYPE", node_name_.c_str());
    ros::shutdown();
  }

  if (mp_.pose_type_ == PoseType::POSE_STAMPED)
  {
    ROS_INFO("[%s] Pose as camera pose input", node_name_.c_str());
    pose_sub_.reset(new message_filters::Subscriber<geometry_msgs::PoseStamped>(
      node_, "grid_map/pose", queue_size, ros::TransportHints().udp()));
  }
  else if (mp_.pose_type_ == PoseType::ODOMETRY)
  {
    ROS_INFO("[%s] Odometry as camera pose input", node_name_.c_str());
    odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(
      node_, "grid_map/odom", queue_size, ros::TransportHints().udp()));
  }
  else if (mp_.pose_type_ == PoseType::TF)
  {
    ROS_INFO("[%s] TF as camera pose input", node_name_.c_str());
    tfListener_.reset(new tf2_ros::TransformListener(tfBuffer_));
    get_tf_timer_.start();
  }
  else {
    ROS_ERROR("[%s] Unknown POSE_TYPE", node_name_.c_str());
    ros::shutdown();
  }

  if (mp_.pose_type_ == PoseType::POSE_STAMPED) {
    if (mp_.sensor_type_ == SensorType::SENSOR_CLOUD) {
      sync_cloud_pose_.reset(new message_filters::Synchronizer<SyncPolicyCloudPose>(
          SyncPolicyCloudPose(queue_size), *cloud_sub_, *pose_sub_));
      sync_cloud_pose_->registerCallback(boost::bind(&GridMap::cloudPoseCB, this, _1, _2));
    }
    else { // Depth image
      sync_image_pose_.reset(new message_filters::Synchronizer<SyncPolicyImagePose>(
          SyncPolicyImagePose(queue_size), *depth_sub_, *pose_sub_));
      sync_image_pose_->registerCallback(boost::bind(&GridMap::depthPoseCB, this, _1, _2));
    }
  }
  else if (mp_.pose_type_ == PoseType::ODOMETRY) { // Odom
    if (mp_.sensor_type_ == SensorType::SENSOR_CLOUD) {
      sync_cloud_odom_.reset(new message_filters::Synchronizer<SyncPolicyCloudOdom>(
          SyncPolicyCloudOdom(queue_size), *cloud_sub_, *odom_sub_));
      sync_cloud_odom_->registerCallback(boost::bind(&GridMap::cloudOdomCB, this, _1, _2));
    }
    else { // Depth image
      sync_image_odom_.reset(new message_filters::Synchronizer<SyncPolicyImageOdom>(
          SyncPolicyImageOdom(queue_size), *depth_sub_, *odom_sub_));
      sync_image_odom_->registerCallback(boost::bind(&GridMap::depthOdomCB, this, _1, _2));
    }
  }

  /* Set initial values for raycasting */
  mp_.map_origin_ = Eigen::Vector3d(-x_size / 2.0, -y_size / 2.0, mp_.ground_height_);
  mp_.map_size_ = Eigen::Vector3d(x_size, y_size, z_size);

  // // Initialize camera to body matrices
  // Eigen::Affine3d cam2body_tf = Eigen::Affine3d::Identity();
  // cam2body_tf.translation() << cam2body_t_x, cam2body_t_y, cam2body_t_z;
  // cam2body_tf.rotate(Eigen::AngleAxisd(cam2body_roll * (M_PI/180.0), Eigen::Vector3d::UnitX()));
  // cam2body_tf.rotate(Eigen::AngleAxisd(cam2body_pitch * (M_PI/180.0), Eigen::Vector3d::UnitY()));
  // cam2body_tf.rotate(Eigen::AngleAxisd(cam2body_yaw * (M_PI/180.0), Eigen::Vector3d::UnitZ()));

  // md_.cam2body_ = cam2body_tf.matrix();

  // Initialize camera to body matrices
  double c2b_r = (M_PI/180.0) * cam2body_roll;
  double c2b_p = (M_PI/180.0) * cam2body_pitch;
  double c2b_y = (M_PI/180.0) * cam2body_yaw;

  md_.cam2body_ << cos(c2b_y) * cos(c2b_p),    -sin(c2b_y) * cos(c2b_r) + cos(c2b_y) * sin(c2b_p) * sin(c2b_r),    sin(c2b_y) * sin(c2b_r) + cos(c2b_y) * sin(c2b_p) * cos(c2b_r),  cam2body_t_x,
                  sin(c2b_y) * cos(c2b_p),     cos(c2b_y) * cos(c2b_r) + sin(c2b_y) * sin(c2b_p) * sin(c2b_r),    -cos(c2b_y) * sin(c2b_r) + sin(c2b_y) * sin(c2b_p) * cos(c2b_r),  cam2body_t_y,
                  -sin(c2b_p),                  cos(c2b_p) * sin(c2b_r),                                            cos(c2b_p) * cos(c2b_y),                                        cam2body_t_z,
                  0.0,                          0.0,               0.0,                                                             1.0;

  md_.cam_pos_ << 0.0, 0.0, 0.0;
  md_.cam_to_global_r_m_ <<  1.0, 0.0, 0.0,
                      0.0, 1.0, 0.0,
                      0.0, 0.0, 1.0;
  
  // Initialize data structures for occupancy map and point clouds
  // vox_grid_.setLeafSize(mp_.voxel_size_, mp_.voxel_size_, mp_.voxel_size_);

  cloud_origin_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  octree_map_ = std::make_shared<pcl::octree::OctreePointCloudOccupancy<pcl::PointXYZ>>(mp_.occ_resolution_);
  octree_map_inflated_  = std::make_shared<pcl::octree::OctreePointCloudOccupancy<pcl::PointXYZ>>(mp_.occ_inflation_);

  octree_map_->defineBoundingBox (
    -mp_.map_size_(0), -mp_.map_size_(1), -mp_.map_size_(2), 
    mp_.map_size_(0), mp_.map_size_(1), mp_.map_size_(2));

  octree_map_inflated_->defineBoundingBox (
    -mp_.map_size_(0), -mp_.map_size_(1), -mp_.map_size_(2), 
    mp_.map_size_(0), mp_.map_size_(1), mp_.map_size_(2));

}

void GridMap::initTimeBenchmark(std::shared_ptr<TimeBenchmark> time_benchmark){
  time_benchmark_ = time_benchmark;
}

/** Timer callbacks */

void GridMap::visTimerCB(const ros::TimerEvent & /*event*/)
{
  publishMap();
}

void GridMap::getTFTimerCB(const ros::TimerEvent & /*event*/) 
{
  geometry_msgs::TransformStamped cam_to_origin_tf;

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
  md_.cam2global_.block<3, 3>(0, 0) = Eigen::Quaterniond(cam_to_origin_tf.transform.rotation.w,
                                        cam_to_origin_tf.transform.rotation.x,
                                        cam_to_origin_tf.transform.rotation.y,
                                        cam_to_origin_tf.transform.rotation.z).toRotationMatrix();
  md_.cam2global_(0, 3) = cam_to_origin_tf.transform.translation.x;
  md_.cam2global_(1, 3) = cam_to_origin_tf.transform.translation.y;
  md_.cam2global_(2, 3) = cam_to_origin_tf.transform.translation.z;
  md_.cam2global_(3, 3) = 1.0;

  // Converts camera to global frame
  md_.cam_pos_(0) = md_.cam2global_(0, 3);
  md_.cam_pos_(1) = md_.cam2global_(1, 3);
  md_.cam_pos_(2) = md_.cam2global_(2, 3);
  md_.cam_to_global_r_m_ = md_.cam2global_.block<3, 3>(0, 0);

  md_.has_pose_ = true;
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
  cloudToCloudMap(msg_pc);
}

void GridMap::cloudPoseCB( const sensor_msgs::PointCloud2ConstPtr &msg_pc,
                            const geometry_msgs::PoseStampedConstPtr &msg_pose)
{
  getCamToGlobalPose(msg_pose->pose);
  cloudToCloudMap(msg_pc);
}

void GridMap::cloudOnlyCB( const sensor_msgs::PointCloud2ConstPtr &msg_pc) 
{
  cloudToCloudMap(msg_pc);
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

void GridMap::getCamToGlobalPose(const geometry_msgs::Pose &pose)
{
  // Transform camera frame to that of the uav
  Eigen::Quaterniond body_q = Eigen::Quaterniond(pose.orientation.w,
                                                pose.orientation.x,
                                                pose.orientation.y,
                                                pose.orientation.z);
  Eigen::Matrix3d body_r_m = body_q.toRotationMatrix();
  // Body of uav to global frame
  md_.body2global_.block<3, 3>(0, 0) = body_r_m;
  md_.body2global_(0, 3) = pose.position.x;
  md_.body2global_(1, 3) = pose.position.y;
  md_.body2global_(2, 3) = pose.position.z;
  md_.body2global_(3, 3) = 1.0;

  // Converts camera to global frame
  md_.cam2global_ = md_.body2global_ * md_.cam2body_;
  md_.cam_pos_(0) = md_.cam2global_(0, 3);
  md_.cam_pos_(1) = md_.cam2global_(1, 3);
  md_.cam_pos_(2) = md_.cam2global_(2, 3);
  md_.cam_to_global_r_m_ = md_.cam2global_.block<3, 3>(0, 0);

  md_.has_pose_ = true;
}

void GridMap::cloudToCloudMap(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  // Point cloud is assumed to be in frame id of the sensor

  if (!isPoseValid()){
    return;
  }

  if (msg->data.empty()){
    ROS_WARN_THROTTLE(1.0, "[grid_map]: Empty point cloud received");
    // return;
  }

  pcl::fromROSMsg(*msg, *cloud_origin_);

  // Transform point cloud from camera frame to uav origin frame
  pcl::transformPointCloud (*cloud_origin_, *cloud_origin_, md_.cam2global_);

  // Downsample point cloud
  if (mp_.downsample_cloud_){
    vox_grid_.setInputCloud(cloud_origin_);
    vox_grid_.filter(*cloud_origin_);
  }

  // Change frame_id to the uav origin frame, this is because we did a camera-to-origin
  // transformation 
  cloud_origin_->header.frame_id = mp_.uav_origin_frame_;

  cloud_origin_->height = 1;
  cloud_origin_->width = cloud_origin_->points.size();

  // Octree takes in cloud map in global frame
  octree_map_->setOccupiedVoxelsAtPointsFromCloud(cloud_origin_);
  octree_map_inflated_->setOccupiedVoxelsAtPointsFromCloud(cloud_origin_);
}

void GridMap::depthToCloudMap(const sensor_msgs::ImageConstPtr &msg)
{
  // TODO: Still necessary?
  if (!isPoseValid()){
    return;
  }

  /* get depth image */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);

  if (msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
  }
  cv_ptr->image.copyTo(md_.depth_image_);

  // pcl header.stamp must have the timestamp in microseconds or TF will not
  // be able to resolve the transformation
  cloud_origin_->header.stamp = msg->header.stamp.toNSec() * 1e-3;
  cloud_origin_->header.frame_id = mp_.uav_origin_frame_;

  cloud_origin_->height = 1;
  cloud_origin_->width = (md_.depth_image_.cols * md_.depth_image_.rows) / (mp_.depth_stride_ * mp_.depth_stride_);

  cloud_origin_->points.resize(cloud_origin_->width);
  cloud_origin_->is_dense = true;

  uint16_t *row_ptr;

  int cloud_idx = 0;

  for (int v = 0; v < md_.depth_image_.rows; v+=mp_.depth_stride_){
    row_ptr = md_.depth_image_.ptr<uint16_t>(v);
    for (int u = 0; u < md_.depth_image_.cols; u+=mp_.depth_stride_, row_ptr+=mp_.depth_stride_, cloud_idx++){

      pcl::PointXYZ& pt = cloud_origin_->points[cloud_idx];

      float bad_point = std::numeric_limits<float>::quiet_NaN ();

      if (*row_ptr == 0 || *row_ptr == bad_point || *row_ptr == 255){
        pt.x = pt.y = pt.z = bad_point;
        // cloud_origin_->is_dense = false;
      }
      else{
        pt.z = (*row_ptr) / 1000.0;
        pt.x = (u - mp_.cx_) * pt.z * mp_.fx_inv_;
        pt.y = (v - mp_.cy_) * pt.z * mp_.fy_inv_;
      }
    
    }
  }

  // Downsample point cloud
  if (mp_.downsample_cloud_){
    vox_grid_.setInputCloud(cloud_origin_);
    vox_grid_.filter(*cloud_origin_);
  }

  pcl::transformPointCloud(*cloud_origin_, *cloud_origin_, md_.cam2global_);

  // Octree takes in cloud map in global frame
  octree_map_->setOccupiedVoxelsAtPointsFromCloud(cloud_origin_);
  octree_map_inflated_->setOccupiedVoxelsAtPointsFromCloud(cloud_origin_);
}

/* Checks */

bool GridMap::isPoseValid() {
  if (!md_.has_pose_){
    ROS_ERROR_NAMED("[%s] No pose/odom received", node_name_.c_str());
    return false;
  }

  if (isnan(md_.cam_pos_(0)) || isnan(md_.cam_pos_(1)) || isnan(md_.cam_pos_(2))){
    ROS_ERROR_NAMED("[%s] Camera pose has NAN value", node_name_.c_str());
    return false;
  }

  if (!isInMap(md_.cam_pos_))
  {
    ROS_ERROR("[%s] Camera pose (%.2f, %.2f, %.2f) is not within map boundary", 
      node_name_.c_str(), md_.cam_pos_(0), md_.cam_pos_(1), md_.cam_pos_(2));
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
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud_origin_, cloud_msg);

  occ_map_pub_.publish(cloud_msg);
}
