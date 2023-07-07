#include "plan_env/grid_map.h"

// #define current_img_ md_.depth_image_[image_cnt_ & 1]
// #define last_img_ md_.depth_image_[!(image_cnt_ & 1)]

void GridMap::initMap(ros::NodeHandle &nh)
{
  node_ = nh;
  node_name_ = "grid_map";

  /* get parameter */
  double x_size, y_size, z_size;
  node_.param("grid_map/resolution", mp_.resolution_, -1.0);
  node_.param("grid_map/map_size_x", x_size, -1.0);
  node_.param("grid_map/map_size_y", y_size, -1.0);
  node_.param("grid_map/map_size_z", z_size, -1.0);
  node_.param("grid_map/local_update_range_x", mp_.local_update_range_(0), -1.0);
  node_.param("grid_map/local_update_range_y", mp_.local_update_range_(1), -1.0);
  node_.param("grid_map/local_update_range_z", mp_.local_update_range_(2), -1.0);
  node_.param("grid_map/obstacles_inflation", mp_.obstacles_inflation_, -1.0);

  node_.param("grid_map/use_depth_filter", mp_.use_depth_filter_, true);
  node_.param("grid_map/depth_filter_tolerance", mp_.depth_filter_tolerance_, -1.0);
  node_.param("grid_map/depth_filter_maxdist", mp_.depth_filter_maxdist_, -1.0);
  node_.param("grid_map/depth_filter_mindist", mp_.depth_filter_mindist_, -1.0);
  node_.param("grid_map/depth_filter_margin", mp_.depth_filter_margin_, -1);
  node_.param("grid_map/k_depth_scaling_factor", mp_.k_depth_scaling_factor_, -1.0);
  node_.param("grid_map/skip_pixel", mp_.skip_pixel_, -1);

  node_.param("grid_map/p_hit", mp_.p_hit_, 0.70);
  node_.param("grid_map/p_miss", mp_.p_miss_, 0.35);
  node_.param("grid_map/p_min", mp_.p_min_, 0.12);
  node_.param("grid_map/p_max", mp_.p_max_, 0.97);
  node_.param("grid_map/p_occ", mp_.p_occ_, 0.80);
  node_.param("grid_map/fading_time", mp_.fading_time_, 1000.0);
  node_.param("grid_map/min_ray_length", mp_.min_ray_length_, -0.1);
  node_.param("grid_map/max_ray_length", mp_.max_ray_length_, -0.1);

  node_.param("grid_map/visualization_truncate_height", mp_.visualization_truncate_height_, -0.1);

  node_.param("grid_map/show_occ_time", mp_.show_occ_time_, false);
  node_.param("grid_map/pose_type", mp_.pose_type_, 1);
  node_.param("grid_map/sensor_type", mp_.sensor_type_, 1);

  node_.param("grid_map/frame_id", mp_.frame_id_, string("world"));
  node_.param("grid_map/local_map_margin", mp_.local_map_margin_, 1);
  node_.param("grid_map/ground_height", mp_.ground_height_, 0.0);

  node_.param("grid_map/odom_depth_timeout", mp_.odom_depth_timeout_, 1.0);

  double c2b_r, c2b_p, c2b_y, c2b_t_x, c2b_t_y, c2b_t_z;
  node_.param("grid_map/camera_to_body/roll", c2b_r, 0.0);
  node_.param("grid_map/camera_to_body/pitch", c2b_p, 0.0);
  node_.param("grid_map/camera_to_body/yaw", c2b_y, 0.0);
  node_.param("grid_map/camera_to_body/t_x", c2b_t_x, 0.0);
  node_.param("grid_map/camera_to_body/t_y", c2b_t_y, 0.0);
  node_.param("grid_map/camera_to_body/t_z", c2b_t_z, 0.0);

  /* Initialize ROS Subscribers, publishers */

  // depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(node_, "grid_map/depth", 50));
  // extrinsic_sub_ = node_.subscribe<nav_msgs::Odometry>(
  //     "/vins_estimator/extrinsic", 10, &GridMap::extrinsicCallback, this); //sub

  // Subscribe to camera information (only once)
  camera_info_sub_ = node_.subscribe<sensor_msgs::CameraInfo>(
    "camera/depth/camera_info", 10, &GridMap::cameraInfoCallback, this);

  if (mp_.pose_type_ == POSE_STAMPED)
  {
    ROS_INFO("Using camera pose");
    // pose_sub_.reset(
    //     new message_filters::Subscriber<geometry_msgs::PoseStamped>(node_, "grid_map/pose", 25));

    // sync_image_pose_.reset(new message_filters::Synchronizer<SyncPolicyImagePose>(
    //     SyncPolicyImagePose(100), *depth_sub_, *pose_sub_));
    // sync_image_pose_->registerCallback(boost::bind(&GridMap::depthPoseCallback, this, _1, _2));

    // Subscribe to global pose of camera
    pose_sub_ = node_.subscribe<geometry_msgs::PoseStamped>(
      "grid_map/pose", 10, &GridMap::poseCallback, this);
  }
  else if (mp_.pose_type_ == ODOMETRY)
  {
    ROS_INFO("Using camera odometry");
    // odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(node_, "grid_map/odom", 100, ros::TransportHints().tcpNoDelay()));

    // sync_image_odom_.reset(new message_filters::Synchronizer<SyncPolicyImageOdom>(
    //     SyncPolicyImageOdom(100), *depth_sub_, *odom_sub_));
    // sync_image_odom_->registerCallback(boost::bind(&GridMap::depthOdomCallback, this, _1, _2));

    odom_sub_ =
        node_.subscribe<nav_msgs::Odometry>("grid_map/odom", 10, &GridMap::odomCallback, this);
  }
  else {
    ROS_ERROR_NAMED(node_name_, "Unknown pose type");
    ros::shutdown();
  }

  if (mp_.sensor_type_ == SensorType::SENSOR_CLOUD)
  {
    ROS_INFO("Using point clouds as input");
    // Subscribe to point cloud
    cloud_sub_ =
        node_.subscribe<sensor_msgs::PointCloud2>("grid_map/cloud", 10, &GridMap::cloudCallback, this);
  }
  else if (mp_.sensor_type_ == SensorType::SENSOR_DEPTH)
  {
    ROS_INFO("Using depth image as input");
    // Subscribe to depth image of camera
    depth_sub_ = node_.subscribe<sensor_msgs::Image>("grid_map/depth", 25, &GridMap::depthImgCallback, this);
  }

  // Timers
  occ_timer_ = node_.createTimer(ros::Duration(0.05), &GridMap::updateOccupancyTimerCB, this);
  vis_timer_ = node_.createTimer(ros::Duration(0.05), &GridMap::visTimerCB, this);
  fading_timer_ = node_.createTimer(ros::Duration(0.5), &GridMap::fadingTimerCB, this);

  map_pub_ = node_.advertise<sensor_msgs::PointCloud2>("grid_map/occupancy", 10);
  map_inf_pub_ = node_.advertise<sensor_msgs::PointCloud2>("grid_map/occupancy_inflate", 10);

  /* Set initial values for raycasting */
  mp_.resolution_inv_ = 1 / mp_.resolution_;
  mp_.map_origin_ = Eigen::Vector3d(-x_size / 2.0, -y_size / 2.0, mp_.ground_height_);
  mp_.map_size_ = Eigen::Vector3d(x_size, y_size, z_size);

  mp_.prob_hit_log_ = logit(mp_.p_hit_);
  mp_.prob_miss_log_ = logit(mp_.p_miss_);
  mp_.clamp_min_log_ = logit(mp_.p_min_);
  mp_.clamp_max_log_ = logit(mp_.p_max_);
  mp_.min_occupancy_log_ = logit(mp_.p_occ_);
  mp_.unknown_flag_ = 0.01;

  cout << "hit: " << mp_.prob_hit_log_ << endl;
  cout << "miss: " << mp_.prob_miss_log_ << endl;
  cout << "min log: " << mp_.clamp_min_log_ << endl;
  cout << "max: " << mp_.clamp_max_log_ << endl;
  cout << "thresh log: " << mp_.min_occupancy_log_ << endl;

  for (int i = 0; i < 3; ++i)
    mp_.map_voxel_num_(i) = ceil(mp_.map_size_(i) / mp_.resolution_);

  mp_.map_min_boundary_ = mp_.map_origin_;
  mp_.map_max_boundary_ = mp_.map_origin_ + mp_.map_size_;

  /* initialize data buffers */
  int buffer_size = mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2);

  md_.occupancy_buffer_ = vector<double>(buffer_size, mp_.clamp_min_log_ - mp_.unknown_flag_);
  md_.occupancy_buffer_inflate_ = vector<char>(buffer_size, 0);

  md_.count_hit_and_miss_ = vector<short>(buffer_size, 0);
  md_.count_hit_ = vector<short>(buffer_size, 0);
  md_.flag_rayend_ = vector<char>(buffer_size, -1);
  md_.flag_traverse_ = vector<char>(buffer_size, -1);

  // Initialize camera to body matrices
  c2b_r = (M_PI/180.0) * c2b_r;
  c2b_p = (M_PI/180.0) * c2b_p;
  c2b_y = (M_PI/180.0) * c2b_y;

  md_.cam2body_ << cos(c2b_y) * cos(c2b_p),    -sin(c2b_y) * cos(c2b_r) + cos(c2b_y) * sin(c2b_p) * sin(c2b_r),    sin(c2b_y) * sin(c2b_r) + cos(c2b_y) * sin(c2b_p) * cos(c2b_r),  c2b_t_x,
                  sin(c2b_y) * cos(c2b_p),     cos(c2b_y) * cos(c2b_r) + sin(c2b_y) * sin(c2b_p) * sin(c2b_r),    -cos(c2b_y) * sin(c2b_r) + sin(c2b_y) * sin(c2b_p) * cos(c2b_r),  c2b_t_y,
                  -sin(c2b_p),                  cos(c2b_p) * sin(c2b_r),                                            cos(c2b_p) * cos(c2b_y),                                        c2b_t_z,
                  0.0,                          0.0,                                                                0.0,                                                             1.0;

  md_.camera_pos_ << 0.0, 0.0, 0.0;
  md_.cam_to_world_r_m_ <<  1.0, 0.0, 0.0,
                      0.0, 1.0, 0.0,
                      0.0, 0.0, 1.0;

  md_.last_occ_update_time_.fromSec(0);
  
  ROS_INFO("BEFORE INIT DATA STRUCT");

  // Initialize data structures
  pc_sensor_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  octree_map_ = std::make_shared<pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>>(mp_.resolution_);

  ROS_INFO("AFTER INIT DATA STRUCT");
}

void GridMap::initTimeBenchmark(std::shared_ptr<TimeBenchmark> time_benchmark){
  time_benchmark_ = time_benchmark;
}

/** Gridmap data manipulation methods*/

void GridMap::resetBuffer()
{
  Eigen::Vector3d min_pos = mp_.map_min_boundary_;
  Eigen::Vector3d max_pos = mp_.map_max_boundary_;

  resetBuffer(min_pos, max_pos);

  md_.local_bound_min_ = Eigen::Vector3i::Zero();
  md_.local_bound_max_ = mp_.map_voxel_num_ - Eigen::Vector3i::Ones();
}

void GridMap::resetBuffer(Eigen::Vector3d min_pos, Eigen::Vector3d max_pos)
{
  Eigen::Vector3i min_id, max_id;
  posToIndex(min_pos, min_id);
  posToIndex(max_pos, max_id);

  boundIndex(min_id);
  boundIndex(max_id);

  /* reset occ and dist buffer */
  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y)
      for (int z = min_id(2); z <= max_id(2); ++z)
      {
        md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
      }
}

int GridMap::setCacheOccupancy(const Eigen::Vector3d& pos, const int& occ)
{
  if (occ != 1 && occ != 0)
    return INVALID_IDX;

  Eigen::Vector3i id;
  posToIndex(pos, id);
  int idx_ctns = toAddress(id);

  md_.count_hit_and_miss_[idx_ctns] += 1;

  if (md_.count_hit_and_miss_[idx_ctns] == 1)
  {
    md_.cache_voxel_.push(id);
  }

  if (occ == 1)
    md_.count_hit_[idx_ctns] += 1;

  return idx_ctns;
}

void GridMap::clearAndInflateLocalMap()
{
  /*clear outside local*/
  const int vec_margin = 5;
  // Eigen::Vector3i min_vec_margin = min_vec - Eigen::Vector3i(vec_margin,
  // vec_margin, vec_margin); Eigen::Vector3i max_vec_margin = max_vec +
  // Eigen::Vector3i(vec_margin, vec_margin, vec_margin);

  Eigen::Vector3i min_cut = md_.local_bound_min_ -
                            Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  Eigen::Vector3i max_cut = md_.local_bound_max_ +
                            Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  boundIndex(min_cut);
  boundIndex(max_cut);

  Eigen::Vector3i min_cut_m = min_cut - Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
  Eigen::Vector3i max_cut_m = max_cut + Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
  boundIndex(min_cut_m);
  boundIndex(max_cut_m);

  // clear data outside the local range

  for (int x = min_cut_m(0); x <= max_cut_m(0); ++x)
    for (int y = min_cut_m(1); y <= max_cut_m(1); ++y)
    {

      for (int z = min_cut_m(2); z < min_cut(2); ++z)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }

      for (int z = max_cut(2) + 1; z <= max_cut_m(2); ++z)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }
    }

  for (int z = min_cut_m(2); z <= max_cut_m(2); ++z)
    for (int x = min_cut_m(0); x <= max_cut_m(0); ++x)
    {

      for (int y = min_cut_m(1); y < min_cut(1); ++y)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }

      for (int y = max_cut(1) + 1; y <= max_cut_m(1); ++y)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }
    }

  for (int y = min_cut_m(1); y <= max_cut_m(1); ++y)
    for (int z = min_cut_m(2); z <= max_cut_m(2); ++z)
    {

      for (int x = min_cut_m(0); x < min_cut(0); ++x)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }

      for (int x = max_cut(0) + 1; x <= max_cut_m(0); ++x)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }
    }

  // inflate occupied voxels to compensate robot size

  int inf_step = ceil(mp_.obstacles_inflation_ - 0.001 / mp_.resolution_) + 1; // plus 1 for safety since we use "getLessInflateOccupancy()"
  if ( inf_step > 4 )
  {
    ROS_ERROR( "Inflation is too big, which will cause siginificant computation! Reduce inflation or enlarge resolution." );
  }
  vector<Eigen::Vector3i> inf_pts(pow(2 * inf_step + 1, 3));
  // inf_pts.resize(4 * inf_step + 3);
  Eigen::Vector3i inf_pt;

  // clear outdated data
  for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
    for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
      for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z)
      {
        md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
      }

  // inflate obstacles
  for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
    for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
      for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z)
      {

        if (md_.occupancy_buffer_[toAddress(x, y, z)] > mp_.min_occupancy_log_)
        {
          inflatePoint(Eigen::Vector3i(x, y, z), inf_step, inf_pts);

          for (int k = 0; k < (int)inf_pts.size(); ++k)
          {
            inf_pt = inf_pts[k];
            int idx_inf = toAddress(inf_pt);
            if (idx_inf < 0 ||
                idx_inf >= mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2))
            {
              continue;
            }
            md_.occupancy_buffer_inflate_[idx_inf] = 1;
          }
        }
      }
}

/** Timer callbacks */

void GridMap::visTimerCB(const ros::TimerEvent & /*event*/)
{
  publishMapInflate(true);
  publishMap();
}

void GridMap::fadingTimerCB(const ros::TimerEvent & /*event*/)
{
  // Eigen::Vector3d local_range_min = md_.camera_pos_ - mp_.local_update_range_;
  // Eigen::Vector3d local_range_max = md_.camera_pos_ + mp_.local_update_range_;

  // Eigen::Vector3i min_id, max_id;
  // posToIndex(local_range_min, min_id);
  // posToIndex(local_range_max, max_id);
  // boundIndex(min_id);
  // boundIndex(max_id);

  // const double reduce = (mp_.clamp_max_log_ - mp_.min_occupancy_log_) / (mp_.fading_time_ * 2); // function called at 2Hz 
  // const double low_thres = mp_.clamp_min_log_ + reduce;

  // for (int x = min_id(0); x <= max_id(0); ++x)
  //   for (int y = min_id(1); y <= max_id(1); ++y)
  //     for (int z = min_id(2); z <= max_id(2); ++z)
  //     {
  //       int address = toAddress(x, y, z);
  //       if ( md_.occupancy_buffer_[address] > low_thres )
  //       {
  //         // cout << "A" << endl;
  //         md_.occupancy_buffer_[address] -= reduce;
  //       }
  //     }
}

void GridMap::updateOccupancyTimerCB(const ros::TimerEvent & /*event*/)
{
  // std::string benchmark_id = "grid_map_update_occupancy";
  // time_benchmark_->start_stopwatch(benchmark_id);

  // if (md_.last_occ_update_time_.toSec() < 1.0){
  //   md_.last_occ_update_time_ = ros::Time::now();
  // }

  // if (!md_.occ_need_update_) // Flag set by depthImgCallback() 
  // {
  //   if (md_.flag_use_depth_fusion && (ros::Time::now() - md_.last_occ_update_time_).toSec() > mp_.odom_depth_timeout_)
  //   {
  //     ROS_ERROR_THROTTLE(1.0, "odom or depth lost!\
  //       ros::Time::now()=%f,\
  //       md_.last_occ_update_time_=%f,\ 
  //       mp_.odom_depth_timeout_=%f", 
  //       ros::Time::now().toSec(), 
  //       md_.last_occ_update_time_.toSec(), 
  //       mp_.odom_depth_timeout_
  //       );
  //     md_.flag_depth_odom_timeout_ = true; // This flag when true causes the planner to perform an emergency stop
  //   }
  //   return;
  // }
  // md_.last_occ_update_time_ = ros::Time::now();

  // projectDepthImage();

  // raycastProcess();

  // if (md_.local_updated_){ // This is updated after raycasting
  //   clearAndInflateLocalMap();
  // }

  // md_.occ_need_update_ = false;
  // md_.local_updated_ = false;

  // time_benchmark_->stop_stopwatch(benchmark_id);
}

/** Subscriber callbacks */

void GridMap::odomCallback(const nav_msgs::OdometryConstPtr &odom)
{
  ROS_INFO("Got odom!");

  // TODO: The chunk of original code below prevents odom from being updated
  // if (md_.has_first_depth_){
  //   return;
  // }

  md_.camera_pos_(0) = odom->pose.pose.position.x;
  md_.camera_pos_(1) = odom->pose.pose.position.y;
  md_.camera_pos_(2) = odom->pose.pose.position.z;

  Eigen::Quaterniond body_q = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                odom->pose.pose.orientation.x,
                                                odom->pose.pose.orientation.y,
                                                odom->pose.pose.orientation.z);
  Eigen::Matrix3d body_r_m = body_q.toRotationMatrix();
  Eigen::Matrix4d body2world; // Body of uav to world frame
  body2world.block<3, 3>(0, 0) = body_r_m;
  body2world(0, 3) = odom->pose.pose.position.x;
  body2world(1, 3) = odom->pose.pose.position.y;
  body2world(2, 3) = odom->pose.pose.position.z;
  body2world(3, 3) = 1.0;

  // md_.cam2body_ is formed from extrinsicCallback() to "/vins_estimator/extrinsic"
  // Converts camera to world frame
  
  // cam_to_world_T is the transformation from camera to world frame;
  Eigen::Matrix4d cam_to_world_T = body2world * md_.cam2body_;
  md_.camera_pos_(0) = cam_to_world_T(0, 3);
  md_.camera_pos_(1) = cam_to_world_T(1, 3);
  md_.camera_pos_(2) = cam_to_world_T(2, 3);
  md_.cam_to_world_r_m_ = cam_to_world_T.block<3, 3>(0, 0);

  md_.has_odom_ = true;
}

void GridMap::poseCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
  // Transform camera frame to that of the uav
  Eigen::Quaterniond body_q = Eigen::Quaterniond(msg->pose.orientation.w,
                                                msg->pose.orientation.x,
                                                msg->pose.orientation.y,
                                                msg->pose.orientation.z);
  Eigen::Matrix3d body_r_m = body_q.toRotationMatrix();
  // Body of uav to world frame
  Eigen::Matrix4d body2world; 
  body2world.block<3, 3>(0, 0) = body_r_m;
  body2world(0, 3) = msg->pose.position.x;
  body2world(1, 3) = msg->pose.position.y;
  body2world(2, 3) = msg->pose.position.z;
  body2world(3, 3) = 1.0;

  // md_.cam2body_ is formed from extrinsicCallback() to "/vins_estimator/extrinsic"
  // Converts camera to world frame
  Eigen::Matrix4d cam_to_world_T = body2world * md_.cam2body_;
  md_.camera_pos_(0) = cam_to_world_T(0, 3);
  md_.camera_pos_(1) = cam_to_world_T(1, 3);
  md_.camera_pos_(2) = cam_to_world_T(2, 3);
  md_.cam_to_world_r_m_ = cam_to_world_T.block<3, 3>(0, 0);

  md_.has_odom_ = true;
}

void GridMap::depthImgCallback(const sensor_msgs::ImageConstPtr &img)
{
  ROS_INFO("Received depth!");
  if (!md_.has_odom_){
    ROS_ERROR_NAMED(node_name_, "No pose/odom received");
    return;
  }

  /* get depth image */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);

  ROS_INFO("A!");

  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
  }
  cv_ptr->image.copyTo(md_.depth_image_);

  if (!md_.init_depth_img_){
    md_.proj_points_.resize(md_.depth_image_.cols * md_.depth_image_.rows / mp_.skip_pixel_ / mp_.skip_pixel_);
  }

  if (isInMap(md_.camera_pos_))
  {
    md_.update_num_ += 1;
    md_.occ_need_update_ = true;
  }
  else
  {
    ROS_ERROR("Camera pose: (%.2f, %.2f, %.2f) is not within map boundary of (%.2f->%.2f, %.2f->%.2f, %.2f->%.2f)", 
      md_.camera_pos_(0), md_.camera_pos_(1), md_.camera_pos_(2),
      mp_.map_min_boundary_(0), mp_.map_max_boundary_(0),
      mp_.map_min_boundary_(1), mp_.map_max_boundary_(1),
      mp_.map_min_boundary_(2), mp_.map_max_boundary_(2));
    md_.occ_need_update_ = false;
  }

  ROS_INFO("B!");

  md_.flag_use_depth_fusion = true;
  md_.init_depth_img_ = true;
}

void GridMap::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &img)
{
  ROS_INFO("Cloud callback");

  if (!md_.has_odom_){
    ROS_ERROR_NAMED(node_name_, "No pose/odom received");
    return;
  }

  if (isnan(md_.camera_pos_(0)) || isnan(md_.camera_pos_(1)) || isnan(md_.camera_pos_(2))){
    ROS_ERROR_NAMED(node_name_, "Nan camera pose");
    return;
  }

  if (img->data.empty()){
    ROS_ERROR_THROTTLE_NAMED(1.0, node_name_, "Empty point cloud received");
    return;
  }

  md_.has_cloud_ = true;

  pcl::fromROSMsg(*img, *pc_sensor_);

  octree_map_->setInputCloud(pc_sensor_);
  octree_map_->addPointsFromInputCloud();

  // resetBuffer(md_.camera_pos_ - mp_.local_update_range_,
  //                   md_.camera_pos_ + mp_.local_update_range_);

  // int inf_step = ceil(mp_.obstacles_inflation_ - 0.001 / mp_.resolution_) + 1; // plus 1 for safety since we use "getLessInflateOccupancy()"
  // if ( inf_step > 4 )
  // {
  //   ROS_ERROR( "Inflation is too big, which will cause siginificant computation! Reduce inflation or enlarge resolution." );
  // }
  // int inf_step_z = 1;

  // // max and min (x,y,z) values are used to determine the range of the updating grid
  // double max_x, max_y, max_z, min_x, min_y, min_z;

  // min_x = mp_.map_max_boundary_(0);
  // min_y = mp_.map_max_boundary_(1);
  // min_z = mp_.map_max_boundary_(2);

  // max_x = mp_.map_min_boundary_(0);
  // max_y = mp_.map_min_boundary_(1);
  // max_z = mp_.map_min_boundary_(2);

  // Eigen::Vector3d p3d; //Point
  // Eigen::Vector3d p3d_inf; //Inflated point

  // // For each point
  // for (size_t i = 0; i < pc_sensor_->points.size(); ++i)
  // {
  //   // Current point
  //   pcl::PointXYZ pt = pc_sensor_->points[i];
  //   p3d(0) = pt.x, p3d(1) = pt.y, p3d(2) = pt.z;

  //   /* point inside update range */
    
  //   // Vector from camera pose to point
  //   Eigen::Vector3d devi = p3d - md_.camera_pos_;

  //   if (fabs(devi(0)) < mp_.local_update_range_(0) && fabs(devi(1)) < mp_.local_update_range_(1) &&
  //       fabs(devi(2)) < mp_.local_update_range_(2))
  //   {
  //     /* Add non-inflated occupancy grid */
  //     // Eigen::Vector3i actual_pt;
  //     // posToIndex(p3d, actual_pt);
  //     // int idx_actual = toAddress(actual_pt);  
  //     // md_.occupancy_buffer_[idx_actual] = mp_.clamp_max_log_;

  //     /* inflate the point */
  //     for (int x = -inf_step; x <= inf_step; ++x){
  //       for (int y = -inf_step; y <= inf_step; ++y){
  //         for (int z = -inf_step_z; z <= inf_step_z; ++z)
  //         {
  //           p3d_inf(0) = pt.x + x * mp_.resolution_;
  //           p3d_inf(1) = pt.y + y * mp_.resolution_;
  //           p3d_inf(2) = pt.z + z * mp_.resolution_;

  //           Eigen::Vector3i inf_pt;
  //           posToIndex(p3d_inf, inf_pt);

  //           if (!isInMap(inf_pt)){
  //             // If inflated point is not within map then skip this iteration
  //             continue;
  //           }

  //           int idx_inf = toAddress(inf_pt);  

  //           // TODO: Add occupancy_buffer too
  //           md_.occupancy_buffer_inflate_[idx_inf] = 1;

  //           // Get the max/min (x,y,z) occupied by the voxels
  //           max_x = max(max_x, p3d_inf(0));
  //           max_y = max(max_y, p3d_inf(1));
  //           max_z = max(max_z, p3d_inf(2));

  //           min_x = min(min_x, p3d_inf(0));
  //           min_y = min(min_y, p3d_inf(1));
  //           min_z = min(min_z, p3d_inf(2));
  //         }
  //       }
  //     }
  //   }
  // }

  // min_x = min(min_x, md_.camera_pos_(0));
  // min_y = min(min_y, md_.camera_pos_(1));
  // min_z = min(min_z, md_.camera_pos_(2));

  // max_x = max(max_x, md_.camera_pos_(0));
  // max_y = max(max_y, md_.camera_pos_(1));
  // max_z = max(max_z, md_.camera_pos_(2));

  // max_z = max(max_z, mp_.ground_height_);

  // posToIndex(Eigen::Vector3d(max_x, max_y, max_z), md_.local_bound_max_);
  // posToIndex(Eigen::Vector3d(min_x, min_y, min_z), md_.local_bound_min_);

  // boundIndex(md_.local_bound_min_);
  // boundIndex(md_.local_bound_max_);
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
}

// void GridMap::extrinsicCallback(const nav_msgs::OdometryConstPtr &odom)
// {
//   Eigen::Quaterniond cam2body_q = Eigen::Quaterniond(odom->pose.pose.orientation.w,
//                                                      odom->pose.pose.orientation.x,
//                                                      odom->pose.pose.orientation.y,
//                                                      odom->pose.pose.orientation.z);
//   Eigen::Matrix3d cam2body_r_m = cam2body_q.toRotationMatrix();
//   md_.cam2body_.block<3, 3>(0, 0) = cam2body_r_m;
//   md_.cam2body_(0, 3) = odom->pose.pose.position.x;
//   md_.cam2body_(1, 3) = odom->pose.pose.position.y;
//   md_.cam2body_(2, 3) = odom->pose.pose.position.z;
//   md_.cam2body_(3, 3) = 1.0;
// }

/** Helper methods */

bool GridMap::odomValid() { return md_.has_odom_; }

bool GridMap::hasDepthObservation() { return md_.has_first_depth_; }

Eigen::Vector3d GridMap::getOrigin() { return mp_.map_origin_; }

void GridMap::getRegion(Eigen::Vector3d &ori, Eigen::Vector3d &size)
{
  ori = mp_.map_origin_, size = mp_.map_size_;
}

Eigen::Vector3d GridMap::closetPointInMap(const Eigen::Vector3d &pt, const Eigen::Vector3d &camera_pt)
{
  Eigen::Vector3d diff = pt - camera_pt;
  Eigen::Vector3d max_tc = mp_.map_max_boundary_ - camera_pt;
  Eigen::Vector3d min_tc = mp_.map_min_boundary_ - camera_pt;

  double min_t = 1000000;

  for (int i = 0; i < 3; ++i)
  {
    if (fabs(diff[i]) > 0)
    {

      double t1 = max_tc[i] / diff[i];
      if (t1 > 0 && t1 < min_t)
        min_t = t1;

      double t2 = min_tc[i] / diff[i];
      if (t2 > 0 && t2 < min_t)
        min_t = t2;
    }
  }

  return camera_pt + (min_t - 1e-3) * diff;
}

/** Processing methods */

void GridMap::projectDepthImage()
{
  md_.proj_points_cnt = 0;

  uint16_t *row_ptr;
  int cols = md_.depth_image_.cols;
  int rows = md_.depth_image_.rows;
  int skip_pix = mp_.skip_pixel_;

  if (!mp_.use_depth_filter_)
  {
    for (int v = 0; v < rows; v += skip_pix)
    {
      row_ptr = md_.depth_image_.ptr<uint16_t>(v);

      for (int u = 0; u < cols; u += skip_pix)
      {
        // projected point
        Eigen::Vector3d proj_pt;  

        // Project 
        // u = fx * (xc / zc) + cx 
        //    where xc = proj_pt(0) && 
        //          zc = depth
        // v = fy * (yc / zc) + cy 
        //    where yc = proj_pt(1) && 
        //          zc = depth

        // Get coordinates of projected points relative to camera optical frame
        // (x_c, y_c, z_c)

        // Depth (z coordinate) at each pixel
        double z_c = (*row_ptr++) / mp_.k_depth_scaling_factor_;
        // x and y coordiantes
        double x_c = (u - mp_.cx_) * z_c / mp_.fx_;
        double y_c = (v - mp_.cy_) * z_c / mp_.fy_;

        proj_pt << x_c, y_c, z_c;

        proj_pt = md_.cam_to_world_r_m_ * proj_pt + md_.camera_pos_;

        if (u == 320 && v == 240)
        {
          std::cout << "depth: " << z_c << std::endl;
        }

        md_.proj_points_cnt++;
        md_.proj_points_[md_.proj_points_cnt] = proj_pt;
      }
    }
  }
  /* use depth filter */
  else
  {
    if (!md_.has_first_depth_){
      md_.has_first_depth_ = true;
    }
    else
    {
      Eigen::Vector3d pt_cur, pt_world, pt_reproj;

      const double inv_factor = 1.0 / mp_.k_depth_scaling_factor_;

      // Iterate through rows
      for (int v = mp_.depth_filter_margin_; v < rows - mp_.depth_filter_margin_; v += mp_.skip_pixel_)
      {
        row_ptr = md_.depth_image_.ptr<uint16_t>(v) + mp_.depth_filter_margin_;

        // Iterate through columns
        for (int u = mp_.depth_filter_margin_; u < cols - mp_.depth_filter_margin_;
             u += mp_.skip_pixel_)
        {
          double z_c = (*row_ptr) * inv_factor;
          // depth = (*row_ptr) * inv_factor;
          row_ptr = row_ptr + mp_.skip_pixel_;

          if (*row_ptr == 0)
          {
            z_c = mp_.max_ray_length_ + 0.1;
          }
          else if (z_c < mp_.depth_filter_mindist_)
          {
            continue;
          }
          else if (z_c > mp_.depth_filter_maxdist_)
          {
            z_c = mp_.max_ray_length_ + 0.1;
          }

          // Get coordinates of projected points relative to camera optical frame (x_c, y_c, z_c)
          double x_c = (u - mp_.cx_) * z_c / mp_.fx_;
          double y_c = (v - mp_.cy_) * z_c / mp_.fy_;

          pt_cur << x_c, y_c, z_c;

          // point w.r.t world frame
          pt_world = md_.cam_to_world_r_m_ * pt_cur + md_.camera_pos_;

          md_.proj_points_cnt++;
          md_.proj_points_[md_.proj_points_cnt] = pt_world;

        }
      }
    }
  }

}

void GridMap::raycastProcess()
{
  if (md_.proj_points_cnt == 0){
    return;
  }

  ros::Time t1, t2;

  md_.raycast_num_ += 1;

  int vox_idx;
  double length;

  // bounding box of updated region
  double min_x = mp_.map_max_boundary_(0);
  double min_y = mp_.map_max_boundary_(1);
  double min_z = mp_.map_max_boundary_(2);

  double max_x = mp_.map_min_boundary_(0);
  double max_y = mp_.map_min_boundary_(1);
  double max_z = mp_.map_min_boundary_(2);

  RayCaster raycaster;
  Eigen::Vector3d half = Eigen::Vector3d(0.5, 0.5, 0.5);
  Eigen::Vector3d ray_pt, pt_w;

  for (int i = 0; i < md_.proj_points_cnt; ++i)
  {
    pt_w = md_.proj_points_[i];

    // set flag for projected point
    if (!isInMap(pt_w))
    {
      pt_w = closetPointInMap(pt_w, md_.camera_pos_);

      length = (pt_w - md_.camera_pos_).norm();
      if (length > mp_.max_ray_length_)
      {
        pt_w = (pt_w - md_.camera_pos_) / length * mp_.max_ray_length_ + md_.camera_pos_;
      }
      vox_idx = setCacheOccupancy(pt_w, 0);
    }
    else
    {
      length = (pt_w - md_.camera_pos_).norm();

      if (length > mp_.max_ray_length_)
      {
        pt_w = (pt_w - md_.camera_pos_) / length * mp_.max_ray_length_ + md_.camera_pos_;
        vox_idx = setCacheOccupancy(pt_w, 0);
      }
      else
      {
        vox_idx = setCacheOccupancy(pt_w, 1);
      }
    }

    max_x = max(max_x, pt_w(0));
    max_y = max(max_y, pt_w(1));
    max_z = max(max_z, pt_w(2));

    min_x = min(min_x, pt_w(0));
    min_y = min(min_y, pt_w(1));
    min_z = min(min_z, pt_w(2));

    // raycasting between camera center and point

    if (vox_idx != INVALID_IDX)
    {
      if (md_.flag_rayend_[vox_idx] == md_.raycast_num_)
      {
        continue;
      }
      else
      {
        md_.flag_rayend_[vox_idx] = md_.raycast_num_;
      }
    }

    raycaster.setInput(pt_w / mp_.resolution_, md_.camera_pos_ / mp_.resolution_);

    while (raycaster.step(ray_pt))
    {
      Eigen::Vector3d tmp = (ray_pt + half) * mp_.resolution_;
      length = (tmp - md_.camera_pos_).norm();

      // if (length < mp_.min_ray_length_) break;

      vox_idx = setCacheOccupancy(tmp, 0);

      if (vox_idx != INVALID_IDX)
      {
        if (md_.flag_traverse_[vox_idx] == md_.raycast_num_)
        {
          break;
        }
        else
        {
          md_.flag_traverse_[vox_idx] = md_.raycast_num_;
        }
      }
    }
  }

  min_x = min(min_x, md_.camera_pos_(0));
  min_y = min(min_y, md_.camera_pos_(1));
  min_z = min(min_z, md_.camera_pos_(2));

  max_x = max(max_x, md_.camera_pos_(0));
  max_y = max(max_y, md_.camera_pos_(1));
  max_z = max(max_z, md_.camera_pos_(2));
  max_z = max(max_z, mp_.ground_height_);

  posToIndex(Eigen::Vector3d(max_x, max_y, max_z), md_.local_bound_max_);
  posToIndex(Eigen::Vector3d(min_x, min_y, min_z), md_.local_bound_min_);
  boundIndex(md_.local_bound_min_);
  boundIndex(md_.local_bound_max_);

  md_.local_updated_ = true;

  // update occupancy cached in queue
  Eigen::Vector3d local_range_min = md_.camera_pos_ - mp_.local_update_range_;
  Eigen::Vector3d local_range_max = md_.camera_pos_ + mp_.local_update_range_;

  Eigen::Vector3i min_id, max_id;
  posToIndex(local_range_min, min_id);
  posToIndex(local_range_max, max_id);
  boundIndex(min_id);
  boundIndex(max_id);

  // std::cout << "cache all: " << md_.cache_voxel_.size() << std::endl;

  while (!md_.cache_voxel_.empty())
  {

    Eigen::Vector3i idx = md_.cache_voxel_.front();
    int idx_ctns = toAddress(idx);
    md_.cache_voxel_.pop();

    double log_odds_update =
        md_.count_hit_[idx_ctns] >= md_.count_hit_and_miss_[idx_ctns] - md_.count_hit_[idx_ctns] ? mp_.prob_hit_log_ : mp_.prob_miss_log_;

    md_.count_hit_[idx_ctns] = md_.count_hit_and_miss_[idx_ctns] = 0;

    if (log_odds_update >= 0 && md_.occupancy_buffer_[idx_ctns] >= mp_.clamp_max_log_)
    {
      continue;
    }
    else if (log_odds_update <= 0 && md_.occupancy_buffer_[idx_ctns] <= mp_.clamp_min_log_)
    {
      md_.occupancy_buffer_[idx_ctns] = mp_.clamp_min_log_;
      continue;
    }

    bool in_local = idx(0) >= min_id(0) && idx(0) <= max_id(0) && idx(1) >= min_id(1) &&
                    idx(1) <= max_id(1) && idx(2) >= min_id(2) && idx(2) <= max_id(2);
    if (!in_local)
    {
      md_.occupancy_buffer_[idx_ctns] = mp_.clamp_min_log_;
    }

    md_.occupancy_buffer_[idx_ctns] =
        std::min(std::max(md_.occupancy_buffer_[idx_ctns] + log_odds_update, mp_.clamp_min_log_),
                 mp_.clamp_max_log_);
  }
}

/** Publishers */

void GridMap::publishMap()
{
  if (map_pub_.getNumSubscribers() <= 0)
    return;

  // pcl::PointXYZ pt;
  // pcl::PointCloud<pcl::PointXYZ> cloud;

  // Eigen::Vector3i min_cut = md_.local_bound_min_;
  // Eigen::Vector3i max_cut = md_.local_bound_max_;

  // int lmm = mp_.local_map_margin_ / 2;
  // min_cut -= Eigen::Vector3i(lmm, lmm, lmm);
  // max_cut += Eigen::Vector3i(lmm, lmm, lmm);

  // boundIndex(min_cut);
  // boundIndex(max_cut);

  // for (int x = min_cut(0); x <= max_cut(0); ++x){
  //   for (int y = min_cut(1); y <= max_cut(1); ++y){
  //     for (int z = min_cut(2); z <= max_cut(2); ++z){
  //       {
  //         if (md_.occupancy_buffer_[toAddress(x, y, z)] < mp_.min_occupancy_log_){
  //           continue;
  //         }

  //         Eigen::Vector3d pos;
  //         indexToPos(Eigen::Vector3i(x, y, z), pos);
  //         if (pos(2) > mp_.visualization_truncate_height_)
  //           continue;

  //         pt.x = pos(0);
  //         pt.y = pos(1);
  //         pt.z = pos(2);
  //         cloud.push_back(pt);
  //       }
  //     }
  //   }
  // }
  // // width specifies the total number of points in the cloud for unorganized datasets
  // cloud.width = cloud.points.size();
  // // height is set to 1 for unorganized datasets
  // cloud.height = 1; 
  // cloud.is_dense = true;
  // cloud.header.frame_id = mp_.frame_id_;
  // sensor_msgs::PointCloud2 cloud_msg;

  // pcl::toROSMsg(cloud, cloud_msg);
  // map_pub_.publish(cloud_msg);
}

void GridMap::publishMapInflate(bool all_info)
{
  if (map_inf_pub_.getNumSubscribers() <= 0)
    return;

  // // TODO: Allocate size to cloud first and then assign, instead of dynamically pushing back

  // pcl::PointXYZ pt;
  // pcl::PointCloud<pcl::PointXYZ> cloud;

  // Eigen::Vector3i min_cut = md_.local_bound_min_;
  // Eigen::Vector3i max_cut = md_.local_bound_max_;

  // if (all_info)
  // {
  //   int lmm = mp_.local_map_margin_;
  //   min_cut -= Eigen::Vector3i(lmm, lmm, lmm);
  //   max_cut += Eigen::Vector3i(lmm, lmm, lmm);
  // }

  // boundIndex(min_cut);
  // boundIndex(max_cut);

  // for (int x = min_cut(0); x <= max_cut(0); ++x){
  //   for (int y = min_cut(1); y <= max_cut(1); ++y){
  //     for (int z = min_cut(2); z <= max_cut(2); ++z){
  //       if (md_.occupancy_buffer_inflate_[toAddress(x, y, z)] == 0)
  //         continue;

  //       Eigen::Vector3d pos;
  //       indexToPos(Eigen::Vector3i(x, y, z), pos);
  //       if (pos(2) > mp_.visualization_truncate_height_)
  //         continue;

  //       pt.x = pos(0);
  //       pt.y = pos(1);
  //       pt.z = pos(2);
  //       cloud.push_back(pt);
  //     }
  //   }
  // }

  // cloud.width = cloud.points.size();
  // cloud.height = 1;
  // cloud.is_dense = true;
  // cloud.header.frame_id = mp_.frame_id_;
  // sensor_msgs::PointCloud2 cloud_msg;

  // pcl::toROSMsg(cloud, cloud_msg);
  // map_inf_pub_.publish(cloud_msg);

  // ROS_INFO("pub map");
}
