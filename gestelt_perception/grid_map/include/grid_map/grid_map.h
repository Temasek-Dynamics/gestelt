#ifndef _GRID_MAP_H
#define _GRID_MAP_H

#include <Eigen/Eigen>

#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/filters/passthrough.h>

#include <ikd_tree/ikd_tree.h>

#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/CameraInfo.h>
#include <visualization_msgs/Marker.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>

#include <bonxai/bonxai.hpp>
#include <bonxai/pcl_utils.hpp>
#include <bonxai/probabilistic_map.hpp>

/* For debugging */
#include <logger/timer.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>

///Pre-allocated std::vector for Eigen using vec_E
template <typename T>
using vec_E = std::vector<T, Eigen::aligned_allocator<T>>;
///Eigen 1D float vector of size N
template <int N>
using Vecf = Eigen::Matrix<double, N, 1>;
///Eigen 1D int vector of size N
template <int N>
using Veci = Eigen::Matrix<int, N, 1>;
///Eigen 1D float vector of dynamic size
using VecDf = Eigen::Matrix<double, Eigen::Dynamic, 1>;

///Vector of Eigen 1D float vector
template <int N>
using vec_Vecf = vec_E<Vecf<N>>;
///Vector of Eigen 1D int vector
template <int N>
using vec_Veci = vec_E<Veci<N>>;

struct MappingParameters
{
  /* map properties */
  // Local and global map are bounded 3d boxes
  Eigen::Vector3d global_map_origin_; // Origin of map (Set to be the corner of the map)
  Eigen::Vector3d local_map_origin_; // Origin of local map (Set to be the corner of the map)
  Eigen::Vector3d local_map_max_; // max position of local map (Set to be the corner of the map)
  
  Eigen::Vector3d global_map_size_; //  Size of global occupancy map  (m)
  Eigen::Vector3d local_map_size_; //  Size of local occupancy map (m)

  Eigen::Vector3i global_map_num_voxels_; //  Size of global occupancy grid (no. of voxels)
  Eigen::Vector3i local_map_num_voxels_; //  Size of local occupancy grid (no. of voxels)

  double resolution_;   // Also defined as the size of each individual voxel                 
  double inflation_;    // Inflation in units of meters
  int inf_num_voxels_;  // Inflation in units of number of voxels, = inflation_/resolution_ 
  int pose_type_;       // Type of pose input (pose, odom or TF)

  double pose_timeout_; // Timeout for pose update before emergency stop is activated

  double max_range;

  /* Cloud downsampler parameters */
  // bool downsample_cloud_; // True if downsampling cloud before input to octree occupancy
  // int depth_stride_; // Number of depth pixels to skip
  // double voxel_size_; // Size of voxel for voxel grid filter

  /* visualization and computation time display */
  double ground_height_; // Lowest possible height (z-axis)

  std::string cam_frame_;
  std::string global_frame_; // frame id of global reference 
  std::string uav_origin_frame_; // frame id of UAV origin

  // bool keep_global_map_{false}; // If true, Bonxai will not discard any nodes outside of the local map bounds. We will map the entire area but at the cost of additional memory.

};

// intermediate mapping data for fusion

struct MappingData
{
  Eigen::Vector3d cam2body_rpy_deg{0.0, 0.0, 0.0};

  // Homogenous Transformation matrix of camera to body frame
  Eigen::Matrix4d cam2body_{Eigen::Matrix4d::Identity(4, 4)};
  // Homogenous Transformation matrix of body to UAV origin frame
  // NOTE: USE `body2origin_.block<3,1>(0,3)` FOR UAV POSE!
  Eigen::Matrix4d body2origin_{Eigen::Matrix4d::Identity(4, 4)};
  // Homogenous Transformation matrix of camera to UAV origin frame
  Eigen::Matrix4d cam2origin_{Eigen::Matrix4d::Identity(4, 4)};

  // True if pose has been received
  bool has_pose_{false};

  // TODO: Use this to flag timeout
  // True if depth and odom has timed out
  double last_sensor_msg_time{-1.0};

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class GridMap
{

// Custom type definition for message filters
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry>
    SyncPolicyImageOdom;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, geometry_msgs::PoseStamped>
    SyncPolicyImagePose;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry>
    SyncPolicyCloudOdom;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, geometry_msgs::PoseStamped>
    SyncPolicyCloudPose;

typedef std::shared_ptr<message_filters::Synchronizer<SyncPolicyImagePose>> SynchronizerImagePose;
typedef std::shared_ptr<message_filters::Synchronizer<SyncPolicyImageOdom>> SynchronizerImageOdom;
typedef std::shared_ptr<message_filters::Synchronizer<SyncPolicyCloudPose>> SynchronizerCloudPose;
typedef std::shared_ptr<message_filters::Synchronizer<SyncPolicyCloudOdom>> SynchronizerCloudOdom;

public:
  typedef std::shared_ptr<GridMap> Ptr;
  using BonxaiT = Bonxai::ProbabilisticMap;

  enum PoseType
  {
    POSE_STAMPED = 1,
    ODOMETRY = 2,
    TF = 3,
    INVALID_IDX = -10000
  };

  /* Initialization methods */

  GridMap() {}

  ~GridMap() {}

  // Reset map data
  void reset(const double& resolution);

  // Initialize gridmap without ros
  // void initMap(pcl::PointCloud<pcl::PointXYZ>::Ptr pcd, const Eigen::Vector3d& map_size, const double& inflation, const double& resolution);

  // Initialize gridmap for ros
  void initMapROS(ros::NodeHandle &nh, ros::NodeHandle &pnh);

  void initROSPubSubTimers(ros::NodeHandle &nh, ros::NodeHandle &pnh);

  void readROSParams(ros::NodeHandle &nh, ros::NodeHandle &pnh);

  /* Gridmap conversion methods */

  // Get camera-to-global frame transformation
  void getCamToGlobalPose(const geometry_msgs::Pose &pose);
  
  // Convert point cloud message to point cloud map, transform it from camera-to-global frame and save it. 
  void pcdMsgToMap(const sensor_msgs::PointCloud2 &msg);
  
  // Convert point cloud to point cloud map, transform it from camera-to-global frame and save it. 
  void pcdToMap(pcl::PointCloud<pcl::PointXYZ>::Ptr pcd);


  /** Getter methods */

  // Get occupancy grid resolution
  double getRes() { return mp_.resolution_; }

  // Get global map origin (This is defined to be a corner of the global map i.e. (-W/2, -L/2, 0))
  Eigen::Vector3d getGlobalOrigin() { return mp_.global_map_origin_; }

  // Get local map origin (This is defined to be a corner of the local map i.e. (-local_W/2, -local_L/2, 0))
  Eigen::Vector3d getLocalOrigin() { return mp_.local_map_origin_; }

  // Get number of voxels in global map
  Eigen::Vector3i getGlobalMapNumVoxels() const { return mp_.global_map_num_voxels_; }

  // Get number of voxels in local map
  Eigen::Vector3i getLocalMapNumVoxels() const { return mp_.local_map_num_voxels_; }

  // Get inflation value
  double getInflation() const{ return mp_.inflation_; }

  /* Checks */

  // Checks if time elapsed has exceeded a given threshold
  bool isTimeout(const double& last_state_time, const double& threshold){
    return (ros::Time::now().toSec() - last_state_time) >= threshold;
  } 

  // Checks if camera pose is valid
  bool isPoseValid();

  /** Publisher methods */

  /**
   * @brief Publish map for visualization
   * 
   */
  void publishOccMap(const pcl::PointCloud<pcl::PointXYZ>& occ_map_pts);

  // Publish sphere to indicate collision with interpolated colors between fatal and warning radius
  void publishCollisionSphere(
    const Eigen::Vector3d &pos, const double& dist_to_obs, 
    const double& fatal_radius, const double& warn_radius);

  // Publish local map bounds
  void publishLocalMapBounds();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  MappingParameters mp_;
  MappingData md_;
  std::string node_name_{"grid_map"};

  /**
   * Subscriber Callbacks
  */

  // Subscriber callback to odom for collision checking
  void odomColCheckCB( const nav_msgs::OdometryConstPtr &msg_odom);

  // Subscriber callback to point cloud and odom
  void cloudOdomCB( const sensor_msgs::PointCloud2ConstPtr &msg_pc, 
                    const nav_msgs::OdometryConstPtr &msg_odom);

  // Subscriber callback to point cloud and pose
  void cloudPoseCB(const sensor_msgs::PointCloud2ConstPtr &msg_pc,
                    const geometry_msgs::PoseStampedConstPtr &msg_pose);

  // Subscriber callback to point cloud and TF
  void cloudTFCB(const sensor_msgs::PointCloud2ConstPtr &msg_pc);

  /**
   * Timer Callbacks
  */

  /**
   * @brief This timer publishes a visualization of the occupancy grid
  */
  void visTimerCB(const ros::TimerEvent & /*event*/);

  /**
   * @brief This timer updates the local map for use by planners
  */
  void updateLocalMapTimerCB(const ros::TimerEvent & /*event*/);

  // /**
  //  * @brief Timer for building KDTree
  // */
  // void buildKDTreeTimerCB(const ros::TimerEvent & /*event*/);

  /**
   * @brief Timer for checking collision of drone with obstacles
   * 
   */
  void checkCollisionsTimerCB(const ros::TimerEvent & /*event*/);


private: 
  /* ROS Publishers, subscribers and Timers */

  // Message filters for point cloud/depth camera and pose/odom
  SynchronizerCloudPose sync_cloud_pose_;
  SynchronizerCloudOdom sync_cloud_odom_;
  
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> cloud_sub_;
  std::shared_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped>> pose_sub_;
  std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;

  ros::Subscriber odom_col_check_sub_; // Subscriber to odom for collision checking

  // Message filters for point cloud and tf
  std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::PointCloud2>> tf_cloud_filter_;

  ros::Subscriber camera_info_sub_;
  ros::Subscriber cloud_only_sub_;

  ros::Publisher occ_map_pub_; // Publisher for occupancy map
  ros::Publisher collision_viz_pub_; // Publisher for collision visualization spheres
  ros::Publisher local_map_poly_pub_;

  ros::Timer vis_occ_timer_; // Timer for visualization
  ros::Timer check_collisions_timer_; // Timer for checking collisions
  ros::Timer update_local_map_timer_; // Timer for updating local map
  // ros::Timer build_kd_tree_timer_; // Timer for updating local map

  // TF transformation 
  tf2_ros::Buffer tfBuffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_;

  /* Params */
  bool dbg_input_entire_map_; // flag to indicate that map will be constructed at the start from the entire pcd map (instead of through incremental sensor data)
  std::string entire_pcd_map_topic_; // Topic to listen for an entire PCD for debugging

  double col_warn_radius_, col_fatal_radius_; // collision check radius

  double viz_occ_map_freq_{-1.0}; // Frequency to publish occupancy map visualization
  double update_local_map_freq_{-1.0};  // Frequency to update local map
  // double build_kd_tree_freq_{-1.0};  // Frequency to build kdtree

  /* Data structures for point clouds */
  pcl::PointCloud<pcl::PointXYZ>::Ptr local_map_in_origin_;  // Point cloud local map in UAV origin frame
  pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_in_origin_;  // Point cloud global map in UAV Origin frame

  std::unique_ptr<BonxaiT> bonxai_map_; // Bonxai data structure 
  pcl::PointCloud<pcl::PointXYZ> occ_map_pts_; // Occupancy map points formed by Bonxai probabilistic mapping
  
  std::shared_ptr<KD_TREE<pcl::PointXYZ>> kdtree_; // KD-Tree 

  std::vector<int8_t> local_map_data_; // 1D array used by path planners for collision checking

  // pcl::VoxelGrid<pcl::PointXYZ> vox_grid_filter_; // Voxel filter

  /* Logic flags*/

  bool check_collisions_{true}; // Flag for checking collisions

  /* Mutexes */
  std::mutex occ_map_pts_mutex_;

  /* Stopwatch for profiling performance */
  Timer tm_bonxai_insert_{"bonxai->insertPointCloud"};
  Timer tm_kdtree_build_{"kdtree_->Build"};

// Frequently used methods
public:

  /* Gridmap operation methods */

  // Called by planners to update the local map
  void updateLocalMap();

  std::vector<int8_t> getData() const {
    return local_map_data_;
  }

  // True if given GLOBAL position is within the GLOBAL map boundaries, else False
  bool isInGlobalMap(const Eigen::Vector3d &pos)
  {
    if (pos(0) >= -mp_.global_map_size_(0)/2 && pos(0) < mp_.global_map_size_(0)/2
      && pos(1) >= -mp_.global_map_size_(1)/2 && pos(1) < mp_.global_map_size_(1)/2
      && pos(2) >= -mp_.global_map_size_(2)/2 && pos(2) < mp_.global_map_size_(2)/2)
    {
      return true;
    }

    return false;
  }

  // True if given GLOBAL position is within the LOCAL map boundaries, else False
  bool isInLocalMap(const Eigen::Vector3d &pos)
  {
    if (pos(0) >= mp_.local_map_origin_(0)   && pos(0) < mp_.local_map_max_(0)
      && pos(1) >= mp_.local_map_origin_(1)  && pos(1) < mp_.local_map_max_(1)
      && pos(2) >= mp_.local_map_origin_(2)  && pos(2) < mp_.local_map_max_(2))
    {
      return true;
    }

    return false;
  }

  // Get occupancy value of given position in Occupancy grid
  bool getOccupancy(const Eigen::Vector3d &pos){
    // If not in map or not in octree bounding box. return -1 
    if (!isInGlobalMap(pos)){
      return true;
    }

    Bonxai::CoordT coord = bonxai_map_->grid().posToCoord(pos(0), pos(1), pos(2));

    return bonxai_map_->isOccupied(coord);
  }

  // Get occupied bool of given position in inflated Occupancy grid
  bool getInflateOccupancy(const Eigen::Vector3d &pos) {
    return getInflateOccupancy(pos, mp_.inflation_);
  }

  // Get occupied bool of given position in occ grid with specified inflation
  bool getInflateOccupancy(const Eigen::Vector3d &pos, const double& inflation) {
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
    
    // return false;

    /* Using KDTree to check for inflation */
    if (!isInGlobalMap(pos)){
      return true;
    }

    return withinObsRadius(pos, inflation);
  }

  /**
   * @brief Get the Nearest Occupied Cell  
   * 
   * @param pos 
   * @param occ_nearest position of nearest occupied cell
   * @param radius 
   * @return true 
   * @return false 
   */
  bool getNearestOccupiedCell(const Eigen::Vector3d &pos, 
                              Eigen::Vector3d& occ_nearest, double& dist_to_nearest_nb){
    int nearest_num_nb = 1;
    pcl::PointXYZ search_point(pos(0), pos(1), pos(2));
    std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> nb_points;
    std::vector<float> nb_radius_vec;

    kdtree_->Nearest_Search(search_point, nearest_num_nb, nb_points, nb_radius_vec);

    if (nb_points.empty()){
      return false;
    }

    dist_to_nearest_nb = sqrt(nb_radius_vec[0]);

    occ_nearest = Eigen::Vector3d{nb_points[0].x, nb_points[0].y, nb_points[0].z};

    return true;
  }

  /**
   * @brief Check if position is within a radius of an obstacle
   * 
   * @param pos 
   * @param radius 
   * @return true 
   * @return false 
   */
  bool withinObsRadius(const Eigen::Vector3d &pos, const double& radius){
    pcl::PointXYZ search_point(pos(0), pos(1), pos(2));
    std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> nb_points;
    kdtree_->Radius_Search(search_point, radius, nb_points);

    return !nb_points.empty();
  }

  // Check if current index is free
  bool isFree(const Eigen::Vector3i& idx) {
    return !isOccupied(idx);

    // if (!isInGlobalVoxelMap(idx)){
    //   std::cout << "isFree (" << idx.transpose() <<") NOT in global voxel map which has size " << mp_.global_map_num_voxels_.transpose() << std::endl;
    //   return false;
    // }

    // Eigen::Vector3d pos = intToFloat(idx);
    // Bonxai::CoordT coord = bonxai_map_->grid().posToCoord(pos(0), pos(1), pos(2));

    // std::cout << "bonxai_map_->isFree( " << pos.transpose() <<  "): " << bonxai_map_->isFree(coord) << std::endl;
    // return bonxai_map_->isFree(coord);
  }

  /// Check current index is unknown
  bool isUnknown(const Veci<3> &idx) {
    if (!isInGlobalVoxelMap(idx)){
      return true;
    }

    Eigen::Vector3d pos = intToFloat(idx);
    Bonxai::CoordT coord = bonxai_map_->grid().posToCoord(pos(0), pos(1), pos(2));

    // TODO: change is isUnknown method from bonxai_map_
    // return bonxai_map_->isUnknown(coord);
    return bonxai_map_->isOccupied(coord);
  }

  // Check if current index is occupied
  bool isOccupied(const Eigen::Vector3i& idx) {

    if (!isInGlobalVoxelMap(idx)){
      return true; 
    }

    Eigen::Vector3d pos = intToFloat(idx);
    Bonxai::CoordT coord = bonxai_map_->grid().posToCoord(pos(0), pos(1), pos(2));

    return bonxai_map_->isOccupied(coord);
  }

  // Check if index is within map bounds
  bool isInLocalVoxelMap(const Eigen::Vector3i& idx) {
    if (idx(0) >= 0 && idx(0) < mp_.local_map_num_voxels_(0)
      && idx(1) >= 0 && idx(1) < mp_.local_map_num_voxels_(1)
      && idx(2) >= 0 && idx(2) < mp_.local_map_num_voxels_(2))
    {
      return true;
    }
    return false;
  }

  // Check if index is within map bounds
  bool isInGlobalVoxelMap(const Eigen::Vector3i& idx) {
    if (idx(0) >= 0 && idx(0) < mp_.global_map_num_voxels_(0)
      && idx(1) >= 0 && idx(1) < mp_.global_map_num_voxels_(1)
      && idx(2) >= 0 && idx(2) < mp_.global_map_num_voxels_(2))
    {
      return true;
    }
    return false;
  }

  bool isOutside(const Eigen::Vector3i& idx){
    return !isInGlobalVoxelMap(idx);
  }
  
  /// Check if the ray from p1 to p2 is occluded
  // TODO Remove val
  bool isBlocked(const Vecf<3> &p1, const Vecf<3> &p2) {
    vec_Veci<3> pns = rayTrace(p1, p2);
    for (const auto &pn : pns) {
      if (isOccupied(pn)) {
        return true;
      }
    }
    return false;
  }

  // // returns probability value of grid cell from 0.0 to 1.0
  // double getOccVal(const Eigen::Vector3i& idx)
  // {
  //   if (!isInGlobalVoxelMap(idx)){
  //     return 1.0;
  //   }

  //   Eigen::Vector3d pos = intToFloat(idx);
  //   Bonxai::CoordT coord = bonxai_map_->grid().posToCoord(pos(0), pos(1), pos(2));

  //   return bonxai_map_->getOccVal(coord);
  // }

  /// Float position to discrete cell coordinate
  Veci<3> floatToInt(const Eigen::Vector3d &pos) {

    // Veci<3> idx = (((pos - getGlobalOrigin()) / getRes() ) - Eigen::Vector3d::Constant(0.5)).cast<int>() ;
    Veci<3> idx = ((pos - getGlobalOrigin()) / getRes() ).cast<int>() ;
    return idx;
  }

  /// Discrete cell coordinate to float position
  Eigen::Vector3d intToFloat(const Veci<3> &idx) {
    // return (idx.template cast<double>() + Vecf<3>::Constant(0.5)) * getRes() + getGlobalOrigin();

    // Eigen::Vector3d pos = (idx.cast<double>() + Eigen::Vector3d::Constant(0.5))  * getRes() + getGlobalOrigin();
    Eigen::Vector3d pos = (idx.cast<double>())  * getRes() + getGlobalOrigin();
    return pos;
  }

  /// Raytrace from float point pt1 to pt2
  vec_Veci<3> rayTrace(const Vecf<3> &pt1, const Vecf<3> &pt2) {
    Vecf<3> diff = pt2 - pt1;
    double k = 0.8;
    int max_diff = (diff / getRes()).template lpNorm<Eigen::Infinity>() / k;
    double s = 1.0 / max_diff;
    Vecf<3> step = diff * s;

    vec_Veci<3> pns;
    Veci<3> prev_pn = Veci<3>::Constant(-1);
    for (int n = 1; n < max_diff; n++) {
      Vecf<3> pt = pt1 + step * n;
      Veci<3> new_pn = floatToInt(pt);
      if (!isInGlobalVoxelMap(new_pn))
        break;
      if (new_pn != prev_pn)
        pns.push_back(new_pn);
      prev_pn = new_pn;
    }
    return pns;
  }

}; // class GridMap



#endif //_GRID_MAP_H
