#ifndef _GRID_MAP_H
#define _GRID_MAP_H

// #include <random>
// #include <queue>
// #include <tuple>
#include <Eigen/Eigen>
// #include <Eigen/StdVector>
#include <cv_bridge/cv_bridge.h>

#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/octree/octree_pointcloud_occupancy.h>
#include <pcl/filters/voxel_grid.h>

#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/CameraInfo.h>

#include <gestelt_utils/timebenchmark.h>

struct MappingParameters
{
  /* map properties */
  Eigen::Vector3d map_origin_, map_size_; // Origin and size of occupancy grid 
  double occ_resolution_; // Voxel size for occupancy grid without inflation                  
  double occ_inflation_; // Voxel size for occupancy grid with inflation
  int pose_type_; // Type of pose input (pose or odom)
  int sensor_type_; // Type of sensor (cloud or depth image)
  std::string global_frame_id_; // frame id to display occupancy grid in

  double pose_timeout_; // Timeout for pose update before emergency stop is activated

  /* camera parameters */
  double cx_, cy_, fx_, fy_, fx_inv_, fy_inv_; // Intrinsic camera parameters
  double k_depth_scaling_factor_; // Scaling factor for depth value of depth image

  /* Cloud downsampler parameters */
  bool downsample_cloud_; // True if downsampling cloud before input to octree occupancy
  int depth_stride_; // Number of depth pixels to skip
  double voxel_size_; // Size of voxel for voxel grid filter

  /* visualization and computation time display */
  double ground_height_; // Lowest possible height (z-axis)

};

// intermediate mapping data for fusion

struct MappingData
{
  // camera position in global frame 
  Eigen::Vector3d cam_pos_{0.0, 0.0, 0.0};

  // Rotation matrix of camera to global frame
  Eigen::Matrix3d cam_to_global_r_m_;

  // Transformation matrix of camera to body frame
  Eigen::Matrix4d cam2body_;
  // Transformation matrix of body to global frame
  Eigen::Matrix4d body2global_;
  // Transformation matrix of camera to global frame
  Eigen::Matrix4d cam2global_;

  // depth image data
  cv::Mat depth_image_;

  // True if pose has been received
  bool has_pose_{false};

  // TODO: Use this to flag timeout
  // True if depth and odom has timed out
  bool flag_depth_odom_timeout_{false};

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class GridMap
{
public:
  typedef std::shared_ptr<GridMap> Ptr;

  enum PoseType
  {
    POSE_STAMPED = 1,
    ODOMETRY = 2,
    INVALID_IDX = -10000
  };

  enum SensorType
  {
    SENSOR_CLOUD = 1,
    SENSOR_DEPTH = 2,
  };

  /* Initialization methods */

  GridMap() {}
  ~GridMap() {}

  // Initialize the GridMap class and it's callbacks
  void initMap(ros::NodeHandle &nh);

  // Get time benchmark shared pointer
  void initTimeBenchmark(std::shared_ptr<TimeBenchmark> time_benchmark);
  
  /* Gridmap access methods */

  // True if the position camera pose is currently within the map boundaries
  inline bool isInMap(const Eigen::Vector3d &pos);
  // Get occupancy value of given position in Occupancy grid
  inline int getOccupancy(const Eigen::Vector3d &pos);
  // Get occupancy value of given position in inflated Occupancy grid
  inline int getInflateOccupancy(const Eigen::Vector3d &pos);

  /* Gridmap conversion methods */

  // Get occupancy value of given position in inflated Occupancy grid
  void poseToCamPose(const geometry_msgs::Pose &pose);
  void cloudToCloudMap(const sensor_msgs::PointCloud2ConstPtr &msg);
  void depthToCloudMap(const sensor_msgs::ImageConstPtr &msg);

  /** Helper methods */
  bool isPoseValid();

  // Get map origin resolution
  inline double getResolution() { return mp_.occ_resolution_; }

  bool getOdomDepthTimeout() { return md_.flag_depth_odom_timeout_; }

  /** Publisher methods */
  void publishMap();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  MappingParameters mp_;
  MappingData md_;
  std::string node_name_;

  /**
   * Subscriber Callbacks
  */

  // Subscriber callback to camera info 
  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg);

  // Subscriber callback to depth image and base_link odom
  void depthOdomCB( const sensor_msgs::ImageConstPtr &msg_img, 
                    const nav_msgs::OdometryConstPtr &msg_odom);

  // Subscriber callback to depth image and base_link pose
  void depthPoseCB(const sensor_msgs::ImageConstPtr &msg_img,
                    const geometry_msgs::PoseStampedConstPtr &msg_pose);

  // Subscriber callback to point cloud and odom
  void cloudOdomCB( const sensor_msgs::PointCloud2ConstPtr &msg_pc, 
                    const nav_msgs::OdometryConstPtr &msg_odom);

  // Subscriber callback to point cloud and pose
  void cloudPoseCB(const sensor_msgs::PointCloud2ConstPtr &msg_pc,
                    const geometry_msgs::PoseStampedConstPtr &msg_pose);

  /**
   * Timer Callbacks
  */

  /**
   * @brief This timer publishes a visualization of the occupancy grid
  */
  void visTimerCB(const ros::TimerEvent & /*event*/);

private: 
  ros::NodeHandle node_;

  /* ROS Publishers, subscribers and Timers */
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

  SynchronizerImagePose sync_image_pose_;
  SynchronizerImageOdom sync_image_odom_;
  SynchronizerCloudPose sync_cloud_pose_;
  SynchronizerCloudOdom sync_cloud_odom_;
  
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depth_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> cloud_sub_;
  std::shared_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped>> pose_sub_;
  std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;

  ros::Subscriber camera_info_sub_;

  ros::Publisher occ_map_pub_;
  ros::Timer vis_timer_;

  /* Benchmarking */
  std::shared_ptr<TimeBenchmark> time_benchmark_;

  /* Data structures for point clouds */
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_global_;  // Point cloud in global frame
  std::shared_ptr<pcl::octree::OctreePointCloudOccupancy<pcl::PointXYZ>> octree_map_; // In global frame
  std::shared_ptr<pcl::octree::OctreePointCloudOccupancy<pcl::PointXYZ>> octree_map_inflated_; // In global frame

  pcl::VoxelGrid<pcl::PointXYZ> vox_grid_;
};

/* ============================== definition of inline function
 * ============================== */

inline int GridMap::getOccupancy(const Eigen::Vector3d &pos)
{
  // If not in map or not in octree bounding box. return -1 
  if (!isInMap(pos)){
    return -1;
  }

  pcl::PointXYZ search_pt(pos(0), pos(1), pos(2));
  return octree_map_->isVoxelOccupiedAtPoint(search_pt) ? 1 : 0;
}

inline int GridMap::getInflateOccupancy(const Eigen::Vector3d &pos)
{

  pcl::PointXYZ search_pt(pos(0), pos(1), pos(2));
  return octree_map_inflated_->isVoxelOccupiedAtPoint(search_pt) ? 1 : 0;
}

inline bool GridMap::isInMap(const Eigen::Vector3d &pos)
{
  double min_x, min_y, min_z, max_x, max_y, max_z;
  octree_map_->getBoundingBox(min_x, min_y, min_z, max_x, max_y, max_z);

  return (pos(0) >= min_x && pos(0) <= max_x)
    && (pos(1) >= min_y && pos(1) <= max_y)
    && (pos(2) >= min_z && pos(2) <= max_z);
}

#endif
