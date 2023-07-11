#ifndef _GRID_MAP_H
#define _GRID_MAP_H

#include <random>
#include <queue>
#include <tuple>
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <cv_bridge/cv_bridge.h>

#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/octree/octree_pointcloud_occupancy.h>
#include <pcl/filters/voxel_grid.h>

#include <ros/ros.h>
#include <plan_env/raycast.h>
#include <gestelt_utils/timebenchmark.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/CameraInfo.h>
// #include <trajectory_server_msgs/TimeBenchmark.h>
#include <visualization_msgs/Marker.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
// #include <message_filters/sync_policies/exact_time.h>
// #include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>

#define logit(x) (log((x) / (1 - (x))))

// TODO: Remove
using namespace std;

// voxel hashing
template <typename T>
struct matrix_hash : std::unary_function<T, size_t>
{
  std::size_t operator()(T const &matrix) const
  {
    size_t seed = 0;
    for (size_t i = 0; i < matrix.size(); ++i)
    {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};

// constant parameters

struct MappingParameters
{
  /* map properties */
  Eigen::Vector3d map_origin_, map_size_;
  Eigen::Vector3d map_min_boundary_, map_max_boundary_; // map range in metric position
  Eigen::Vector3i map_voxel_num_;                       // map range in index
  Eigen::Vector3d local_update_range_;                  // Range w.r.t camera pose 
  double resolution_, resolution_inv_;                  
  double map_inflation_;
  std::string global_frame_id_;
  int pose_type_;
  int sensor_type_;

  /* camera parameters */
  double cx_, cy_, fx_, fy_;

  /* time out */
  double odom_depth_timeout_;

  // Point cloud filter
  bool use_cloud_filter_;
  double voxel_size_; // Size of voxel for voxel grid filter

  /* depth image projection filtering */
  double depth_filter_maxdist_, depth_filter_mindist_, depth_filter_tolerance_;
  int depth_filter_margin_;
  bool use_depth_filter_;
  double k_depth_scaling_factor_;
  int skip_pixel_;

  /* raycasting */
  double p_hit_, p_miss_, p_min_, p_max_, p_occ_; // occupancy probability
  double prob_hit_log_, prob_miss_log_, clamp_min_log_, clamp_max_log_,
      min_occupancy_log_;                  // logit of occupancy probability
  double min_ray_length_, max_ray_length_; // range of doing raycasting
  double fading_time_;

  /* local map update and clear */
  int local_map_margin_;

  /* visualization and computation time display */
  double ground_height_;

  /* active mapping */
  double unknown_flag_;
};

// intermediate mapping data for fusion

struct MappingData
{
  // main map data, occupancy of each voxel and Euclidean distance

  std::vector<double> occupancy_buffer_;
  std::vector<char> occupancy_buffer_inflate_;

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

  // flags of map state
  bool occ_need_update_{false}; //Indicates if occupancy map needs to be updated. Typically set to true after receiving new sensor data (e.g. depth image, point cloud pose, odom). 
  bool local_updated_{false}; // Indicates if map has already updated the local cache of occupancy grid, so that inflation can be performed
  bool has_first_depth_{false};
  bool has_pose_{false};

  // odom_depth_timeout_
  ros::Time last_occ_update_time_;
  bool flag_depth_odom_timeout_{false};
  bool flag_use_depth_fusion{false};

  bool init_depth_img_{false}; // First depth image received
  
  std::vector<Eigen::Vector3d> proj_points_; // depth image projected point cloud
  int proj_points_cnt{0}; // Number of pixels from input depth map that have been projected

  // flag buffers for speeding up raycasting
  std::vector<short> count_hit_, count_hit_and_miss_;
  std::vector<char> flag_traverse_, flag_rayend_;
  char raycast_num_{0};
  std::queue<Eigen::Vector3i> cache_voxel_;

  // range of updating grid
  Eigen::Vector3i local_bound_min_, local_bound_max_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class GridMap
{
public:
  GridMap() {}
  ~GridMap() {}

  // Initialize the GridMap class and it's callbacks
  void initMap(ros::NodeHandle &nh);

  // Get time benchmark shared pointer
  void initTimeBenchmark(std::shared_ptr<TimeBenchmark> time_benchmark);

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

  // occupancy map management
  void resetBuffer();
  void resetBuffer(Eigen::Vector3d min, Eigen::Vector3d max);

  // Convert from 3d index to 1d index
  inline int to1DIdx(const Eigen::Vector3i &idx);
  // Convert from 3d index (x,y,z) to 1d index
  inline int to1DIdx(const int &x, const int &y, const int &z);

  inline bool isInMap(const Eigen::Vector3d &pos);
  // inline bool isInMap(const Eigen::Vector3i &idx);

  inline int getOccupancy(const Eigen::Vector3d &pos);
  // inline int getOccupancy(const Eigen::Vector3i &idx);
  inline int getInflateOccupancy(const Eigen::Vector3d &pos);

  // Bound the index to the map range
  // inline void boundIndex(Eigen::Vector3i &idx);
  // inline bool isUnknown(const Eigen::Vector3i &idx);
  // inline bool isUnknown(const Eigen::Vector3d &pos);
  // inline bool isKnownFree(const Eigen::Vector3i &idx);
  // inline bool isKnownOccupied(const Eigen::Vector3i &idx);

  // inline void setOccupancy(const Eigen::Vector3d &pos, double occ = 1);
  // inline void setOccupied(const Eigen::Vector3d &pos);

  // // Convert absolute position to 3d map index
  // inline void posToIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &idx);
  // // Convert 3d map index to position
  // inline void indexToPos(const Eigen::Vector3i &idx, Eigen::Vector3d &pos);

  /** Helper methods */
  
  // Get map origin resolution
  inline double getResolution() { return mp_.resolution_; }

  bool getOdomDepthTimeout() { return md_.flag_depth_odom_timeout_; }

  bool isValid();

  /** Publisher methods */
  void publishMap();

  typedef std::shared_ptr<GridMap> Ptr;

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

  // // VINS estimation callback
  // void extrinsicCallback(const nav_msgs::OdometryConstPtr &odom);

  /**
   * Timer Callbacks
  */

  /**
   * @brief This timer updates the occupancy grid by processing the depth image through ray tracing
  */
  void updateOccupancyTimerCB(const ros::TimerEvent & /*event*/);

  /**
   * @brief This timer publishes a visualization of the occupancy grid
  */
  void visTimerCB(const ros::TimerEvent & /*event*/);

  /**
   * @brief This timer causes the occupancy grid to decay over time
  */
  // void fadingTimerCB(const ros::TimerEvent & /*event*/);

  /**
   * Depth image processing methods
  */

  void projectDepthImage();
  // void raycastProcess();
  // Eigen::Vector3d closestPointInMap(const Eigen::Vector3d &pt, const Eigen::Vector3d &camera_pt);

  /**
   * Gridmap data manipulation methods
  */

  // inline void inflatePoint(const Eigen::Vector3i &pt, int step, std::vector<Eigen::Vector3i> &pts);
  // // int setCacheOccupancy(const Eigen::Vector3d &pos, const int& occ);

  // void clearAndInflateLocalMap();

  void poseToCamPose(const geometry_msgs::Pose &pose);

  void cloudToCloudMap(const sensor_msgs::PointCloud2ConstPtr &msg);
  void depthToCloudMap(const sensor_msgs::ImageConstPtr &msg);

private: 
  ros::NodeHandle node_;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry>
      SyncPolicyImageOdom;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, geometry_msgs::PoseStamped>
      SyncPolicyImagePose;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry>
      SyncPolicyCloudOdom;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, geometry_msgs::PoseStamped>
      SyncPolicyCloudPose;

  // TODO: Tune the parameters http://wiki.ros.org/message_filters/ApproximateTime
  //    Specify Inter message lower bound
  //    Max interval duration
  //    Age penalty
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

  /* ROS Publishers, subscribers and Timers */

  ros::Subscriber camera_info_sub_;

  // ros::Subscriber extrinsic_sub_;

  ros::Publisher map_pub_;
  ros::Timer occ_timer_, vis_timer_, fading_timer_;

  /* Benchmarking */
  std::shared_ptr<TimeBenchmark> time_benchmark_;

  /* Data structures for point clouds */
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_global_;  // Point cloud in global frame
  std::shared_ptr<pcl::octree::OctreePointCloudOccupancy<pcl::PointXYZ>> octree_map_; // In global frame
  std::shared_ptr<pcl::octree::OctreePointCloudOccupancy<pcl::PointXYZ>> octree_map_inflated_; // In global frame
};

/* ============================== definition of inline function
 * ============================== */

// Convert from 3d to 1d index
inline int GridMap::to1DIdx(const Eigen::Vector3i &idx)
{
  return idx(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) + idx(1) * mp_.map_voxel_num_(2) + idx(2);
}

// Convert from 3d to 1d index
inline int GridMap::to1DIdx(const int &x, const int &y, const int &z)
{
  return x * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) + y * mp_.map_voxel_num_(2) + z;
}

// Bound index to size of voxel map
// inline void GridMap::boundIndex(Eigen::Vector3i &idx)
// {
//   Eigen::Vector3i idx1;
//   idx1(0) = max(min(idx(0), mp_.map_voxel_num_(0) - 1), 0);
//   idx1(1) = max(min(idx(1), mp_.map_voxel_num_(1) - 1), 0);
//   idx1(2) = max(min(idx(2), mp_.map_voxel_num_(2) - 1), 0);
//   idx = idx1;
// }

inline int GridMap::getOccupancy(const Eigen::Vector3d &pos)
{
  // If not in map or not in octree bounding box. return -1 
  if (!isInMap(pos)){
    return -1;
  }

  // std::vector<int> pointIdxVec;
  // pcl::PointXYZ searchPoint(pos(0), pos(1), pos(2));
  // if (octree_map_->voxelSearch(searchPoint, pointIdxVec)){
  //   return true;
  // }

  // return false;

  pcl::PointXYZ search_pt(pos(0), pos(1), pos(2));
  return octree_map_->isVoxelOccupiedAtPoint(search_pt) ? 1 : 0;
}

// inline int GridMap::getOccupancy(const Eigen::Vector3i& idx)
// {
//   Eigen::Vector3d pos;
//   indexToPos(idx, pos);

//   return getOccupancy(pos);
// }

inline int GridMap::getInflateOccupancy(const Eigen::Vector3d &pos)
{

  // Eigen::Vector3f min_pt = pos.cast<float>().array() - (float) mp_.map_inflation_;
  // Eigen::Vector3f max_pt = pos.cast<float>().array() + (float) mp_.map_inflation_;

  // std::cout << "min_pt " << min_pt << std::endl;
  // std::cout << "max_pt " << max_pt << std::endl;

  // std::vector<int> pointIdxVec;
  // return octree_map_->boxSearch(min_pt, max_pt, pointIdxVec) ? 1 : 0;

  pcl::PointXYZ search_pt(pos(0), pos(1), pos(2));
  return octree_map_inflated_->isVoxelOccupiedAtPoint(search_pt) ? 1 : 0;
}

// Checks if the position of the camera is currently within the map boundaries
inline bool GridMap::isInMap(const Eigen::Vector3d &pos)
{
  double min_x, min_y, min_z, max_x, max_y, max_z;
  octree_map_->getBoundingBox(min_x, min_y, min_z, max_x, max_y, max_z);

  return (pos(0) >= min_x && pos(0) <= max_x)
    && (pos(1) >= min_y && pos(1) <= max_y)
    && (pos(2) >= min_z && pos(2) <= max_z);
}

// inline bool GridMap::isInMap(const Eigen::Vector3i &idx)
// {
//   Eigen::Vector3d pos;
//   indexToPos(idx, pos);

//   return isInMap(pos);
// }

// inline void GridMap::posToIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &idx)
// {
//   for (int i = 0; i < 3; ++i){
//     idx(i) = floor((pos(i) - mp_.map_origin_(i)) * mp_.resolution_inv_);
//   }
// }

// inline void GridMap::indexToPos(const Eigen::Vector3i &idx, Eigen::Vector3d &pos)
// {
//   for (int i = 0; i < 3; ++i){
//     pos(i) = (idx(i) + 0.5) * mp_.resolution_ + mp_.map_origin_(i);
//   }
// }

// inline void GridMap::inflatePoint(const Eigen::Vector3i &pt, const int step, vector<Eigen::Vector3i> &pts)
// {
//   int num = 0;

//   /* ---------- + shape inflate ---------- */
//   // for (int x = -step; x <= step; ++x)
//   // {
//   //   if (x == 0)
//   //     continue;
//   //   pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1), pt(2));
//   // }
//   // for (int y = -step; y <= step; ++y)
//   // {
//   //   if (y == 0)
//   //     continue;
//   //   pts[num++] = Eigen::Vector3i(pt(0), pt(1) + y, pt(2));
//   // }
//   // for (int z = -1; z <= 1; ++z)
//   // {
//   //   pts[num++] = Eigen::Vector3i(pt(0), pt(1), pt(2) + z);
//   // }

//   /* ---------- all inflate ---------- */
//   for (int x = -step; x <= step; ++x){
//     for (int y = -step; y <= step; ++y){
//       for (int z = -step; z <= step; ++z)
//       {
//         pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1) + y, pt(2) + z);
//       }
//     }
//   }
// }

// inline bool GridMap::isUnknown(const Eigen::Vector3i &idx)
// {
//   Eigen::Vector3i idx1 = idx;
//   boundIndex(idx1);
//   return md_.occupancy_buffer_[to1DIdx(idx1)] < mp_.clamp_min_log_ - 1e-3;
// }

// inline bool GridMap::isUnknown(const Eigen::Vector3d &pos)
// {
//   Eigen::Vector3i idc;
//   posToIndex(pos, idc);
//   return isUnknown(idc);
// }

// inline bool GridMap::isKnownFree(const Eigen::Vector3i &idx)
// {
//   Eigen::Vector3i idx1 = idx;
//   boundIndex(idx1);
//   int adr = to1DIdx(idx1);

//   // return md_.occupancy_buffer_[adr] >= mp_.clamp_min_log_ &&
//   //     md_.occupancy_buffer_[adr] < mp_.min_occupancy_log_;
//   return md_.occupancy_buffer_[adr] >= mp_.clamp_min_log_ && md_.occupancy_buffer_inflate_[adr] == 0;
// }

// inline bool GridMap::isKnownOccupied(const Eigen::Vector3i &idx)
// {
//   Eigen::Vector3i idx1 = idx;
//   boundIndex(idx1);
//   int adr = to1DIdx(idx1);

//   return md_.occupancy_buffer_inflate_[adr] == 1;
// }

// inline void GridMap::setOccupied(const Eigen::Vector3d &pos)
// {
//   if (!isInMap(pos))
//     return;

//   Eigen::Vector3i idx;
//   posToIndex(pos, idx);

//   md_.occupancy_buffer_inflate_[idx(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) +
//                                 idx(1) * mp_.map_voxel_num_(2) + idx(2)] = 1;
// }

// inline void GridMap::setOccupancy(const Eigen::Vector3d &pos, const double occ)
// {
//   if (occ != 1 && occ != 0)
//   {
//     cout << "occ value error!" << endl;
//     return;
//   }

//   if (!isInMap(pos))
//     return;

//   Eigen::Vector3i idx;
//   posToIndex(pos, idx);

//   md_.occupancy_buffer_[to1DIdx(idx)] = occ;
// }


#endif
