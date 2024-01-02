#ifndef _PLANNER_COMMON_H_
#define _PLANNER_COMMON_H_

// Common helper methods for planners
#include <grid_map/grid_map.h>
#include <limits>
#include <Eigen/Eigen>
#include <pcl/octree/octree_search.h>
#include <pcl/point_cloud.h>

#include <visualization_msgs/Marker.h>

using namespace Eigen;
constexpr double inf = numeric_limits<float>::infinity();
constexpr double epsilon = std::numeric_limits<double>::epsilon();

struct GridNode; //forward declration
typedef std::shared_ptr<GridNode> GridNodePtr;

enum CellState
{
  OPEN = 1,
  CLOSED = 2,
  UNDEFINED = 3
};

struct GridNode
{
  GridNode(const Eigen::Vector3i& idx)
  {
    this->idx = idx;
  }

	Eigen::Vector3i idx;
	double g_cost{inf}, f_cost{inf};
	std::shared_ptr<GridNode> parent{nullptr};
  CellState state{CellState::UNDEFINED};

  // Equality
  bool operator==(const GridNode& node) const
  {
    if (this->idx(0) == node.idx(0) 
      && this->idx(1) == node.idx(1)
      && this->idx(2) == node.idx(2)){
      return true;
    } 
    return false;
  }

  struct ObjHash
  {
    size_t operator()(GridNode& node) const
    {
      return (node.idx(0) * 7927 + node.idx(1)) * 7993 + node.idx(2);
    }
  };

  struct PointedObjEq {
    bool operator () ( GridNodePtr l_node, GridNodePtr r_node) const {
      return *l_node == *r_node;
    }
  };

  struct PointedObjHash
  {
    size_t operator()(const GridNodePtr& node) const
    {
      // https://stackoverflow.com/questions/1358468/how-to-create-unique-integer-number-from-3-different-integers-numbers1-oracle-l
      // https://stackoverflow.com/questions/38965931/hash-function-for-3-integers
      return (node->idx(0) * 7927 + node->idx(1)) * 7993 + node->idx(2);
    }
  };

  struct CompareCostPtr
  {
    bool operator()(const GridNodePtr& l_node, const GridNodePtr& r_node)
    {
      return l_node->f_cost > r_node->f_cost;
    }
  };

};

class PlannerCommon {
/**
 * PlannerCommon acts a wrapper to the underlying obstacle map and provides commonly
 * used methods for search-based planners
 * */ 
public:
  PlannerCommon(pcl::PointCloud<pcl::PointXYZ>::Ptr& map, 
    const Eigen::Vector3d& map_origin, 
    const Eigen::Vector3d& map_size, 
    const double& map_res,
    std::string uav_origin_frame)
  : map_origin_(map_origin), map_size_(map_size), map_res_(map_res), uav_origin_frame_(uav_origin_frame)
  {
    for (int i = 0; i < 3; ++i){
      map_voxel_size_(i) = ceil(map_size(i) / map_res);
    }

    map_max_boundary_ = map_origin + map_size;

    // The resolution parameter describes the length of the smallest voxels at lowest octree level. 
    // octree_map_.reset(new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(map_res));
    octree_map_ = std::make_shared<pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>>(map_res);
    updateMap(map);


    ROS_INFO("Map size (m): (%f, %f, %f) -> (%f, %f, %f)", 
      map_origin_(0), map_origin_(1), map_origin_(2),
      map_max_boundary_(0), map_max_boundary_(1), map_max_boundary_(2));

    ROS_INFO("Map voxel size: (%d, %d, %d)", 
      map_voxel_size_(0), map_voxel_size_(1), map_voxel_size_(2));

    ROS_INFO("Map resolution: (%f)", map_res_);

  }

  void updateMap(pcl::PointCloud<pcl::PointXYZ>::Ptr& map) {
    pc_map_ = map;

    octree_map_->setInputCloud(pc_map_);
    octree_map_->addPointsFromInputCloud();
  }

  std::vector<GridNodePtr> getNeighbors(GridNodePtr cur_node) {
    std::vector<GridNodePtr> neighbors;

    for (int dx = -1; dx <= 1; dx++){
      for (int dy = -1; dy <= 1; dy++){
        for (int dz = -1; dz <= 1; dz++)
        {
          if (dx == 0 && dy == 0 && dz == 0)
            continue;

          Vector3i nb_idx;
          nb_idx(0) = (cur_node->idx)(0) + dx;
          nb_idx(1) = (cur_node->idx)(1) + dy;
          nb_idx(2) = (cur_node->idx)(2) + dz;
          
          if (!isInGlobalMap(nb_idx) || isOccupied(nb_idx)){
            continue;
          }

          GridNodePtr nb_node = std::make_shared<GridNode>(nb_idx);

          neighbors.push_back(nb_node);
        }
      }
    }

    return neighbors;
  } 

  // Get euclidean distance between node_1 and node_2
  double get_euclidean_cost(GridNodePtr node_1, GridNodePtr node_2) {
    return (node_2->idx - node_1->idx).norm();
  }

  // Get euclidean distance between node_1 and node_2
  double get_manhattan_cost(GridNodePtr node_1, GridNodePtr node_2) {
    double dx = abs(node_1->idx(0) - node_2->idx(0));
    double dy = abs(node_1->idx(1) - node_2->idx(1));
    double dz = abs(node_1->idx(2) - node_2->idx(2));

    return dx + dy + dz;
  }

  double get_diag_cost(GridNodePtr node_1, GridNodePtr node_2) {
    double dx = abs(node_1->idx(0) - node_2->idx(0));
    double dy = abs(node_1->idx(1) - node_2->idx(1));
    double dz = abs(node_1->idx(2) - node_2->idx(2));

    double h = 0.0;
    int diag = std::min(min(dx, dy), dz);
    dx -= diag;
    dy -= diag;
    dz -= diag;

    if (dx == 0)
    {
      h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * std::min(dy, dz) + 1.0 * abs(dy - dz);
    }
    if (dy == 0)
    {
      h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * std::min(dx, dz) + 1.0 * abs(dx - dz);
    }
    if (dz == 0)
    {
      h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * std::min(dx, dy) + 1.0 * abs(dx - dy);
    }
    return h;
  }

  // Convert from 3d position to gridmap index
  bool posToIdx(const Vector3d& pos, Vector3i& idx) {
    ROS_INFO("PosToIdx: Pos(%f, %f, %f)", pos(0), pos(1), pos(2));
    if (!isInGlobalMap(pos)){
      return false;
    }

    for (int i = 0; i < 3; ++i){
      idx(i) = floor((pos(i) - map_origin_(i)) / map_res_);
    }
    ROS_INFO("  Converted to idx (%d, %d, %d)", idx(0), idx(1), idx(2));

    return true;
  }

  bool idxToPos(const Vector3i& idx, Vector3d& pos){
    if (!isInGlobalMap(idx)){
      return false;
    }

    for (int i = 0; i < 3; ++i){
      pos(i) = (idx(i) + 0.5) * map_res_ + map_origin_(i);
    }

    return true;
  }

  bool isOccupied(const Vector3i& idx){
    Eigen::Vector3d pos;
    if (!idxToPos(idx, pos)){
      return true;
    }

    return isOccupied(pos);
  }

  bool isOccupied(const Vector3d& pos){
    std::vector<int> pointIdxVec;
    pcl::PointXYZ searchPoint(pos(0), pos(1), pos(2));
    if (octree_map_->voxelSearch(searchPoint, pointIdxVec)){
      return true;
    }
    return false;
  }

  bool isInGlobalMap(const Vector3i& idx){
    if (idx(0) < 0 || idx(1) < 0 || idx(2) < 0)
    {
      return false;
    }
    if (idx(0) > map_voxel_size_(0) - 1 
      || idx(1) > map_voxel_size_(1) - 1 
      || idx(2) > map_voxel_size_(2) - 1)
    {
      ROS_ERROR("idx(%d, %d, %d) is outside voxel size(%d, %d, %d)",
        idx(0), idx(1), idx(2), map_voxel_size_(0), map_voxel_size_(1), map_voxel_size_(2));
      return false;
    }

    return true;
  }

  bool isInGlobalMap(const Vector3d& pos){
    if (pos(0) < map_origin_(0) + epsilon 
        || pos(1) < map_origin_(1) + epsilon 
        || pos(2) < map_origin_(2) + epsilon)
    {
      // std::cout << "less than min range!" << std::endl;
      return false;
    }
    if (pos(0) > map_max_boundary_(0) - epsilon 
      || pos(1) > map_max_boundary_(1) - epsilon 
      || pos(2) > map_max_boundary_(2) - epsilon)
    {
      return false;
    }

    return true;
  }

  void publishClosedList(const std::unordered_set<GridNodePtr, GridNode::PointedObjHash, GridNode::PointedObjEq>& closed_list, ros::Publisher& marker_viz_pub) {
    visualization_msgs::Marker closed_nodes;

    closed_nodes.header.frame_id = uav_origin_frame_;
    closed_nodes.header.stamp = ros::Time::now();
    closed_nodes.type = visualization_msgs::Marker::CUBE_LIST;
    closed_nodes.action = visualization_msgs::Marker::ADD;
    closed_nodes.id = 0; 
    closed_nodes.pose.orientation.w = 1.0;

    closed_nodes.color.r = 1.0;
    closed_nodes.color.g = 1.0;
    closed_nodes.color.b = 1.0;
    closed_nodes.color.a = 0.6;

    closed_nodes.scale.x = 0.1;
    closed_nodes.scale.y = 0.1;
    closed_nodes.scale.z = 0.1;

    geometry_msgs::Point pt;
    for (auto itr = closed_list.begin(); itr != closed_list.end(); ++itr) {
      Eigen::Vector3d node_pos;
      idxToPos((*itr)->idx, node_pos);
      pt.x = node_pos(0);
      pt.y = node_pos(1);
      pt.z = node_pos(2);
      closed_nodes.points.push_back(pt);
    }

    marker_viz_pub.publish(closed_nodes);
  }

  void publishPath(const std::vector<Eigen::Vector3d>& path, ros::Publisher& marker_viz_pub) {
    visualization_msgs::Marker path_spheres, start_sphere, goal_sphere, path_line_strip;

    start_sphere.header.frame_id = goal_sphere.header.frame_id = uav_origin_frame_;
    start_sphere.header.stamp = goal_sphere.header.stamp = ros::Time::now();
    start_sphere.type = goal_sphere.type = visualization_msgs::Marker::SPHERE;
    start_sphere.action = goal_sphere.action = visualization_msgs::Marker::ADD;
    start_sphere.id = 1;
    goal_sphere.id = 2; 
    start_sphere.pose.orientation.w = goal_sphere.pose.orientation.w = 1.0;

    start_sphere.color.r = goal_sphere.color.r = 0.0;
    start_sphere.color.g = goal_sphere.color.g = 0.0;
    start_sphere.color.b = goal_sphere.color.b = 1.0;
    start_sphere.color.a = goal_sphere.color.a = 0.6;

    start_sphere.scale.x = goal_sphere.scale.x = 0.2;
    start_sphere.scale.y = goal_sphere.scale.y = 0.2;
    start_sphere.scale.z = goal_sphere.scale.z = 0.2;

    path_line_strip.header.frame_id = uav_origin_frame_;
    path_line_strip.header.stamp = ros::Time::now();
    path_line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    path_line_strip.action = visualization_msgs::Marker::ADD;
    path_line_strip.id = 1000;
    path_line_strip.pose.orientation.w = 1.0;

    path_line_strip.color.r = 0.0;
    path_line_strip.color.g = 0.0;
    path_line_strip.color.b = 1.0;
    path_line_strip.color.a = 0.6;

    path_line_strip.scale.x = 0.1;

    path_spheres.header.frame_id = uav_origin_frame_;
    path_spheres.header.stamp = ros::Time::now();
    path_spheres.type = visualization_msgs::Marker::SPHERE_LIST;
    path_spheres.action = visualization_msgs::Marker::ADD;
    path_spheres.ns = "global_plan_path_spheres"; 
    path_spheres.id = 995; 
    path_spheres.pose.orientation.w = 1.0;

    path_spheres.color.r = 0.0;
    path_spheres.color.g = 0.0;
    path_spheres.color.b = 1.0;
    path_spheres.color.a = 0.6;

    path_spheres.scale.x = 0.1;
    path_spheres.scale.y = 0.1;
    path_spheres.scale.z = 0.1;

    start_sphere.pose.position.x = path[0](0);
    start_sphere.pose.position.y = path[0](1);
    start_sphere.pose.position.z = path[0](2);

    geometry_msgs::Point pt;
    for (int i = 1; i < path.size() - 1; i++){
      pt.x = path[i](0);
      pt.y = path[i](1);
      pt.z = path[i](2);

      path_spheres.points.push_back(pt);
      // path_line_strip.points.push_back(pt);
    }

    goal_sphere.pose.position.x = path.back()(0);
    goal_sphere.pose.position.y = path.back()(1);
    goal_sphere.pose.position.z = path.back()(2);

    // marker_viz_pub.publish(start_sphere);
    // marker_viz_pub.publish(goal_sphere);
    marker_viz_pub.publish(path_spheres);
    // marker_viz_pub.publish(path_line_strip);

    ROS_INFO("Size of path: %ld", path.size());
  }

private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_map_;
  std::shared_ptr<pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>> octree_map_;

  Eigen::Vector3d map_origin_; // point at which (x,y,z) is a minimum
  Eigen::Vector3d map_size_; // Maximum size of map in (x,y,z)
  Eigen::Vector3d map_max_boundary_; // point at which (x,y,z) is a maximum

  Eigen::Vector3i map_voxel_size_; // Maximum number of voxels in (x,y,z)

  std::string uav_origin_frame_;
  double map_res_;
};

#endif
