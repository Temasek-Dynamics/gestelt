#ifndef _A_STAR_PLANNER_H_
#define _A_STAR_PLANNER_H_

#include <global_planner/planner_base.h>

#include <unordered_set>
#include <queue>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <chrono>

class AStarPlanner : public PlannerBase
{
public:

  struct AStarParams{
    int max_iterations; // Maximum iterations for Astar to run
    double tie_breaker;
    bool debug_viz; // Publish visualization messages for debugging 
  }; // struct SphericalSFCParams

  AStarPlanner(std::shared_ptr<GridMap> grid_map, const AStarParams& astar_params);

  /**
   * @brief Clear closed, open list and reset planning_successful flag for new plan generation
   * 
   */
  void reset();

  /**
   * @brief Generate a new plan. 
   * 
   * @param start_pos 
   * @param goal_pos 
   * @return true 
   * @return false 
   */
  bool generatePlan(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& goal_pos);

  /**
   * @brief Get successful plan in terms of path positions
   * 
   * @return std::vector<Eigen::Vector3d> 
   */
  std::vector<Eigen::Vector3d> getPathPos();

  /**
   * @brief Get successful plan in terms of path positions
   * 
   * @return std::vector<Eigen::Vector3d> 
   */
  std::vector<Eigen::Vector3d> getClosedList();

  void addVizPublishers(ros::Publisher& closed_list_viz_pub)
  {
      closed_list_viz_pub_ = closed_list_viz_pub;
  }

private:

  void addToOpenlist(OccNodePtr node);

  OccNodePtr popOpenlist();

  void addToClosedlist(OccNodePtr node);

  bool isInClosedList(OccNodePtr node);

  void tracePath(OccNodePtr node);

  void publishVizPoints(const std::vector<Eigen::Vector3d>& pts, ros::Publisher& publisher, Eigen::Vector3d color = Eigen::Vector3d{0.0, 0.0, 0.0}, double radius = 0.1, const std::string& frame_id = "world")
  {
    if (!astar_params_.debug_viz || pts.empty()){
      return;
    }

    visualization_msgs::Marker sphere_list;
    double alpha = 0.7;

    sphere_list.header.frame_id = frame_id;
    sphere_list.header.stamp = ros::Time::now();
    sphere_list.type = visualization_msgs::Marker::SPHERE_LIST;
    sphere_list.action = visualization_msgs::Marker::ADD;
    sphere_list.ns = "spherical_sfc_pts"; 
    sphere_list.id = 1; 
    sphere_list.pose.orientation.w = 1.0;

    sphere_list.color.r = color(0);
    sphere_list.color.g = color(1);
    sphere_list.color.b = color(2);
    sphere_list.color.a = alpha;

    sphere_list.scale.x = radius;
    sphere_list.scale.y = radius;
    sphere_list.scale.z = radius;

    geometry_msgs::Point pt;
    for (size_t i = 0; i < pts.size(); i++){
      pt.x = pts[i](0);
      pt.y = pts[i](1);
      pt.z = pts[i](2);

      sphere_list.points.push_back(pt);
    }

    publisher.publish(sphere_list);
  }

private: 
  std::vector<OccNodePtr> path_gridnode_; // Path in terms of Grid Nodes
  std::vector<Eigen::Vector3i> path_idx_; // Path in terms of indices
  std::vector<Eigen::Vector3d> path_pos_; // Path in terms of 3d position

  ros::Publisher closed_list_viz_pub_;

  /* Params */
  // const double tie_breaker_ = 1.0 + 1.0 / 10000; 
  AStarParams astar_params_;

  /* Path planner data structures */

  // class for commonly used helper functions 
  std::unique_ptr<PlannerCommon> common_; 

  // Priority queue: open list stores nodes yet to be visited
  std::priority_queue<OccNodePtr, std::vector<OccNodePtr>, OccNode::CompareCostPtr> open_list_; 
  // Hashmap: closed list stores nodes that have been visited
  std::unordered_set<OccNodePtr, OccNode::PointedObjHash, OccNode::PointedObjEq> closed_list_;

  std::unique_ptr<OccMap<OccNodePtr>> occ_map_;
};

#endif // _A_STAR_PLANNER_H_