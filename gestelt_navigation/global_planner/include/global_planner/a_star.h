#ifndef _A_STAR_PLANNER_H_
#define _A_STAR_PLANNER_H_

#include <global_planner/planner_base.h>

#include <unordered_set>
#include <queue>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <chrono>

#include "dynamic_voronoi/dynamicvoronoi.h"

class AStarPlanner : public PlannerBase
{
public:

  struct AStarParams{
    int max_iterations; // Maximum iterations for Astar to run
    double tie_breaker;
    bool debug_viz; // Publish visualization messages for debugging 

    /**
     * 0: Octile
     * 1: L1 Norm 
     * 2: L2 Norm (Euclidean)
     * 3: Chebyshev
     * 4: 
     */
    int cost_function_type; // Type of cost function to use
  }; // struct AStarParams

  AStarPlanner(std::shared_ptr<GridMap> grid_map, const AStarParams& astar_params);

  AStarPlanner( const std::map<int, std::shared_ptr<DynamicVoronoi>>& dyn_voro_arr, 
                const int& z_separation_cm,
                const AStarParams& astar_params);

  /**
   * @brief Clear closed, open list and reset planning_successful flag for 
   * new plan generation
   * 
   */
  void reset();

  // Add ROS Publishers
  void addPublishers(std::unordered_map<std::string, ros::Publisher> &publisher_map);

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
   * @brief Generate a new plan. 
   * 
   * @param start_pos 
   * @param goal_pos 
   * @return true 
   * @return false 
   */
  bool generatePlan(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& goal_pos, 
    std::function<double(const PosIdx&, const PosIdx&)> cost_function);

  bool generatePlanVoronoi(const Eigen::Vector3d& start_pos_3d, const Eigen::Vector3d& goal_pos_3d);

  bool generatePlanVoronoi(const Eigen::Vector3d& start_pos_3d, const Eigen::Vector3d& goal_pos_3d, 
                          std::function<double(const VCell&, const VCell&)> cost_function);

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
  std::vector<Eigen::Vector3d> getPathPosRaw();

  /**
   * @brief Get visited cells
   * 
   * @return std::vector<Eigen::Vector3d> 
   */
  std::vector<Eigen::Vector3d> getClosedList();

  /**
   * @brief Get visited cells
   * 
   * @return std::vector<Eigen::Vector3d> 
   */
  std::vector<Eigen::Vector3d> getClosedListVoronoi();

  void addPublishers(ros::Publisher& closed_list_viz_pub)
  {
      closed_list_viz_pub_ = closed_list_viz_pub;
  }


  void publishClosedList(const std::vector<Eigen::Vector3d>& pts, ros::Publisher& publisher, const std::string& frame_id = "map", Eigen::Vector3d color = Eigen::Vector3d{0.0, 0.0, 0.0}, double radius = 0.1)
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
    sphere_list.ns = "closed_list"; 
    sphere_list.id = 0; 
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

  void expandVoronoiBubble(const VCell& cell, const bool& makeGoalBubble);

  void tracePath(PosIdx final_node);

  void tracePathVoronoi(VCell final_node);

  void clearVisualizations();

  // Round up to multiples of mult
  int roundUpMult(const double& num, const int& mult)
  {
    if (mult == 0){
      return num;
    }
    int rem = (int)num % mult;
    if (rem == 0){
      return num;
    }

    return (num-rem) + mult;
  }

private: 
  std::vector<PosIdx> path_idx_; // Path in terms of indices
  std::vector<Eigen::Vector3d> path_pos_; // Path in terms of 3d position

  ros::Publisher closed_list_viz_pub_;

  /* Params */
  // const double tie_breaker_ = 1.0 + 1.0 / 10000; 
  AStarParams astar_params_;

  /* Path planner data structures */

  // class for commonly used helper functions 
  std::unique_ptr<PlannerCommon> common_; 

  // Priority queue: open list stores nodes yet to be visited
  // std::priority_queue<OccNodePtr, std::vector<OccNodePtr>, OccNode::CompareCostPtr> open_list_; 
  // Hashmap: closed list stores nodes that have been visited
  // std::unordered_set<OccNodePtr, OccNode::PointedObjHash, OccNode::PointedObjEq> closed_list_;

  std::unordered_map<PosIdx, double> g_cost_; 
  std::unordered_map<PosIdx, PosIdx> came_from_;
  PriorityQueue<PosIdx, double> open_list_; // Min priority queue 
  std::unordered_set<PosIdx> closed_list_; // All closed nodes

  std::unique_ptr<OccMap> occ_map_;            // 3D occupancy grid map object
  std::map<int, std::shared_ptr<DynamicVoronoi>> dyn_voro_arr_; // array of voronoi objects with key of height (cm)

  // Voronoi search data structures

  std::unordered_map<VCell, double> g_cost_v_;  
  std::unordered_map<VCell, VCell> came_from_v_;
  PriorityQueue<VCell, double> open_list_v_; // Min priority queue 
  std::unordered_set<VCell> closed_list_v_; // All closed nodes

  std::map<int, std::unordered_set<IntPoint>> marked_bubble_cells_; // Cells that are marked as part of the voronoi bubble with key of height(cm)

  std::vector<VCell> path_idx_v_; // Final planned Path in terms of indices

  int z_separation_cm_; // separation between the voronoi planes in units of centimeters
};

#endif // _A_STAR_PLANNER_H_