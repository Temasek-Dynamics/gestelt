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
                const AStarParams& astar_params,
                std::shared_ptr<std::unordered_set<VCell_T>> resrv_tbl
                );

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

  /* Generate plan on voronoi graph  */
  bool generatePlanVoronoi(const Eigen::Vector3d& start_pos_3d, const Eigen::Vector3d& goal_pos_3d);

  /* Generate plan on voronoi graph  */
  bool generatePlanVoronoi(const Eigen::Vector3d& start_pos_3d, const Eigen::Vector3d& goal_pos_3d, 
                          std::function<double(const VCell&, const VCell&)> cost_function);

  void expandVoronoiBubble(const VCell& cell, const bool& makeGoalBubble);

  // /* Generate space-time plan on voronoi graph  */
  // bool generatePlanVoroT( const Eigen::Vector3d& start_pos_3d, 
  //                         const Eigen::Vector3d& goal_pos_3d);

  // // /* Generate space-time plan on voronoi graph  */
  // bool generatePlanVoroT( const Eigen::Vector3d& start_pos_3d, 
  //                         const Eigen::Vector3d& goal_pos_3d, 
  //                         std::function<double(const VCell_T&, const VCell_T&)> cost_function);


  /* Generate space-time plan on voronoi graph  */
  bool generatePlanVoroT( const Eigen::Vector3i& start_idx_3d, 
                          const Eigen::Vector3i& goal_idx_3d);

  // /* Generate space-time plan on voronoi graph  */
  bool generatePlanVoroT( const Eigen::Vector3i& start_idx_3d, 
                          const Eigen::Vector3i& goal_idx_3d, 
                          std::function<double(const VCell_T&, const VCell_T&)> cost_function);

  /**
   * @brief Get successful plan in terms of path positions
   *
   * @return std::vector<Eigen::Vector3d>
   */
  std::vector<Eigen::Vector3d> getPathPos()
  {
      return path_pos_;
  }

  /**
   * @brief Get successful plan in terms of path positions
   *
   * @return std::vector<Eigen::Vector3d>
   */
  std::vector<Eigen::Vector3d> getPathPosRaw()
  {
      return path_pos_;
  }

  /**
   * @brief Get successful plan in terms of path positions
   *
   * @return std::vector<Eigen::Vector3d>
   */
  std::vector<Eigen::Vector3d> getClosedList()
  {
      std::vector<Eigen::Vector3d> closed_list_pos;
      for (auto itr = closed_list_.begin(); itr != closed_list_.end(); ++itr) {
        Eigen::Vector3d node_pos;
        common_->idxToPos(*itr, node_pos);
        closed_list_pos.push_back(node_pos);
      }

      return closed_list_pos;
  }

  /**
   * @brief Get successful plan in terms of path positions
   *
   * @return std::vector<Eigen::Vector3d>
   */
  std::vector<Eigen::Vector3d> getClosedListVoronoi()
  {
      std::vector<Eigen::Vector3d> closed_list_pos;
      for (auto itr = closed_list_v_.begin(); itr != closed_list_v_.end(); ++itr) {
          DblPoint map_pos;
          IntPoint grid_pos((*itr).x, (*itr).y);

          dyn_voro_arr_[(*itr).z_cm]->idxToPos(grid_pos, map_pos);

          closed_list_pos.push_back(Eigen::Vector3d{map_pos.x, map_pos.y, (*itr).z_m});
      }

      return closed_list_pos;
  }

  void tracePath(PosIdx final_node)
  {
      // Clear existing data structures
      path_idx_.clear();
      path_pos_.clear();

      // Trace back the nodes through the pointer to their parent
      PosIdx cur_node = final_node;
      while (!(cur_node == came_from_[cur_node]))
      {
          path_idx_.push_back(cur_node);
          cur_node = came_from_[cur_node];
      }
      // Push back the start node
      path_idx_.push_back(cur_node);

      // Reverse the order of the path so that it goes from start to goal
      std::reverse(path_idx_.begin(), path_idx_.end());

      // For each gridnode, get the position and index,
      // So we can obtain a path in terms of indices and positions
      for (auto idx : path_idx_)
      {
          Eigen::Vector3d gridnode_pos;
          common_->idxToPos(idx, gridnode_pos);

          path_pos_.push_back(gridnode_pos);
      }
  }

  void tracePathVoronoi(VCell final_node)
{
    // Clear existing data structures
    path_idx_v_.clear();
    path_pos_.clear();

    // Trace back the nodes through the pointer to their parent
    VCell cur_node = final_node;
    while (!(cur_node == came_from_v_[cur_node]))
    {
        path_idx_v_.push_back(cur_node);
        cur_node = came_from_v_[cur_node];
    }
    // Push back the start node
    path_idx_v_.push_back(cur_node);

    // Reverse the order of the path so that it goes from start to goal
    std::reverse(path_idx_v_.begin(), path_idx_v_.end());

    // For each gridnode, get the position and index,
    // So we can obtain a path in terms of indices and positions
    for (const VCell& cell : path_idx_v_)
    {
        DblPoint map_pos;

        IntPoint grid_pos(cell.x, cell.y);
        dyn_voro_arr_[cell.z_cm]->idxToPos(grid_pos, map_pos);

        path_pos_.push_back(Eigen::Vector3d{map_pos.x, map_pos.y, cell.z_m});
    }
}

public:

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

  std::unordered_map<PosIdx, double> g_cost_; 
  std::unordered_map<PosIdx, PosIdx> came_from_;
  PriorityQueue<PosIdx, double> open_list_; // Min priority queue 
  std::unordered_set<PosIdx> closed_list_; // All closed nodes

  std::map<int, std::shared_ptr<DynamicVoronoi>> dyn_voro_arr_; // array of voronoi objects with key of height (cm)


  // General voronoi params
  int z_separation_cm_; // separation between the voronoi planes in units of centimeters
  std::map<int, std::unordered_set<IntPoint>> marked_bubble_cells_; // Cells that are marked as part of the voronoi bubble with key of height(cm)

  // Voronoi search data structures

  std::unordered_map<VCell, double> g_cost_v_;  
  std::unordered_map<VCell, VCell> came_from_v_;
  PriorityQueue<VCell, double> open_list_v_; // Min priority queue 
  std::unordered_set<VCell> closed_list_v_; // All closed nodes

  std::vector<VCell> path_idx_v_; // Final planned Path in terms of indices

  // Space time voronoi Voronoi search data structures
  
  std::unordered_map<VCell_T, double> g_cost_vt_;  
  std::unordered_map<VCell_T, VCell_T> came_from_vt_;
  PriorityQueue<VCell_T, double> open_list_vt_; // Min priority queue 
  std::unordered_set<VCell_T> closed_list_vt_; // All closed nodes

  std::vector<VCell_T> path_idx_vt_; // Final planned Path in terms of indices

  std::shared_ptr<std::unordered_set<VCell_T>> resrv_tbl_; // Reservation table 
};

#endif // _A_STAR_PLANNER_H_