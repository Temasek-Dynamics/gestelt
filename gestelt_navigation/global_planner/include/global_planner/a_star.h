#ifndef _A_STAR_PLANNER_H_
#define _A_STAR_PLANNER_H_

#include <global_planner/planner_common.h>

#include <Eigen/Eigen>

#include <unordered_set>
#include <queue>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <chrono>

#include "dynamic_voronoi/dynamicvoronoi.h"

class AStarPlanner 
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

    int dt{1};
    // int dt_space_diag{3};   // Time to move from (0,0,0) to (1,1,1) in 26-connected 3D grid
    // int dt_face_diag{2};    // Time to move from (0,0,0) to (0,1,1) in 26-connected 3D grid
    // int dt_straight{1};     // Time to move  from (0,0,0) to (0,1,0) in 26-connected 3D grid
    // int dt_wait{1};         // Time to wait in the same cell

  }; // struct AStarParams

  AStarPlanner(std::shared_ptr<GridMap> grid_map, const AStarParams& astar_params);

  AStarPlanner( const AStarParams& astar_params,
                std::shared_ptr<std::unordered_set<Eigen::Vector4d>> resrv_tbl
                );

  /**
   * @brief Clear closed, open list and reset planning_successful flag for 
   * new plan generation
   * 
   */
  void reset();

  /**
   * @brief Assign a dynamic voronoi object 
   * 
   */
  void assignVoroMap(const std::map<int, std::shared_ptr<DynamicVoronoi>>& dyn_voro_arr,
                    const int& z_separation_cm, 
                    const double& local_origin_x,
                    const double& local_origin_y,
                    const double& max_height);

  // Expand voronio bubble around given cell
  void expandVoronoiBubbleT(const VCell_T& origin_cell);

  /* Generate space-time plan on voronoi graph  */
  bool generatePlanVoroT( const Eigen::Vector3d& start_pos_3d, 
                          const Eigen::Vector3d& goal_pos_3d);

  // /* Generate space-time plan on voronoi graph  */
  bool generatePlanVoroT( const Eigen::Vector3d& start_pos_3d, 
                          const Eigen::Vector3d& goal_pos_3d, 
                          std::function<double(const VCell_T&, const VCell_T&)> cost_function);

  /**
   * @brief Get successful plan in terms of path positions
   *
   * @return std::vector<Eigen::Vector3d>
   */
  std::vector<Eigen::Vector4d> getSpaceTimePath()
  {
      return path_pos_t_;
  }

  /**
   * @brief Get successful plan in terms of path positions
   *
   * @return std::vector<Eigen::Vector3d>
   */
  std::vector<Eigen::Vector3d> getClosedListVoroT()
  {
    std::vector<Eigen::Vector3d> closed_list_pos;
    for (auto itr = closed_list_vt_.begin(); itr != closed_list_vt_.end(); ++itr) {
      DblPoint map_pos;
      IntPoint grid_pos((*itr).x, (*itr).y);

      dyn_voro_arr_[(*itr).z_cm]->idxToPos(grid_pos, map_pos);

      closed_list_pos.push_back(Eigen::Vector3d{map_pos.x, map_pos.y, (*itr).z_m});
    }

    return closed_list_pos; 
  }

  void tracePathVoroT(const VCell_T& final_node)
  {
    // Clear existing data structures
    path_idx_vt_.clear();
    path_pos_t_.clear();

    // Trace back the nodes through the pointer to their parent
    VCell_T cur_node = final_node;
    while (!(cur_node == came_from_vt_[cur_node])) // Start node's parents was initially set as itself, hence this indicates the end of the path
    {
      path_idx_vt_.push_back(cur_node);
      cur_node = came_from_vt_[cur_node];
    }
    // Push back the start node
    path_idx_vt_.push_back(cur_node);

    // Reverse the order of the path so that it goes from start to goal
    std::reverse(path_idx_vt_.begin(), path_idx_vt_.end());

    // For each gridnode, get the position and index,
    // So we can obtain a path in terms of indices and positions
    for (const VCell_T& cell : path_idx_vt_)
    {
      DblPoint map_2d_pos;

      // Convert to map position
      dyn_voro_arr_[cell.z_cm]->idxToPos(IntPoint(cell.x, cell.y), map_2d_pos);
      // Add space time map position to path and transform it from local map to world frame
      path_pos_t_.push_back(Eigen::Vector4d{map_2d_pos.x + local_origin_x_, 
                                            map_2d_pos.y + local_origin_y_, cell.z_m, (double) cell.t});
    } 
  }

public:

  // Round up to multiples of mult
  int roundUpMult(const double& num, const int& mult)
  {
    if (num >= max_height_){
      return max_height_;
    }

    if (num <= min_height_){
      return min_height_;
    }

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

  /* Params */
  AStarParams astar_params_;

  /* Path planner data structures */
  double local_origin_x_{0.0}, local_origin_y_{0.0};
  int max_height_{300}; // [cm]
  int min_height_{50}; // [cm]

  std::map<int, std::shared_ptr<DynamicVoronoi>> dyn_voro_arr_; // array of voronoi objects with key of height (cm)

  // General voronoi params
  int z_separation_cm_; // separation between the voronoi planes in units of centimeters
  std::map<int, std::unordered_set<IntPoint>> marked_bubble_cells_; // Cells that are marked as part of the voronoi bubble with key of height(cm)

  // Space time voronoi Voronoi search data structures
  
  std::vector<Eigen::Vector4d> path_pos_t_; // (WORLD FRAME) Spatial-temporal path 
  std::vector<VCell_T> path_idx_vt_; // (LOCAL FRAME) Final planned Path in terms of indices

  std::unordered_map<VCell, double> g_cost_v_;  
  std::unordered_map<VCell_T, VCell_T> came_from_vt_;
  PriorityQueue<VCell_T, double> open_list_vt_; // Min priority queue 
  std::unordered_set<VCell_T> closed_list_vt_; // All closed nodes

  std::shared_ptr<std::unordered_set<Eigen::Vector4d>> resrv_tbl_; // Reservation table 
};

#endif // _A_STAR_PLANNER_H_