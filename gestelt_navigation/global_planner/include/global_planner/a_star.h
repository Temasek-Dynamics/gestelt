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
  }; // struct AStarParams

  AStarPlanner(std::shared_ptr<GridMap> grid_map, const AStarParams& astar_params);

  AStarPlanner( const AStarParams& astar_params,
                std::shared_ptr<std::unordered_set<Eigen::Vector4d>> resrv_tbl
                );

  /**
   * @brief 
   * 
   */
  void assignVoroMap(const std::map<int, std::shared_ptr<DynamicVoronoi>>& dyn_voro_arr,
                    const int& z_separation_cm);

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

  // Expand voronio bubble around given cell
  void expandVoronoiBubble(const VCell& cell);

  // Expand voronio bubble around given cell
  void expandVoronoiBubbleT(const VCell_T& origin_cell);

  /* Generate space-time plan on voronoi graph  */
  bool generatePlanVoroT( const Eigen::Vector3d& start_pos_3d, 
                          const Eigen::Vector3d& goal_pos_3d);

  // /* Generate space-time plan on voronoi graph  */
  bool generatePlanVoroT( const Eigen::Vector3d& start_pos_3d, 
                          const Eigen::Vector3d& goal_pos_3d, 
                          std::function<double(const VCell_T&, const VCell_T&)> cost_function);


  // /* Generate space-time plan on voronoi graph  */
  // bool generatePlanVoroT( const Eigen::Vector3i& start_idx_3d, 
  //                         const Eigen::Vector3i& goal_idx_3d);

  // // /* Generate space-time plan on voronoi graph  */
  // bool generatePlanVoroT( const Eigen::Vector3i& start_idx_3d, 
  //                         const Eigen::Vector3i& goal_idx_3d, 
  //                         std::function<double(const VCell_T&, const VCell_T&)> cost_function);

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
  std::vector<Eigen::Vector4d> getSpaceTimePath()
  {
      return path_pos_t_;
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
        dyn_voro_arr_[cell.z_cm]->idxToPos(IntPoint(cell.x, cell.y), map_2d_pos);

        Eigen::Vector4d map_spacetime_pos{map_2d_pos.x, map_2d_pos.y, cell.z_m, double(cell.t)};
        // Block out the same cell at (t-1) to prevent swapping conflicts
        Eigen::Vector4d map_spacetime_pos_prev{map_2d_pos.x, map_2d_pos.y, cell.z_m, (double)(cell.t-1)};

        // Add space time map position to path
        path_pos_t_.push_back(map_spacetime_pos);
        // Add map position to reservation table
        resrv_tbl_->insert(map_spacetime_pos);
        resrv_tbl_->insert(map_spacetime_pos_prev);
      }

  }

public:

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

  std::unordered_map<VCell, VCell> came_from_v_;
  PriorityQueue<VCell, double> open_list_v_; // Min priority queue 
  std::unordered_set<VCell> closed_list_v_; // All closed nodes

  std::vector<VCell> path_idx_v_; // Final planned Path in terms of indices

  // Space time voronoi Voronoi search data structures
  
  std::vector<Eigen::Vector4d> path_pos_t_; // Spatial-temporal path 



  std::unordered_map<VCell, double> g_cost_v_;  
  // std::unordered_map<VCell, double> g_cost_vt_;  

  std::unordered_map<VCell_T, VCell_T> came_from_vt_;
  PriorityQueue<VCell_T, double> open_list_vt_; // Min priority queue 
  std::unordered_set<VCell_T> closed_list_vt_; // All closed nodes

  std::vector<VCell_T> path_idx_vt_; // Final planned Path in terms of indices

  std::shared_ptr<std::unordered_set<Eigen::Vector4d>> resrv_tbl_; // Reservation table 
};

#endif // _A_STAR_PLANNER_H_