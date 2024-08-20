#ifndef _A_STAR_PLANNER_H_
#define _A_STAR_PLANNER_H_

#include <global_planner/planner_common.h>
#include <grid_map/grid_map.h>
#include "dynamic_voronoi/dynamicvoronoi.h"

#include <Eigen/Eigen>

#include <unordered_set>
#include <queue>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <chrono>

class AStarPlanner 
{
public:

  struct AStarParams{
    int drone_id{-1};
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

    double t_unit{0.1};          // [s] Time duration of each space-time A* unit

    // int st_space_diag{3};   // Number of space-time A* units to move from (0,0,0) to (1,1,1) in 26-connected 3D grid
    // int st_face_diag{2};    // Number of space-time A* units to move from (0,0,0) to (0,1,1) in 26-connected 3D grid
    int st_straight{1};     // Number of space-time A* units to move from (0,0,0) to (0,1,0) in 26-connected 3D grid
    // int st_wait{1};         // Number of space-time A* units to wait in the same cell

  }; // struct AStarParams

  AStarPlanner(std::shared_ptr<GridMap> grid_map, const AStarParams& astar_params);

  AStarPlanner( const AStarParams& astar_params);

  /**
   * @brief Clear closed, open list and reset planning_successful flag for 
   * new plan generation
   * 
   */
  void reset();

  /* Assign voronoi map. To be executed when map is updated*/
  void assignVoroMap(const std::map<int, std::shared_ptr<DynamicVoronoi>>& dyn_voro_arr,
                    const int& z_separation_cm, 
                    const double& local_origin_x,
                    const double& local_origin_y,
                    const double& max_height,
                    const double& min_height,
                    const double& res);


  /* Update the assignement of the reservation table. TO be executed when the reservation table is updated*/
  void updateReservationTable(const std::map<int, std::unordered_set<Eigen::Vector4i>>& resrv_tbl);

  /* Generate space-time plan on voronoi graph  */
  bool generatePlanVoroT( const Eigen::Vector3d& start_pos_3d, 
                          const Eigen::Vector3d& goal_pos_3d);

private: 

  // Expand voronio bubble around given cell
  void expandVoronoiBubbleT(const VCell_T& origin_cell);

  /* Generate space-time plan on voronoi graph  */
  bool generatePlanVoroT( const Eigen::Vector3d& start_pos_3d, 
                          const Eigen::Vector3d& goal_pos_3d, 
                          std::function<double(const VCell_T&, const VCell_T&)> cost_function);

  void tracePathVoroT(const VCell_T& final_node)
  {
    // Clear existing data structures
    path_idx_vt_.clear();

    path_pos_t_.clear();
    path_pos_.clear();

    path_idx_smoothed_t_.clear();
    path_smoothed_.clear();
    path_smoothed_t_.clear();

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
      path_pos_.push_back(Eigen::Vector3d{map_2d_pos.x + local_origin_x_, 
                                            map_2d_pos.y + local_origin_y_, cell.z_m});
    }

    // /* Get smoothed path */

    // // For each gridnode, get the position and index,
    // // So we can obtain a path in terms of indices and positions
    // path_idx_smoothed_t_.push_back(path_idx_vt_[0]);
    // for (size_t i = 1; i < path_idx_vt_.size()-1; i++)
    // {
    //   if (path_idx_smoothed_t_.back().z_cm != path_idx_vt_[i].z_cm){ // If different height
    //     // Add current point to smoothed path
    //     path_idx_smoothed_t_.push_back(path_idx_vt_[i]);
    //   }
    //   else {
    //     IntPoint a(path_idx_smoothed_t_.back().x, path_idx_smoothed_t_.back().y);
    //     IntPoint b(path_idx_vt_[i].x, path_idx_vt_[i].y);

    //     if (!lineOfSight(a, b, path_idx_smoothed_t_.back().z_cm )){ // If no line of sight
    //       // Add current point to smoothed path
    //       path_idx_smoothed_t_.push_back(path_idx_vt_[i]);
    //     }
    //   }
    // }
    // path_idx_smoothed_t_.push_back(path_idx_vt_.back());

    // // For each gridnode, get the position and index,
    // // So we can obtain a path in terms of indices and positions
    // for (const VCell_T& cell : path_idx_smoothed_t_)
    // {
    //   DblPoint map_2d_pos;

    //   // Convert to map position
    //   dyn_voro_arr_[cell.z_cm]->idxToPos(IntPoint(cell.x, cell.y), map_2d_pos);
    //   // Add space time map position to path and transform it from local map to world frame
    //   path_smoothed_t_.push_back(Eigen::Vector4d{map_2d_pos.x + local_origin_x_, 
    //                                         map_2d_pos.y + local_origin_y_, cell.z_m, (double) cell.t});
    //   path_smoothed_.push_back(Eigen::Vector3d{map_2d_pos.x + local_origin_x_, 
    //                                         map_2d_pos.y + local_origin_y_, cell.z_m});
    // }
  }

/* Getter methods */
public:
  /**
   * @brief Get successful plan in terms of space i,e. (x,y,z)
   *
   * @return std::vector<Eigen::Vector3d>
   */
  std::vector<Eigen::Vector3d> getPath()
  {
    return path_pos_;
  }

  /**
   * @brief Get successful plan in terms of space and time i.e. (x,y,z,t)
   *
   * @return std::vector<Eigen::Vector3d>
   */
  std::vector<Eigen::Vector4d> getPathWithTime()
  {
    return path_pos_t_;
  }


  /**
   * @brief Get successful plan in terms of space i,e. (x,y,z)
   *
   * @return std::vector<Eigen::Vector3d>
   */
  std::vector<Eigen::Vector3d> getPath(const Eigen::Vector3d& cur_pos)
  {
    std::vector<Eigen::Vector3d> path_pos = path_pos_;

    // convert from local frame to world frame
    Eigen::Vector3d start_pos(cur_pos(0)+local_origin_x_, 
                              cur_pos(1)+local_origin_y_, 
                              cur_pos(2));
    path_pos.insert(path_pos.begin(), start_pos);

    return path_pos;
  }


  /**
   * @brief Get successful plan in terms of space and time i.e. (x,y,z,t)
   *
   * @return std::vector<Eigen::Vector4d>
   */
  std::vector<Eigen::Vector4d> getPathWithTime(const Eigen::Vector3d& cur_pos)
  {
    std::vector<Eigen::Vector4d> path_pos_t = path_pos_t_;

    Eigen::Vector4d start_t(cur_pos(0)+local_origin_x_, 
                            cur_pos(1)+local_origin_y_, 
                            cur_pos(2), 
                            0);
    path_pos_t.insert(path_pos_t.begin(), start_t);

    // offset_t: time from current position to start of path
    double offset_t = (int)round((start_t.segment(0,3) - path_pos_t[0].segment(0,3)).norm()/res_) * astar_params_.st_straight; 

    for (size_t i = 1; i < path_pos_t.size(); i++){
      path_pos_t[i](3) += offset_t;
    }

    return path_pos_t;
  }

  /* Get post-smoothed path */
  std::vector<Eigen::Vector4d> getSmoothedPathWithTime()
  {
    return path_smoothed_t_;
  }

  /* Get post-smoothed path */
  std::vector<Eigen::Vector3d> getSmoothedPath()
  {
    return path_smoothed_;
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

/* Helper methods */
public:

  /**
   * @brief Round to nearest multiple 
   * 
   * @param num Number to be rounded
   * @param mult Multiple
   * @return int 
   */
  int roundToMultInt(const int& num, const int& mult)
  {
    if (mult == 0){
      return num;
    }

    if (num >= max_height_){
      return max_height_;
    }

    if (num <= min_height_){
      return min_height_;
    }

    int rem = (int)num % mult;
    if (rem == 0){
      return num;
    }

    return rem < (mult/2) ? (num-rem) : (num-rem) + mult;
  }

  /* Check line of sight between 2 points*/
  bool lineOfSight(IntPoint s, IntPoint s_, int z_cm);

private: 

  /* Params */
  AStarParams astar_params_;

  /* Path planner data structures */
  double local_origin_x_{0.0}, local_origin_y_{0.0};
  int max_height_{300}; // [cm]
  int min_height_{50}; // [cm]
  double res_{0.05};  // [m]

  std::map<int, std::shared_ptr<DynamicVoronoi>> dyn_voro_arr_; // array of voronoi objects with key of height (cm)

  // General voronoi params
  int z_separation_cm_; // separation between the voronoi planes in units of centimeters
  std::map<int, std::unordered_set<IntPoint>> marked_bubble_cells_; // Cells that are marked as part of the voronoi bubble with key of height(cm)

  // Space time voronoi Voronoi search data structures
  
  std::vector<Eigen::Vector3d> path_pos_; // (WORLD FRAME) Spatial path 
  std::vector<Eigen::Vector4d> path_pos_t_; // (WORLD FRAME) Spatial-temporal path 
  std::vector<Eigen::Vector3d> path_smoothed_; // (WORLD FRAME) post smoothed spatial path 
  std::vector<Eigen::Vector4d> path_smoothed_t_; // (WORLD FRAME) post smoothed spatial-temporal path 
  std::vector<VCell_T> path_idx_vt_; // (LOCAL FRAME) Final planned Path in terms of indices
  std::vector<VCell_T> path_idx_smoothed_t_; // (LOCAL FRAME) Final planned Path in terms of indices

  std::unordered_map<VCell, double> g_cost_v_;  
  std::unordered_map<VCell_T, VCell_T> came_from_vt_;
  PriorityQueue<VCell_T, double> open_list_vt_; // Min priority queue 
  std::unordered_set<VCell_T> closed_list_vt_; // All closed nodes

  // map{drone_id : unordered_set{(x,y,z,t)}}
  std::map<int, std::unordered_set<Eigen::Vector4i>> resrv_tbl_; // Reservation table of (x,y,z_cm, t) where x,y are grid positions, z_cm is height in centimeters and t is space time units
};

#endif // _A_STAR_PLANNER_H_