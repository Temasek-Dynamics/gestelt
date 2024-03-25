#ifndef _JPSWRAPPER_H_
#define _JPSWRAPPER_H_

#include <ros/ros.h>

#include <grid_map/grid_map.h>

#include <jps_planner/jps_planner/jps_planner.h>
#include <jps_planner/distance_map_planner/distance_map_planner.h>

#include <chrono/chrono>

class JPSWrapper
{
public:

  struct JPSParams{
    int max_iterations; // Maximum iterations for Astar to run
    double tie_breaker;
    bool debug_viz; // Publish visualization messages for debugging 
    int cost_function_type; // Type of cost function to use
  }; // struct SphericalSFCParams

  JPSWrapper(std::shared_ptr<GridMap> map, const JPSParams& jps_params);

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
   * @brief Get path with positions
   * 
   * @return std::vector<Eigen::Vector3d> 
   */
  std::vector<Eigen::Vector3d> getPathPos();

private:
  std::vector<Eigen::Vector3d> path_pos_; // Path in terms of 3d position

  /* Params */
  bool planner_verbose_ = true;
  JPSParams jps_params_;

  std::shared_ptr<GridMap> map_;

  // std::shared_ptr<path_finding_util::GraphSearch> jps_planner_;
  std::shared_ptr<JPSPlanner3D> jps_planner_;

}; // class JPSWrapper

#endif // _JPSWRAPPER_H_