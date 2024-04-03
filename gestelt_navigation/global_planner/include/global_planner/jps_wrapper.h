#ifndef _JPS_WRAPPER_H_
#define _JPS_WRAPPER_H_

#include <ros/ros.h>

#include <grid_map/grid_map.h>

#include <jps_planner/jps_planner/jps_planner.h>
#include <jps_planner/distance_map_planner/distance_map_planner.h>

#include <logger/timer.h>

class JPSWrapper
{
public:

  struct JPSParams{
    int max_iterations; // Maximum iterations for Astar to run
    double tie_breaker;
    bool debug_viz; // Publish visualization messages for debugging 
    int cost_function_type; // Type of cost function to use
    bool print_timers{true};
    bool planner_verbose{true};
    double dmp_search_rad{1.5}; // DMP search radius
    double dmp_pot_rad{2.0};    // DMP potential radius
    double dmp_col_weight{1.0}; // DMP Collision weight
    double dmp_heuristic_weight{0.1}; // Heuristic weight
    int dmp_pow{1};                   // DMP power index for creating mask
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
   * @brief Get successful JPS+DMP plan in terms of path positions (x,y,z)
   *
   * @return std::vector<Eigen::Vector3d>
   */
  std::vector<Eigen::Vector3d> getPathPos()
  {
    return path_dmp_;
  }

  /**
   * @brief Get successful JPS+DMP plan in terms of path positions (x,y,z)
   *
   * @return std::vector<Eigen::Vector3d>
   */
  std::vector<Eigen::Vector3d> getPathPosRaw()
  {
    return path_dmp_raw_;
  }


  /**
   * @brief Get successful JPS plan in terms of path positions (x,y,z)
   *
   * @return std::vector<Eigen::Vector3d>
   */
  std::vector<Eigen::Vector3d> getPathRaw()
  {
    return path_jps_;
  }

  /**
   * @brief Get DMP search region
   *
   * @return std::vector<Eigen::Vector3d>
   */
  std::vector<Eigen::Vector3d> getDMPSearchRegion()
  {
    return dmp_search_region_;
  }

private:
  std::vector<Eigen::Vector3d> path_jps_; // JPS Path in (x,y,z)
  std::vector<Eigen::Vector3d> path_dmp_; // DMP Path in (x,y,z)
  std::vector<Eigen::Vector3d> path_dmp_raw_; // DMP Path in (x,y,z)
  std::vector<Eigen::Vector3d> dmp_search_region_; // Search region of DMP

  /* Params */
  JPSParams params_;

  std::shared_ptr<GridMap> map_;

  // std::shared_ptr<path_finding_util::GraphSearch> jps_planner_;
  std::shared_ptr<JPSPlanner3D> jps_planner_;
  
  Timer tm_jps_map_{"jps_map", 3};
  Timer tm_jps_plan_{"jps_plan", 3};
  Timer tm_dmp_plan_{"dmp_plan", 3};

  Timer tm_a_{"a",2};
  Timer tm_b_{"b",2};
  Timer tm_c_{"c",2};
  Timer tm_d_{"d",2};

}; // class JPSWrapper

#endif // _JPS_WRAPPER_H_