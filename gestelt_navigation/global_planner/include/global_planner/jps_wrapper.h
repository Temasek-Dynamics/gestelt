#ifndef _JPS_WRAPPER_H_
#define _JPS_WRAPPER_H_

#include <global_planner/planner_base.h>

#include <ros/ros.h>

#include <grid_map/grid_map.h>

#include <jps_planner/jps_planner/jps_planner.h>
#include <jps_planner/distance_map_planner/distance_map_planner.h>

#include <logger/timer.h>

class JPSWrapper : public PlannerBase
{
public:

  struct JPSParams{
    int max_iterations; // Maximum iterations for Astar to run
    double tie_breaker;
    bool debug_viz; // Publish visualization messages for debugging 
    int cost_function_type; // Type of cost function to use
    bool print_timers{false};
    bool planner_verbose{false};

    bool use_dmp{false}; // True if using DMP
    bool interpolate{true}; // True if interpolating JPS

    double dmp_search_rad{1.5}; // DMP search radius
    double dmp_pot_rad{2.0};    // DMP potential radius
    double dmp_col_weight{1.0}; // DMP Collision weight
    double dmp_heuristic_weight{0.1}; // Heuristic weight
    int dmp_pow{1};                   // DMP power index for creating mask
  }; // struct SphericalSFCParams

  JPSWrapper(std::shared_ptr<GridMap> map, const JPSParams& jps_params);

  // Add ROS Publishers
  void addPublishers(std::unordered_map<std::string, ros::Publisher> &publisher_map);

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

  std::vector<Eigen::Vector3d> interpolatePath(const std::vector<Eigen::Vector3d>& path);

  /**
   * @brief Get successful JPS+DMP plan in terms of path positions (x,y,z)
   *
   * @return std::vector<Eigen::Vector3d>
   */
  std::vector<Eigen::Vector3d> getPathPos()
  {
    if (params_.use_dmp){
      return path_dmp_;
    }
    else {
      if (params_.interpolate){
        return path_jps_intp_;
      }
      return path_jps_;
    }
  }

  /**
   * @brief Get successful JPS+DMP plan in terms of path positions (x,y,z)
   *
   * @return std::vector<Eigen::Vector3d>
   */
  std::vector<Eigen::Vector3d> getPathPosRaw()
  {
    if (params_.use_dmp){
      return path_dmp_raw_;
    }
    else {
      if (params_.interpolate){
        return path_jps_intp_;
      }
      return path_jps_raw_;
    }
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

  // // Bresenham 3d line algorithm for interpolating linear piecewise paths
  // std::vector<Eigen::Vector3d> Bresenham3D(
  //   const Eigen::Vector3d& pt_a, const Eigen::Vector3d& pt_b, const double& step){
      
  //     // Bresenham3D
  //     //
  //     // A slightly modified version of the source found at
  //     // http://www.ict.griffith.edu.au/anthony/info/graphics/bresenham.procs
  //     // Provided by Anthony Thyssen, though he does not take credit for the original implementation
  //     //
  //     // It is highly likely that the original Author was Bob Pendelton, as referenced here
  //     //
  //     // ftp://ftp.isc.org/pub/usenet/comp.sources.unix/volume26/line3d
  //     //
  //     // line3d was dervied from DigitalLine.c published as "Digital Line Drawing"
  //     // by Paul Heckbert from "Graphics Gems", Academic Press, 1990
  //     //
  //     // 3D modifications by Bob Pendleton. The original source code was in the public
  //     // domain, the author of the 3D version places his modifications in the
  //     // public domain as well.
  //     //
  //     // line3d uses Bresenham's algorithm to generate the 3 dimensional points on a
  //     // line from (x1, y1, z1) to (x2, y2, z2)
  //     //

  //     std::vector<Eigen::Vector3d> line;

  //     int x1 =  pt_a(0);
  //     int y1 =  pt_a(1);
  //     int z1 =  pt_a(2);

  //     int x2 =  pt_b(0);
  //     int y2 =  pt_b(1);
  //     int z2 =  pt_b(2);

  //     int i, dx, dy, dz, l, m, n, x_inc, y_inc, z_inc, err_1, err_2, dx2, dy2, dz2;
  //     int point[3];
      
  //     point[0] = x1;
  //     point[1] = y1;
  //     point[2] = z1;
  //     dx = x2 - x1;
  //     dy = y2 - y1;
  //     dz = z2 - z1;
  //     x_inc = (dx < 0) ? -1 : 1;
  //     l = abs(dx);
  //     y_inc = (dy < 0) ? -1 : 1;
  //     m = abs(dy);
  //     z_inc = (dz < 0) ? -1 : 1;
  //     n = abs(dz);
  //     dx2 = l << 1;
  //     dy2 = m << 1;
  //     dz2 = n << 1;
      
  //     if ((l >= m) && (l >= n)) {
  //         err_1 = dy2 - l;
  //         err_2 = dz2 - l;
  //         for (i = 0; i < l; i++) {
  //             line.push_back(Eigen:Vector3d{point[0], point[1], point[2]});
  //             if (err_1 > 0) {
  //                 point[1] += y_inc;
  //                 err_1 -= dx2;
  //             }
  //             if (err_2 > 0) {
  //                 point[2] += z_inc;
  //                 err_2 -= dx2;
  //             }
  //             err_1 += dy2;
  //             err_2 += dz2;
  //             point[0] += x_inc;
  //         }
  //     } else if ((m >= l) && (m >= n)) {
  //         err_1 = dx2 - m;
  //         err_2 = dz2 - m;
  //         for (i = 0; i < m; i++) {
  //             line.push_back(Eigen:Vector3d{point[0], point[1], point[2]});
  //             if (err_1 > 0) {
  //                 point[0] += x_inc;
  //                 err_1 -= dy2;
  //             }
  //             if (err_2 > 0) {
  //                 point[2] += z_inc;
  //                 err_2 -= dy2;
  //             }
  //             err_1 += dx2;
  //             err_2 += dz2;
  //             point[1] += y_inc;
  //         }
  //     } else {
  //         err_1 = dy2 - n;
  //         err_2 = dx2 - n;
  //         for (i = 0; i < n; i++) {
  //             line.push_back(Eigen:Vector3d{point[0], point[1], point[2]});
  //             if (err_1 > 0) {
  //                 point[1] += y_inc;
  //                 err_1 -= dz2;
  //             }
  //             if (err_2 > 0) {
  //                 point[0] += x_inc;
  //                 err_2 -= dz2;
  //             }
  //             err_1 += dy2;
  //             err_2 += dx2;
  //             point[2] += z_inc;
  //         }
  //     }
  //     line.push_back(Eigen:Vector3d{point[0], point[1], point[2]});
  // }

private:
  std::vector<Eigen::Vector3d> path_jps_;               // JPS Path in (x,y,z)
  std::vector<Eigen::Vector3d> path_jps_raw_;           // JPS Path raw
  std::vector<Eigen::Vector3d> path_jps_intp_;   // JPS Path interpolated
  std::vector<Eigen::Vector3d> path_dmp_;               // DMP Path in (x,y,z)
  std::vector<Eigen::Vector3d> path_dmp_raw_;           // DMP Path in (x,y,z)
  std::vector<Eigen::Vector3d> dmp_search_region_;      // Search region of DMP

  /* Params */
  JPSParams params_;

  std::shared_ptr<GridMap> map_;

  std::shared_ptr<JPSPlanner3D> jps_planner_;
  
  Timer tm_jps_map_{"jps_map", 3};
  Timer tm_jps_plan_{"jps_plan", 3};
  Timer tm_dmp_plan_{"dmp_plan", 3};

}; // class JPSWrapper

#endif // _JPS_WRAPPER_H_