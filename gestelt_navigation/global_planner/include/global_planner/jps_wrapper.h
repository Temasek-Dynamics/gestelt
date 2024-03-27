#ifndef _JPS_WRAPPER_H_
#define _JPS_WRAPPER_H_

#include <ros/ros.h>

#include <grid_map/grid_map.h>

#include <jps_planner/jps_planner/jps_planner.h>
#include <jps_planner/distance_map_planner/distance_map_planner.h>

#include <chrono>

class Timer
{
public:
  Timer(const std::string& name, bool autostart = false)
  : name_(name)
  {
    if (autostart){
      start();
    }
  } 

  // start timer
  bool start() {
    if (timer_running_){ // True if timer is not running
      std::cout << "Timer " << name_ << " is already running, ignoring start() function call!" << std::endl;
      return false;
    }

    std::lock_guard<std::mutex> cmd_guard(timer_mutex_);

    t_start_cpu_ = std::chrono::high_resolution_clock::now();
    t_start_wall_ = std::chrono::system_clock::now();
    timer_running_ = true;

    return true;
  }

  // stop timer
  bool stop(bool print_dur = false) {
    if (!timer_running_){ // True if timer is not running
      std::cout << "Timer " << name_ << " is not running, ignoring stop() function call!" << std::endl;
      return false;
    }

    std::lock_guard<std::mutex> cmd_guard(timer_mutex_);

    double t_dur_cpu = std::chrono::duration_cast<std::chrono::duration<double>>(
            std::chrono::high_resolution_clock::now() - t_start_cpu_).count() * 1000.0;
    double t_dur_wall = std::chrono::duration_cast<std::chrono::duration<double>>(
            std::chrono::system_clock::now() - t_start_wall_).count() * 1000.0;

    timer_running_ = false;

    t_cum_dur_cpu_ += t_dur_cpu;
    t_cum_dur_wall_ += t_dur_wall;

    if (print_dur) {
      // std::cout << "Timer(" << name_ << ")-cpu: " << t_dur_cpu << " ms" << std::endl;
      std::cout << "Timer(" << name_ << ")-wall: " << t_dur_wall << " ms" << std::endl;
    }

    return true;
  }

  // Get cumulative CPU Ticks 
  double getCPUCumulative(bool print_dur = false) const {
    if (print_dur) {
      std::cout << "Timer("<< name_ << ")-cum,cpu: " << t_cum_dur_cpu_ << " ms" << std::endl;
    }

    return t_cum_dur_cpu_;
  }

  // Get cumulative Wall time
  double getWallCumulative(bool print_dur = false) const {
    if (print_dur) {
      std::cout << "Timer("<< name_ << ")-cum,wall: " << t_cum_dur_wall_ << " ms" << std::endl;
    }

    return t_cum_dur_wall_;
  }

private:
  std::string name_; // name of timer
  bool timer_running_{false}; // Indicates if timer is running

  std::chrono::time_point<std::chrono::high_resolution_clock> t_start_cpu_; // cpu time 
  std::chrono::time_point<std::chrono::system_clock>          t_start_wall_; // wall time

  double t_cum_dur_cpu_{0.0}; // Accumulative duration in ms
  double t_cum_dur_wall_{0.0}; // Accumulative duration in ms

  std::mutex timer_mutex_; // mutex for timer
  
}; // class Timer

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
  std::vector<Eigen::Vector3d> dmp_search_region_; // Search region of DMP

  /* Params */
  JPSParams params_;

  std::shared_ptr<GridMap> map_;

  // std::shared_ptr<path_finding_util::GraphSearch> jps_planner_;
  std::shared_ptr<JPSPlanner3D> jps_planner_;
  
  Timer tm_jps_map_{"jps_map"};
  Timer tm_jps_plan_{"jps_plan"};
  Timer tm_dmp_plan_{"dmp_plan"};

}; // class JPSWrapper

#endif // _JPS_WRAPPER_H_