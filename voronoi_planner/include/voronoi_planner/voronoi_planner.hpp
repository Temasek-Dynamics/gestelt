#ifndef _VORONOI_PLANNER_HPP
#define _VORONOI_PLANNER_HPP

#include "viz_helper.hpp"

#include <Eigen/Eigen>
#include <limits>
#include <queue>

#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <gestelt_msgs/BoolMapArray.h>
#include <gestelt_msgs/PlanRequestDebug.h>
#include <gestelt_msgs/FrontEndPlan.h>
#include <gestelt_msgs/Goals.h>

#include <grid_map/grid_map.h> // Map representation

#include "dynamic_voronoi/dynamicvoronoi.h"
#include "global_planner/a_star.h"

#include <logger/timer.h>

// We use SDL_image to load the image from disk
// #include <SDL/SDL_image.h>

namespace cost_val
{
static const int8_t OCC = 100;
static const int8_t FREE = 0;
static const int8_t UNKNOWN = -1;
}

#define INF std::numeric_limits<double>::max()

template<typename ... Args>
std::string str_fmt( const std::string& format, Args ... args )
{
  int size_s = std::snprintf( nullptr, 0, format.c_str(), args ... ) + 1; // Extra space for '\0'
  if( size_s <= 0 ){ throw std::runtime_error( "Error during formatting." ); }
  auto size = static_cast<size_t>( size_s );
  std::unique_ptr<char[]> buf( new char[ size ] );
  std::snprintf( buf.get(), size, format.c_str(), args ... );
  return std::string( buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
}

class Waypoint
{
// Waypoint class is a LIFO queue 

public:
  // Default constructor
  Waypoint(){
  }

  // Reset
  void reset(){
    wp_queue.clear();
  }
  
  /**
   * @brief Add multiple waypoints
   * 
   * @param wp 
   * @return true 
   * @return false 
   */
  bool addMultipleWP(const std::vector<Eigen::Vector3d>& wp_vec){
    // Reverse the waypoints and add on top of the stack
    std::vector<Eigen::Vector3d> wp_vec_reversed = wp_vec;
    std::reverse(wp_vec_reversed.begin(), wp_vec_reversed.end());

    for (auto wp : wp_vec_reversed){
      wp_queue.push_back(wp);
    }

    return true;
  }

  /**
   * @brief Add a single waypoint
   * 
   * @param wp 
   * @return true 
   * @return false 
   */
  bool addWP(const Eigen::Vector3d& wp){
    wp_queue.push_back(wp);
    return true;
  }

  /* Getter methods */

  /**
   * @brief Get the next waypoint
   * 
   * @return Eigen::Vector3d 
   */
  const Eigen::Vector3d& nextWP(){
    return wp_queue.back();
  }

  /**
   * @brief Get all waypoints as a vector
   * 
   * @return const Eigen::Vector3d& 
   */
  const std::vector<Eigen::Vector3d>& getQueue(){
    return wp_queue;
  }

  /**
   * @brief Get the size of the queue
   * 
   * @return Eigen::Vector3d 
   */
  size_t size() const {
    return wp_queue.size();
  }

  /**
   * @brief Get the size of the queue
   * 
   * @return Eigen::Vector3d 
   */
  bool empty() const {
    return wp_queue.empty();
  }

  /* Setter methods */

  /**
   * @brief Pop the last waypoint
   * 
   */
  void popWP() {
    if (wp_queue.empty()){
        return;
    }
    else {
      wp_queue.pop_back();
    }
  }

private:
  std::vector<Eigen::Vector3d> wp_queue;
};

class VoronoiPlanner
{
public:
  // VoronoiPlanner(){}

  void init(ros::NodeHandle &nh, ros::NodeHandle &pnh);

  void initParams(ros::NodeHandle &pnh);

  /**
   * @brief Plan a path from start to goal
   * 
   * @param start 
   * @param goal 
   * @return true 
   * @return false 
   */
  bool plan(const Eigen::Vector3d& start, const Eigen::Vector3d& goal);

/* Subscriber callbacks */
private:

  /* Timer for front-end planner*/
  void planFETimerCB(const ros::TimerEvent &e);

  /* Generate voronoi map timer callback*/
  void genVoroMapTimerCB(const ros::TimerEvent &e);

  /* Front end plan subscription */
  void FEPlanSubCB(const gestelt_msgs::FrontEndPlanConstPtr& msg);

  /* Plan request (for debug use)*/
  void planReqDbgCB(const gestelt_msgs::PlanRequestDebugConstPtr &msg);

  /* Subscription callback to goals */
  void goalsCB(const gestelt_msgs::GoalsConstPtr &msg);

  /* Subscription callback to odometry */
  void odometryCB(const nav_msgs::OdometryConstPtr &msg);

/* Helper methods */
private:

  bool isGoalReached(const Eigen::Vector3d& pos, const Eigen::Vector3d& goal);

  inline size_t map2Dto1DIdx(const int& width, const int& x, const int& y)
  {
    return width * y + x;
  }

  inline void map1Dto2DIdx(const int& idx, const int& width, int& x, int& y)
  {
    y = idx/width;
    x = idx - (y * width);
  }

  // Convert from map to occupancy grid type
  void voronoimapToOccGrid( const DynamicVoronoi& dyn_voro, 
                            const double& origin_x, const double& origin_y, 
                            nav_msgs::OccupancyGrid& occ_grid)
  {
    occ_grid.header.stamp = ros::Time::now();
    occ_grid.header.frame_id = "map";
    occ_grid.info.width = dyn_voro.getSizeX();
    occ_grid.info.height = dyn_voro.getSizeY();
    occ_grid.info.resolution = bool_map_3d_.resolution;
    occ_grid.info.origin.position.x = origin_x;
    occ_grid.info.origin.position.y = origin_y;
    occ_grid.info.origin.position.z = dyn_voro.getOriginZ();
    tf2::Quaternion q;
    q.setRPY(0, 0, 0.0);
    occ_grid.info.origin.orientation.x = q.x();
    occ_grid.info.origin.orientation.y = q.y();
    occ_grid.info.origin.orientation.z = q.z();
    occ_grid.info.origin.orientation.w = q.w();

    occ_grid.data.resize(occ_grid.info.width * occ_grid.info.height);

    for(int j = 0; j < dyn_voro.getSizeY(); j++)
    {
      for (int i = 0; i < dyn_voro.getSizeX(); i++)
      {
        size_t idx = map2Dto1DIdx(occ_grid.info.width, i, j);
        occ_grid.data[idx] = dyn_voro.isVoronoi(i, j) ? 255: 0;
      }
    }
  }

  // Convert from map to occupancy grid type
  void occmapToOccGrid(const DynamicVoronoi& dyn_voro, 
                      const double& origin_x, const double& origin_y,
                      nav_msgs::OccupancyGrid& occ_grid)
  {
    occ_grid.header.stamp = ros::Time::now();
    occ_grid.header.frame_id = "map";
    occ_grid.info.width = dyn_voro.getSizeX();
    occ_grid.info.height = dyn_voro.getSizeY();
    occ_grid.info.resolution = bool_map_3d_.resolution;
    occ_grid.info.origin.position.x = origin_x;
    occ_grid.info.origin.position.y = origin_y;
    occ_grid.info.origin.position.z = dyn_voro.getOriginZ();
    tf2::Quaternion q;
    q.setRPY(0, 0, 0.0);
    occ_grid.info.origin.orientation.x = q.x();
    occ_grid.info.origin.orientation.y = q.y();
    occ_grid.info.origin.orientation.z = q.z();
    occ_grid.info.origin.orientation.w = q.w();

    occ_grid.data.resize(occ_grid.info.width * occ_grid.info.height);

    for(int j = 0; j < dyn_voro.getSizeY(); j++)
    {
      for (int i = 0; i < dyn_voro.getSizeX(); i++)
      {
        size_t idx = map2Dto1DIdx(occ_grid.info.width, i, j);

        // Flip the coordinates vertically (Occ grid uses top left as origin but original map data uses bottom left as origin)
        // size_t idx = map2Dto1DIdx(occ_grid.info.width, i, occ_grid.info.height - j - 1);
        occ_grid.data[idx] = dyn_voro.isOccupied(i, j) ? 255: 0;
      }
    }
  }

  /* Convert from time [s] to space-time units */
  long tToSpaceTimeUnits(const double& t){
    return std::lround(t / t_unit_);
  }

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

    if (num > bool_map_3d_.max_height_cm){
      return bool_map_3d_.max_height_cm;
    }

    if (num < bool_map_3d_.min_height_cm){
      return bool_map_3d_.min_height_cm;
    }

    int rem = (int)num % mult;
    if (rem == 0){
      return num;
    }

    return rem < (mult/2) ? (num-rem) : (num-rem) + mult;
  }

  // Convert from meters to centimeters
  int mToCm(const double& val_m){
    return (int) (val_m * 100.0);
  }

  // Convert from centimeters to meters
  double cmToM(const int& val_cm) {
    return ((double) val_cm)/100.0;  
  }

private:
  /* Params */
  std::string node_name_{"VoronoiPlanner"};
  int drone_id_{-1};
  int num_agents_; // Number of agents

  std::string local_map_origin_;
  std::string global_origin_;

  // Planning params
  double t_unit_{0.1}; // [s] Time duration of each space-time A* unit
  double fe_planner_freq_{10}; // [Hz] Frequency for front-end planning
  double gen_voro_map_freq_{20}; // [Hz] Frequency for front-end planning
  double sqr_goal_tol_{0.1}; // Distance to goal before it is considered fulfilled.
  bool plan_once_{false}; // Used for testing, only runs the planner once
  bool verbose_print_{false};  // enables printing of planning time

  double resrv_tbl_inflation_{-1.0}; // Inflation of cells in the reservation table
  double resrv_tbl_t_buffer_{-1.0}; // Time buffer in the reservation table

  AStarPlanner::AStarParams astar_params_; 

  /* Timers */
  ros::Timer plan_fe_timer_;
  ros::Timer gen_voro_map_timer_;

  /* Publishers */
  ros::Publisher occ_map_pub_;      // Publishes original occupancy grid
  ros::Publisher voro_occ_grid_pub_; // Publishes voronoi map occupancy grid
  ros::Publisher voronoi_graph_pub_; // publisher of voronoi graph vertices

  // Planning publishers
  ros::Publisher fe_closed_list_pub_; // Closed list publishers
  ros::Publisher fe_plan_viz_pub_; // Publish front-end plan visualization
  ros::Publisher fe_plan_pub_; // Publish front-end plans
  ros::Publisher start_pt_pub_, goal_pt_pub_; // start and goal visualization publisher
  ros::Publisher fe_plan_broadcast_pub_; // Publish front-end plans broadcasted to other agents

  /* Subscribers */
  ros::Subscriber plan_req_dbg_sub_;  // plan request (start and goal) debug subscriber
  ros::Subscriber goals_sub_;  // goal subscriber
  // ros::Subscriber bool_map_sub_; // Subscription to boolean map
  ros::Subscriber fe_plan_broadcast_sub_; // Subscription to broadcasted front end plan from other agents

  ros::Subscriber odom_sub_; // Subscriber to odometry

  /* Mutexes*/
  std::mutex resrv_tbl_mtx_;
  std::mutex voro_map_mtx_;
  std::mutex cur_state_mtx_;

  /* Mapping */
  std::shared_ptr<GridMap> map_;  // Occupancy map object

  BoolMap3D bool_map_3d_; // Bool map slices 

  /* Data structs */
  std::unique_ptr<AStarPlanner> fe_planner_; // Front end planner

  // map{drone_id : unordered_set{(x,y,z,t)}}
  std::map<int, std::unordered_set<Eigen::Vector4i>> resrv_tbl_; // Reservation table of (x,y,z_cm, t) where x,y are grid positions, z_cm is height in centimeters and t is space time units

  Waypoint waypoints_; // Goal waypoint handler object

  std::map<int, std::shared_ptr<DynamicVoronoi>> dyn_voro_arr_; // array of voronoi objects with key of height (cm)

  std::vector<Eigen::Vector3d> front_end_path_; // Front-end Space path in space coordinates (x,y,z) in world frame
  std::vector<Eigen::Vector4d> space_time_path_; // Front-end Space time  path in space-time coordinates (x,y,z,t) in world frame
  std::vector<Eigen::Vector3d> smoothed_path_; // Front end smoothed path in space coordinates (x,y,z) in world frame
  std::vector<Eigen::Vector4d> smoothed_path_t_; // Front end smoothed space-time path in space coordinates (x,y,z) in world frame


  Eigen::Vector3d cur_pos_, cur_vel_;   // [LOCAL FRAME] current state

  /* Flags*/
  bool init_voro_maps_{false}; // flag to indicate if voronoi map is initialized

  /* Debugging */
  Timer tm_front_end_plan_{"front_end_plan"};
  Timer tm_voro_map_init_{"voro_map_init"};

}; // class VoronoiPlanner

  // void realignBoolMap(bool ***map, bool ***map_og, int& size_x, int& size_y)
  // {
  //   for (int x=0; x<size_x; x++) {
  //     (*map)[x] = new bool[size_y];
  //   }

  //   for(int j = 0; j < size_y; j++)
  //   {
  //     for (int i = 0; i < size_x; i++)
  //     {
  //       (*map)[i][j] = (*map_og)[i][size_y-j-1];
  //     }
  //   }
  // }

#endif // _VORONOI_PLANNER_HPP


