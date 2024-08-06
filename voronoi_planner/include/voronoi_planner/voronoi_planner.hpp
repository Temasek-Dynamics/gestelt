// Code inspired by benchmark_utils.cpp from Bonxai

#ifndef _VORONOI_PLANNER_HPP
#define _VORONOI_PLANNER_HPP

// #include <Eigen/Eigen>
#include <limits>
#include <queue>

// We use SDL_image to load the image from disk
#include <SDL/SDL_image.h>

#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <gestelt_msgs/BoolMapArray.h>
#include <gestelt_msgs/PlanRequestDebug.h>

#include <grid_map/grid_map.h> // Map representation

#include "dynamic_voronoi/dynamicvoronoi.h"
#include "global_planner/a_star.h"

#include <logger/timer.h>

namespace cost_val
{
static const int8_t OCC = 100;
static const int8_t FREE = 0;
static const int8_t UNKNOWN = -1;
}

#define INF std::numeric_limits<double>::max()
#define SQRT2 1.41421

template<typename T, typename priority_t>
struct PriorityQueueV {
  typedef std::pair<priority_t, T> PQElement;
  struct PQComp {
    constexpr bool operator()(
      PQElement const& a,
      PQElement const& b)
      const noexcept
    {
      return a.first > b.first;
    }
  };

  std::priority_queue<PQElement, std::vector<PQElement>, PQComp > elements;

  inline bool empty() const {
     return elements.empty();
  }

  inline void put(T item, priority_t priority) {
    elements.emplace(priority, item);
  }

  T get() {
    T best_item = elements.top().second;
    elements.pop();
    return best_item;
  }

  void clear() {
    elements = std::priority_queue<PQElement, std::vector<PQElement>, PQComp>();
  }
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
   * @param id 
   * @param start 
   * @param goal 
   * @return true 
   * @return false 
   */
  bool plan(const int& id, const Eigen::Vector3d& start, const Eigen::Vector3d& goal);

/* Subscriber callbacks */
private:

  void boolMapCB(const gestelt_msgs::BoolMapArrayConstPtr& msg);

  void startDebugCB(const gestelt_msgs::PlanRequestDebugConstPtr &msg);

/* Helper methods */
private:

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
    occ_grid.info.resolution = res_;
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
    occ_grid.info.resolution = res_;
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

/* Visualization methods*/
private:

  void publishStartAndGoal(
    const Eigen::Vector3d& map_start, 
    const Eigen::Vector3d& map_goal, 
    const std::string& frame_id, 
    ros::Publisher& publisher1, ros::Publisher& publisher2)
  {
    visualization_msgs::Marker start_sphere, goal_sphere;
    double radius = 0.1;
    double alpha = 0.5; 

    /* Start/goal sphere*/
    start_sphere.header.frame_id = goal_sphere.header.frame_id = frame_id;
    start_sphere.header.stamp = goal_sphere.header.stamp = ros::Time::now();
    start_sphere.ns = goal_sphere.ns = "start_goal_points";
    start_sphere.type = goal_sphere.type = visualization_msgs::Marker::CUBE;
    start_sphere.action = goal_sphere.action = visualization_msgs::Marker::ADD;
    start_sphere.id = 1;
    goal_sphere.id = 2; 
    start_sphere.pose.orientation.w = goal_sphere.pose.orientation.w = 1.0;

    start_sphere.color.r = 1.0; 
    start_sphere.color.g = 1.0; 
    start_sphere.color.b = 0.0; 
    start_sphere.color.a = goal_sphere.color.a = alpha;

    goal_sphere.color.r = 0.0;
    goal_sphere.color.g = 1.0;
    goal_sphere.color.b = 0.0;

    start_sphere.scale.x = goal_sphere.scale.x = radius;
    start_sphere.scale.y = goal_sphere.scale.y = radius;
    start_sphere.scale.z = goal_sphere.scale.z = radius;

    /* Set Start */
    start_sphere.pose.position.x = map_start(0);
    start_sphere.pose.position.y = map_start(1);
    start_sphere.pose.position.z = map_start(2);

    /* Set Goal */
    goal_sphere.pose.position.x = map_goal(0);
    goal_sphere.pose.position.y = map_goal(1);
    goal_sphere.pose.position.z = map_goal(2);

    publisher1.publish(start_sphere);
    publisher2.publish(goal_sphere);
  }


  inline void publishSpaceTimePath(const std::vector<Eigen::Vector4d>& path, 
                                  const std::string& frame_id, ros::Publisher& publisher) {
    visualization_msgs::Marker cube;
    double radius = 0.1;
    double alpha = 0.8; 

    /* Fixed parameters */
    cube.header.frame_id = frame_id;
    cube.header.stamp = ros::Time::now();
    cube.ns = "front_end_path";
    cube.type = visualization_msgs::Marker::CUBE;
    cube.action = visualization_msgs::Marker::ADD;
    cube.pose.orientation.w = 1.0;
    cube.color.a = alpha;

    // size
    cube.scale.x = radius;
    cube.scale.y = radius;
    cube.scale.z = radius;

    for (int i = 0; i < path.size(); i++)
    {
      cube.id = i; 

      cube.pose.position.x = path[i](0);
      cube.pose.position.y = path[i](1);
      cube.pose.position.z = path[i](2);

      // Make the color value scale from 0.0 to 1.0 depending on the distance to the goal. 
      double time_ratio = std::clamp((path[i](3))/path.size(), 0.0, 1.0);

      cube.color.r = time_ratio ;
      cube.color.g = 0.0; 
      cube.color.b = 1.0 - time_ratio; 

      publisher.publish(cube);

      ros::Duration(0.025).sleep();
    }
  }

  inline void publishFrontEndPath(const std::vector<Eigen::Vector3d>& path, 
                                  const std::string& frame_id, ros::Publisher& publisher) {
    visualization_msgs::Marker wp_sphere_list, path_line_strip;
    visualization_msgs::Marker start_sphere, goal_sphere;
    double radius = 0.3;
    double alpha = 0.8; 

    geometry_msgs::Point pt;

    /* Start/goal sphere*/
    start_sphere.header.frame_id = goal_sphere.header.frame_id = frame_id;
    start_sphere.header.stamp = goal_sphere.header.stamp = ros::Time::now();
    start_sphere.ns = goal_sphere.ns = "start_end_points";
    start_sphere.type = goal_sphere.type = visualization_msgs::Marker::SPHERE;
    start_sphere.action = goal_sphere.action = visualization_msgs::Marker::ADD;
    start_sphere.id = 0;
    goal_sphere.id = 1; 
    start_sphere.pose.orientation.w = goal_sphere.pose.orientation.w = 1.0;

    start_sphere.color.r = 1.0; 
    start_sphere.color.g = 1.0; 
    start_sphere.color.b = 0.0; 
    start_sphere.color.a = goal_sphere.color.a = alpha;

    goal_sphere.color.r = 0.0;
    goal_sphere.color.g = 1.0;
    goal_sphere.color.b = 0.0;

    start_sphere.scale.x = goal_sphere.scale.x = radius;
    start_sphere.scale.y = goal_sphere.scale.y = radius;
    start_sphere.scale.z = goal_sphere.scale.z = radius;

    /* wp_sphere_list: Sphere list (Waypoints) */
    wp_sphere_list.header.frame_id = frame_id;
    wp_sphere_list.header.stamp = ros::Time::now();
    wp_sphere_list.ns = "front_end_sphere_list"; 
    wp_sphere_list.type = visualization_msgs::Marker::SPHERE_LIST;
    wp_sphere_list.action = visualization_msgs::Marker::ADD;
    wp_sphere_list.id = 1; 
    wp_sphere_list.pose.orientation.w = 1.0;

    wp_sphere_list.color.r = 1.0;
    wp_sphere_list.color.g = 0.5;
    wp_sphere_list.color.b = 0.0;
    wp_sphere_list.color.a = alpha;

    wp_sphere_list.scale.x = radius;
    wp_sphere_list.scale.y = radius;
    wp_sphere_list.scale.z = radius;

    /* path_line_strip: Line strips (Connecting waypoints) */
    path_line_strip.header.frame_id = frame_id;
    path_line_strip.header.stamp = ros::Time::now();
    path_line_strip.ns = "front_end_path_lines"; 
    path_line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    path_line_strip.action = visualization_msgs::Marker::ADD;
    path_line_strip.id = 1;
    path_line_strip.pose.orientation.w = 1.0;

    path_line_strip.color.r = 1.0;
    path_line_strip.color.g = 0.5;
    path_line_strip.color.b = 0.0;
    path_line_strip.color.a = alpha * 0.75;

    path_line_strip.scale.x = radius * 0.5;

    start_sphere.pose.position.x = path[0](0);
    start_sphere.pose.position.y = path[0](1);
    start_sphere.pose.position.z = path[0](2);

    pt.x = path[0](0);
    pt.y = path[0](1);
    pt.z = path[0](2);
    path_line_strip.points.push_back(pt);

    for (size_t i = 1; i < path.size()-1; i++){
      pt.x = path[i](0);
      pt.y = path[i](1);
      pt.z = path[i](2);

      wp_sphere_list.points.push_back(pt);
      path_line_strip.points.push_back(pt);
    }

    pt.x = path.back()(0);
    pt.y = path.back()(1);
    pt.z = path.back()(2);
    path_line_strip.points.push_back(pt);

    goal_sphere.pose.position.x = path.back()(0);
    goal_sphere.pose.position.y = path.back()(1);
    goal_sphere.pose.position.z = path.back()(2);

    publisher.publish(start_sphere);
    publisher.publish(goal_sphere);
    publisher.publish(wp_sphere_list);
    publisher.publish(path_line_strip);
  }

  inline void publishVertices(const std::vector<Eigen::Vector3d>& vor_verts, const std::string& frame_id, ros::Publisher& publisher)
  {
    visualization_msgs::Marker vertices;
    double radius = 0.2;
    double alpha = 0.8; 

    /* vertices: Sphere list (voronoi graph vertices) */
    vertices.header.frame_id = frame_id;
    vertices.header.stamp = ros::Time::now();
    vertices.ns = "voronoi_vertices"; 
    vertices.type = visualization_msgs::Marker::SPHERE_LIST;
    vertices.action = visualization_msgs::Marker::ADD;
    vertices.id = 1; 
    vertices.pose.orientation.w = 1.0;

    vertices.color.r = 0.0;
    vertices.color.g = 0.5;
    vertices.color.b = 1.0;
    vertices.color.a = alpha;

    vertices.scale.x = radius;
    vertices.scale.y = radius;
    vertices.scale.z = radius;

    geometry_msgs::Point pt;
    for (size_t i = 0; i < vor_verts.size(); i++){
      pt.x = vor_verts[i](0);
      pt.y = vor_verts[i](1);
      pt.z = vor_verts[i](2);

      vertices.points.push_back(pt);
    }

    publisher.publish(vertices);
  }

  void publishClosedList(const std::vector<Eigen::Vector3d>& pts, 
                        ros::Publisher& publisher, const std::string& frame_id = "map")
  {
    if (pts.empty()){
      return;
    }

    Eigen::Vector3d color = Eigen::Vector3d{0.0, 0.0, 0.0};
    double radius = 0.1;
    double alpha = 0.7;

    visualization_msgs::Marker sphere_list;

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

/* Test functions */
private:
  void generateTestMap2();

private:
  /* Params */
  std::string map_fname_;
  bool verbose_planning_{false};  // enables printing of planning time
  double res_;
  // bool negate_{false};
  // double occ_th_, free_th_;
  // double yaw_;
  double critical_clr_{0.25}; // minimum clearance of drone from obstacle
  double fixed_pt_thresh_{0.3}; // points (on trajectory) below this threshold are defined as fixed points (not decision variables in optimization problem)

  bool use_test_map_{false};

  AStarPlanner::AStarParams astar_params_; 

  /* Pubs, subs */
  ros::Publisher occ_map_pub_;      // Publishes original occupancy grid
  ros::Publisher voro_occ_grid_pub_; // Publishes voronoi map occupancy grid

  std::vector<ros::Publisher> closed_list_pubs_; // Closed list publishers
  std::vector<ros::Publisher> front_end_plan_pubs_; // Publish front-end plan (A*) visualization
  ros::Publisher start_pt_pub_, goal_pt_pub_; // start and goal visualization publisher

  ros::Publisher voronoi_graph_pub_; // Voronoi graph publisher

  ros::Subscriber plan_req_dbg_sub_;  // plan request (start and goal) debug subscriber
  ros::Subscriber bool_map_sub_; // Subscription to boolean map

  /* Planning */

  /* Mapping */
  std::shared_ptr<GridMap> map_;
  double local_origin_x_{0.0}, local_origin_y_{0.0}; // Origin of local map 
  int z_separation_cm_{50};

  std::shared_ptr<std::unordered_set<Eigen::Vector4d>> resrv_tbl_;

  int num_agents_; // Number of agents

  /* Data structs */
  std::vector<std::unique_ptr<AStarPlanner>>front_end_planners_; // Vector of planners

  std::map<int, std::shared_ptr<DynamicVoronoi>> dyn_voro_arr_; // array of voronoi objects with key of height (cm)
  std::map<int, std::shared_ptr<std::vector<std::vector<bool>>>> bool_map_arr_; //  array of voronoi objects with key of height (cm)

  std::unordered_map<int, std::vector<Eigen::Vector4d>> space_time_path_; // Space time front end path in local map origin frame

  bool init_voro_maps_{false}; // flag to indicate if voronoi map is initialized

  bool plan_once_{true}; // for testing

  /* Debugging */
  Timer tm_front_end_plan_{"front_end_plan"};
  Timer tm_voro_map_init_{"voro_map_init"};
};



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


