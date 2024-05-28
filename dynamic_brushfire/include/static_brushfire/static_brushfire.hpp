// Code inspired by benchmark_utils.cpp from Bonxai

#ifndef _STATIC_BRUSHFIRE_HPP
#define _STATIC_BRUSHFIRE_HPP

// #include <Eigen/Eigen>

// We use SDL_image to load the image from disk
#include <SDL/SDL_image.h>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <limits>
#include <queue>

#include "dynamic_brushfire/dynamicvoronoi.h"

namespace cost_val
{
static const int8_t OCC = 100;
static const int8_t FREE = 0;
static const int8_t UNKNOWN = -1;
}

#define INF std::numeric_limits<double>::max()
#define SQRT2 1.41421

template<typename T, typename priority_t>
struct PriorityQueue {
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

class StaticBrushfire
{
public:
  // StaticBrushfire(){}

  void init(ros::NodeHandle &nh, ros::NodeHandle &pnh);

  void initParams(ros::NodeHandle &pnh);

  void loadMapFromFile(nav_msgs::OccupancyGrid& map,
                       const std::string& fname);

  void pgmFileToBoolMap(bool ***map,
                        int& size_x, int& size_y,
                        const std::string& fname);

  // Compute distance map
  void computeDistanceMap();

  // Update distance map of a cell
  void lowerStatic(const size_t& idx);

  // Get 8-connected neighbors of an cell by index
  std::vector<size_t> get8ConNeighbours(const size_t& idx){
    std::vector<size_t> neighbours;
    int x, y;
    map1Dto2DIdx(idx, occ_grid_.info.width, x, y);
    
    // Explore all 8 neighbours
    for (int dx = -1; dx <= 1; dx++)
    {
      for (int dy = -1; dy <= 1; dy++)
      {
        // Skip it's own position
        if (dx == 0 && dy == 0){
          continue;
        }
        const int idx = map2Dto1DIdx(occ_grid_.info.width, x+dx, y+dy);

        // Skip if current index is outside the map
        if (outsideMap(idx)) {
          continue;
        }

        // Skip if current index is occupied
        if (isOcc(idx)){
          continue;
        }

        neighbours.push_back(idx);
      }
    }

    return neighbours;
  } 

  // Get 4-connected neighbors of an cell by index
  std::vector<size_t> get4ConNeighbours(const size_t& idx);
  
  // Convert from map to occupancy grid type
  void mapToOccGrid(const std::vector<double>& map, const size_t& width, const size_t& height, nav_msgs::OccupancyGrid& occ_grid)
  {
    occ_grid.info.width = width;
    occ_grid.info.height = height;
    occ_grid.info.resolution = res_;
    occ_grid.info.origin.position.x = origin_x_;
    occ_grid.info.origin.position.y = origin_y_;
    occ_grid.info.origin.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_);
    occ_grid.info.origin.orientation.x = q.x();
    occ_grid.info.origin.orientation.y = q.y();
    occ_grid.info.origin.orientation.z = q.z();
    occ_grid.info.origin.orientation.w = q.w();

    occ_grid.data.resize(occ_grid.info.width * occ_grid.info.height);

    const double max_val = *std::max_element(map.begin(), map.end());
    // const double min_vel = *std::min_element(map.begin(), map.end());

    for (size_t idx = 0; idx < map.size(); idx++)
    {
      double val = 255.0 - (map[idx] / max_val) * 255.0;
      occ_grid.data[idx] = val;
    }
  }

  // Convert from map to occupancy grid type
  void voronoimapToOccGrid(const DynamicVoronoi& dyn_voro, const size_t& size_x, const size_t& size_y, nav_msgs::OccupancyGrid& occ_grid)
  {
    occ_grid.header.stamp = ros::Time::now();
    occ_grid.header.frame_id = "map";
    occ_grid.info.width = size_x;
    occ_grid.info.height = size_y;
    occ_grid.info.resolution = res_;
    occ_grid.info.origin.position.x = origin_x_;
    occ_grid.info.origin.position.y = origin_y_;
    occ_grid.info.origin.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_);
    occ_grid.info.origin.orientation.x = q.x();
    occ_grid.info.origin.orientation.y = q.y();
    occ_grid.info.origin.orientation.z = q.z();
    occ_grid.info.origin.orientation.w = q.w();

    occ_grid.data.resize(occ_grid.info.width * occ_grid.info.height);

    for(int j = 0; j < size_y; j++)
    {
      for (int i = 0; i < size_x; i++)
      {
        size_t idx = map2Dto1DIdx(occ_grid.info.width, i, occ_grid.info.height - j - 1);
        occ_grid.data[idx] = dyn_voro.isVoronoi(i, j) ? 255: 0;
      }
    }
  }

  // // Convert from map to occupancy grid type
  // void voronoimapToOccGrid(const std::vector<bool>& map, const size_t& width, const size_t& height, nav_msgs::OccupancyGrid& occ_grid)
  // {
  //   occ_grid.info.width = width;
  //   occ_grid.info.height = height;
  //   occ_grid.info.resolution = res_;
  //   occ_grid.info.origin.position.x = origin_x_;
  //   occ_grid.info.origin.position.y = origin_y_;
  //   occ_grid.info.origin.position.z = 0.0;
  //   tf2::Quaternion q;
  //   q.setRPY(0, 0, yaw_);
  //   occ_grid.info.origin.orientation.x = q.x();
  //   occ_grid.info.origin.orientation.y = q.y();
  //   occ_grid.info.origin.orientation.z = q.z();
  //   occ_grid.info.origin.orientation.w = q.w();

  //   occ_grid.data.resize(occ_grid.info.width * occ_grid.info.height);

  //   for (size_t idx = 0; idx < map.size(); idx++)
  //   {
  //     occ_grid.data[idx] = map[idx] ? 255: 0;
  //   }
  // }

  size_t map2Dto1DIdx(const int& width, const int& x, const int& y)
  {
    return width * y + x;
  }

  void map1Dto2DIdx(const int& idx, const int& width, int& x, int& y)
  {
    y = idx/width;
    x = idx - (y * width);
  }

  inline bool isOcc(const int& idx){
    return occ_grid_.data[idx] == cost_val::OCC;
  }

  inline bool outsideMap(const int& idx){
    return idx > (occ_grid_.data.size() - 1);
  }

  // Get euclidean distance between node_1 and node_2
  // NOTE: This is in units of indices
  inline double getL1Norm(const size_t& a, const size_t& b) {
    int a_x, a_y, b_x, b_y;
    map1Dto2DIdx(a, occ_grid_.info.width, a_x, a_y);
    map1Dto2DIdx(b, occ_grid_.info.width, b_x, b_y);
    return abs(a_x - b_x) + abs(a_y - b_y);
  }

  // Get euclidean distance between node_1 and node_2
  // NOTE: This is in units of indices
  inline double getL2Norm(const size_t& a, const size_t& b) {
    int a_x, a_y, b_x, b_y;
    map1Dto2DIdx(a, occ_grid_.info.width, a_x, a_y);
    map1Dto2DIdx(b, occ_grid_.info.width, b_x, b_y);

    double dx = abs(a_x - b_x);
    double dy = abs(a_y - b_y);

    return sqrt(dx*dx + dy*dy);
  }

  // // Get octile distance
  inline double getChebyshevDist(const size_t& a, const size_t& b)  {
    int a_x, a_y, b_x, b_y;
    map1Dto2DIdx(a, occ_grid_.info.width, a_x, a_y);
    map1Dto2DIdx(b, occ_grid_.info.width, b_x, b_y);

    double dx = abs(a_x - b_x);
    double dy = abs(a_y - b_y);

    return (dx + dy) - std::min(dx, dy); 
  }

  // // Get chebyshev distance
  inline double getOctileDist(const size_t& a, const size_t& b)  {
    int a_x, a_y, b_x, b_y;
    map1Dto2DIdx(a, occ_grid_.info.width, a_x, a_y);
    map1Dto2DIdx(b, occ_grid_.info.width, b_x, b_y);

    double dx = abs(a_x - b_x);
    double dy = abs(a_y - b_y);

    return (dx + dy) + (SQRT2 - 2) * std::min(dx, dy); 
  }

  void setObstacle(const size_t& s) {
    obst_map_[s] = s;
    dist_map_[s] = 0; 

    open_queue_.put(s, 0); // add to open queue
  }

  void removeObstacle(const size_t& s) {
    clearCell(s);  
    to_raise_[s] = true; 

    open_queue_.put(s, 0); // add to open queue
  }

  void clearCell(const size_t& s){
    dist_map_[s] = INF; 
    obst_map_[s] = -1; 
  }

  void updateDistanceMap() {
    while (!open_queue_.empty())
    {
      const size_t s = open_queue_.get();
      if (to_raise_[s]){
        raise(s);
      }
      else if (isOcc(obst_map_[s])){
        voro_map_[s] = false;
        lower(s);
      }
    }
  }

  void raise(const size_t& s) {
    for (const size_t& n: get8ConNeighbours(s))
    { 
      if (obst_map_[n] != -1 && to_raise_[n])
      {
        open_queue_.put(n, dist_map_[n]); // add to open queue

        if (!isOcc(obst_map_[n]))
        {
          clearCell(n);
          to_raise_[n] = true;
        }
      }
    }
    to_raise_[s] = false;
  }

  void lower(const size_t& s) {
    for (const size_t& n: get8ConNeighbours(s))
    { 
      if (!to_raise_[n])
      {
        double d = getL2Norm(obst_map_[s], n);
        if (d < dist_map_[n]) {
          dist_map_[n] = d;
          obst_map_[n] = obst_map_[s];
          open_queue_.put(n, d); // add to open queue
        }
        else {
          checkVoro(s, n);
        }
      }
    }
  }

  void checkVoro(const int& s, const int& n){
    std::vector<size_t> nb = get8ConNeighbours(obst_map_[n]);

    if ( (dist_map_[s] > 1 || dist_map_[n] > 1 ) 
          && obst_map_[n] != -1
          && obst_map_[n] != obst_map_[s]
          && std::find(nb.begin(), nb.end(), obst_map_[s]) == nb.end() )
    {
      if (getL2Norm(s, obst_map_[n]) < getL2Norm(n, obst_map_[s])){
        voro_map_[s] = true;
      }

      if (getL2Norm(n, obst_map_[s]) < getL2Norm(s, obst_map_[n])){
        voro_map_[n] = true;
      }
    }
  }

private:
  /* Params */
  std::string map_fname_;
  bool negate_{false};
  double res_;
  double occ_th_, free_th_;
  double origin_x_, origin_y_, yaw_;

  /* Pubs, subs */
  ros::Publisher occ_map_pub_;      // Publishes original occupancy grid
  ros::Publisher dist_occ_map_pub_; // Publishes distance map occupancy grid
  ros::Publisher voro_occ_grid_pub_; // Publishes voronoi map occupancy grid

  /* Data structs */
  bool **bool_map_{NULL};
  int size_x_, size_y_;

  nav_msgs::OccupancyGrid occ_grid_;
  std::vector<size_t> occ_idx_; // Indices of all occupied cells
  std::vector<size_t> free_idx_; // Indices of all free cells
  std::vector<size_t> unknown_idx_; // Indices of all unknown cells

  PriorityQueue<int, double> open_queue_; // Min priority queue of (cell idx, priority value)

  std::vector<double> dist_map_; // index representing map position and values representing distance to nearest obstacle
  std::vector<double> obst_map_; // obstacle reference map, stores the location of the closest obstacle of each visited cell. Value of -1 indicates cleared

  std::vector<bool> to_raise_;

  std::vector<bool> voro_map_; // Voronoi map

  nav_msgs::OccupancyGrid dist_occ_grid_; // Visualization of distance map in occupancy grid form
  nav_msgs::OccupancyGrid voro_occ_grid_; // Visualization of voronoi map in occupancy grid form
};

#endif // _STATIC_BRUSHFIRE_HPP