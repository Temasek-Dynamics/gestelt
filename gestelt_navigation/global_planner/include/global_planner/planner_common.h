#ifndef _PLANNER_COMMON_H_
#define _PLANNER_COMMON_H_

// Common helper methods for planners
#include <grid_map/grid_map.h>
#include <limits>
#include <Eigen/Eigen>
#include <queue>
#include <global_planner/point.h>
#include <boost/functional/hash_fwd.hpp>

using namespace Eigen;
constexpr double infinity = std::numeric_limits<float>::infinity();
constexpr double epsilon = std::numeric_limits<double>::epsilon();

#define SQRT2 1.4142135623

/**
 * @brief Priority queue used for open list in A* search
 * 
 * @tparam T 
 * @tparam priority_t 
 */
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

/**
 * PosIdx: Used for index-based 3d grid operations
 */
struct PosIdx {
  PosIdx() {}

  PosIdx(const int& x, const int& y, const int& z)
    : x(x), y(y), z(z)
  {}

  void setIdx(const Eigen::Vector3i& idx)
  {
    x = idx(0);
    y = idx(1);
    z = idx(2);
  }

  Eigen::Vector3i getIdx() const
  {
    return Eigen::Vector3i{x, y, z};
  }

  // Equality
  bool operator == (const PosIdx& pos) const
  {
    return (this->x == pos.x && this->y == pos.y && this->z == pos.z);
  }

  int x, y, z;
}; // struct PosIdx

template <> 
struct std::hash<PosIdx> {
  /* implement hash function so we can put PosIdx into an unordered_set */
  std::size_t operator()(const PosIdx& pos) const noexcept {
    // NOTE: better to use something like boost hash_combine
    size_t H_x_y = 0.5 * (pos.x + pos.y)*(pos.x + pos.y + 1) + pos.y;
    return 0.5 * (H_x_y + pos.z)*(H_x_y + pos.z + 1) + pos.z;
  }
};

/**
 * @brief Class used by 3D A* search
 * 
 */
class PlannerCommon {
/**
 * PlannerCommon acts a wrapper to the underlying obstacle map and provides commonly
 * used methods for search-based planners
 * */ 
public:
  PlannerCommon(std::shared_ptr<GridMap> grid_map)
  : map_(grid_map)
  {
  }

  void getNeighbours(const PosIdx& cur_node, std::vector<PosIdx>& neighbours) {

    neighbours.clear();
    
    // Explore all 26 neighbours
    for (int dx = -1; dx <= 1; dx++)
    {
      for (int dy = -1; dy <= 1; dy++)
      {
        for (int dz = -1; dz <= 1; dz++)
        {
          // Skip it's own position
          if (dx == 0 && dy == 0 && dz == 0){
            continue;
          }

          PosIdx nb_idx(cur_node.x + dx, cur_node.y + dy, cur_node.z + dz);
          
          if (getOccupancy(nb_idx)){
            // Skip if current index is occupied
            continue;
          }

          neighbours.push_back(nb_idx);
        }
      }
    }
  } 

  // Get index of grid node in string 
  std::string getIndexStr(PosIdx idx){
    return "("  + std::to_string(idx.x) + ", " 
                + std::to_string(idx.y) + ", " 
                + std::to_string(idx.z) + ")";
  }

  // Get position of grid node in string 
  std::string getPosStr(PosIdx idx){
    Eigen::Vector3d pos;
    idxToPos(idx, pos);
    return "(" + std::to_string(pos(0)) + ", " + std::to_string(pos(1)) + ", " +  std::to_string(pos(2)) + ")";
  }

  // Convert from 3d position to gridmap index
  void posToIdx(const Eigen::Vector3d& pos, PosIdx& idx) {
    idx.setIdx(((pos - map_->getGlobalOrigin()) / map_->getRes()).array().ceil().cast<int>());
  }

  // Convert from gridmap index to 3d position
  void idxToPos(const PosIdx& idx, Eigen::Vector3d& pos){
    pos = (idx.getIdx()).cast<double>() * map_->getRes() + map_->getGlobalOrigin();
  }

  bool isInGlobalMap(const PosIdx& idx){
    Eigen::Vector3d pos;
    idxToPos(idx, pos);
    return map_->isInGlobalMap(pos);
  }

  bool isInGlobalMap(const Eigen::Vector3d& pos){
    return map_->isInGlobalMap(pos);
  }

  int getOccupancy(const PosIdx& idx){
    Eigen::Vector3d pos;
    idxToPos(idx, pos);
    return map_->getInflateOccupancy(pos);
  }

  int getOccupancy(const Eigen::Vector3d& pos){
    return map_->getInflateOccupancy(pos);
  }

public: 

  std::shared_ptr<GridMap> map_; 
};


/**
 * Cost operations for 3D A* search
 */

// Get euclidean distance between node_1 and node_2
// NOTE: This is in units of indices
inline double getL1Norm(const PosIdx& a, const PosIdx& b) {
  return abs(a.x - b.x) + abs(a.y - b.y) + abs(a.z - b.z);
}

// Get euclidean distance between node_1 and node_2
// NOTE: This is in units of indices
inline double getL2Norm(const PosIdx& a, const PosIdx& b) {
  double dx = abs(a.x - b.x);
  double dy = abs(a.y - b.y);
  double dz = abs(a.z - b.z);

  return sqrt(dx*dx + dy*dy + dz*dz);
}

// // Get octile distance
inline double getChebyshevDist(const PosIdx& a, const PosIdx& b)  {
  double dx = abs(a.x - b.x);
  double dy = abs(a.y - b.y);
  double dz = abs(a.z - b.z);

  return (dx + dy + dz) - std::min(dx, std::min(dy, dz)); 
}

// // Get chebyshev distance
inline double getOctileDist(const PosIdx& a, const PosIdx& b)  {
  double dx = abs(a.x - b.x);
  double dy = abs(a.y - b.y);
  double dz = abs(a.z - b.z);

  return (dx + dy + dz) + (SQRT2 - 2) * std::min(dx, std::min(dy, dz)); 
}


/**
 * VCell: A cell used by voronoi graph search 
 * It has 3 elements: (x, y, z_cm). Where z is the height but in unit of centimeters.
 */
struct VCell {
  VCell() {}

  VCell(const int& x, const int& y, const int& z_cm)
    : x(x), y(y), z_cm(z_cm)
  {
    z_m = ((double)z_cm)/100.0;
    z = (int)(z_m/0.05); // 0.05 is the resolution
  }

  // Equality
  bool operator == (const VCell& pos) const
  {
    return (this->x == pos.x && this->y == pos.y && this->z_cm == pos.z_cm);
  }

  int x, y, z;  // Index of cell
  double z_m; // [THIS IS NOT THE INDEX] z in meters
  int z_cm; // [THIS IS NOT THE INDEX] z in centimeters
}; // struct VCell

template <> 
struct std::hash<VCell> {
  /* implement hash function so we can put VCell into an unordered_set */
  std::size_t operator()(const VCell& pos) const noexcept {
    std::size_t seed = 0;
    boost::hash_combine(seed, pos.x);
    boost::hash_combine(seed, pos.y);
    boost::hash_combine(seed, pos.z);
    return seed;
  }
};


/**
 * Cost operations for voronoi A* search
 */

// Get euclidean distance between node_1 and node_2
// NOTE: This is in units of indices
inline double getL1NormV(const VCell& a, const VCell& b) {
  return abs(a.x - b.x) + abs(a.y - b.y) + abs(a.z - b.z);
}

// Get euclidean distance between node_1 and node_2
// NOTE: This is in units of indices
inline double getL2NormV(const VCell& a, const VCell& b) {
  double dx = abs(a.x - b.x);
  double dy = abs(a.y - b.y);
  double dz = abs(a.z - b.z);

  return sqrt(dx*dx + dy*dy + dz*dz);
}

// // Get octile distance
inline double getChebyshevDistV(const VCell& a, const VCell& b)  {
  double dx = abs(a.x - b.x);
  double dy = abs(a.y - b.y);
  double dz = abs(a.z - b.z);

  return (dx + dy + dz) - std::min(dx, std::min(dy, dz)); 
}

// // Get chebyshev distance
inline double getOctileDistV(const VCell& a, const VCell& b)  {
  double dx = abs(a.x - b.x);
  double dy = abs(a.y - b.y);
  double dz = abs(a.z - b.z);

  return (dx + dy + dz) + (SQRT2 - 2) * std::min(dx, std::min(dy, dz)); 
}

/**
 * VCell_T: A space-time voronoi graph cell
 * It has 4 elements: (x, y, z, t). Where x,y,z are the index and t is the time at which the cell is occupied
 */
struct VCell_T {
  VCell_T() {}

  /**
   * @brief Construct a new VCell_T object
   * 
   * @param x index x coordinate
   * @param y index y coordinate
   * @param z_cm Height of voronoi map in centimeters 
   * @param t Time at which cell is occupied
   */
  VCell_T(const int& x, const int& y, const int& z_cm, const double& t)
    : x(x), y(y), z_cm(z_cm), t(t)
  {
    z_m = ((double)z_cm)/100.0;
    z = (int)(z_m/0.05); // 0.05 is the resolution
  }

  // Check if position is the same
  bool isSamePositionAs(const VCell_T& cell){
    return (this->x == cell.x && this->y == cell.y && this->z_cm == cell.z_cm );
  } 

  // Equality
  bool operator == (const VCell_T& cell) const
  {
    return (this->x == cell.x && this->y == cell.y && this->z_cm == cell.z_cm && this->t == cell.t);
  }

  int x, y, z;  // Index of cell
  double z_m; // [THIS IS NOT THE INDEX] z in meters
  int z_cm; // [THIS IS NOT THE INDEX] z in centimeters

  double t;  // Time at which cell is occupied
}; // struct VCell_T

template <> 
struct std::hash<VCell_T> {
  /* implement hash function so we can put VCell_T into an unordered_set */
  std::size_t operator()(const VCell_T& pos) const noexcept {
    std::size_t seed = 0;
    boost::hash_combine(seed, pos.x);
    boost::hash_combine(seed, pos.y);
    boost::hash_combine(seed, pos.z);
    boost::hash_combine(seed, pos.t);
    return seed;
  }
};

/**
 * Cost operations for space-time Voronoi A* search
 */

// Get euclidean distance between node_1 and node_2
// NOTE: This is in units of indices
inline double getL1NormVT(const VCell_T& a, const VCell_T& b) {
  return abs(a.x - b.x) + abs(a.y - b.y) + abs(a.z - b.z);
}

// Get euclidean distance between node_1 and node_2
// NOTE: This is in units of indices
inline double getL2NormVT(const VCell_T& a, const VCell_T& b) {
  double dx = abs(a.x - b.x);
  double dy = abs(a.y - b.y);
  double dz = abs(a.z - b.z);

  return sqrt(dx*dx + dy*dy + dz*dz);
}

// // Get octile distance
inline double getChebyshevDistVT(const VCell_T& a, const VCell_T& b)  {
  double dx = abs(a.x - b.x);
  double dy = abs(a.y - b.y);
  double dz = abs(a.z - b.z);

  return (dx + dy + dz) - std::min(dx, std::min(dy, dz)); 
}

// // Get chebyshev distance
inline double getOctileDistVT(const VCell_T& a, const VCell_T& b)  {
  double dx = abs(a.x - b.x);
  double dy = abs(a.y - b.y);
  double dz = abs(a.z - b.z);

  return (dx + dy + dz) + (SQRT2 - 2) * std::min(dx, std::min(dy, dz)); 
}


template <> 
struct std::hash<Eigen::Vector4d> {
  /* implement hash function so we can put VCell_T into an unordered_set */
  std::size_t operator()(const Eigen::Vector4d& pos) const noexcept {
    std::size_t seed = 0;
    boost::hash_combine(seed, pos(0));
    boost::hash_combine(seed, pos(1));
    boost::hash_combine(seed, pos(2));
    boost::hash_combine(seed, pos(3));
    return seed;
  }
};

#endif // _PLANNER_COMMON_H_
