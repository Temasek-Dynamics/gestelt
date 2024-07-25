#ifndef _PLANNER_COMMON_H_
#define _PLANNER_COMMON_H_

// Common helper methods for planners
#include <grid_map/grid_map.h>
#include <limits>
#include <Eigen/Eigen>
#include <queue>
#include <global_planner/point.h>

using namespace Eigen;
constexpr double infinity = std::numeric_limits<float>::infinity();
constexpr double epsilon = std::numeric_limits<double>::epsilon();

#define SQRT2 1.4142135623

enum CellState
{
  OPEN = 1,
  CLOSED = 2,
  UNDEFINED = 3
};

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

  int x, y, z;
  double z_m; // [THIS IS NOT THE INDEX] z in meters
  int z_cm; // [THIS IS NOT THE INDEX] z in centimeters
}; // struct VCell


template <> 
struct std::hash<VCell> {
  /* implement hash function so we can put VCell into an unordered_set */
  std::size_t operator()(const VCell& pos) const noexcept {
    // NOTE: better to use something like boost hash_combine
    size_t H_x_y = 0.5 * (pos.x + pos.y)*(pos.x + pos.y + 1) + pos.y;
    return 0.5 * (H_x_y + pos.z)*(H_x_y + pos.z + 1) + pos.z;
  }
};


class PlannerCommon {
/**
 * PlannerCommon acts a wrapper to the underlying obstacle map and provides commonly
 * used methods for search-based planners
 * */ 
public:
  PlannerCommon(std::shared_ptr<GridMap> grid_map)
  : map_(grid_map)
  {

    nb_idx_8con_ <<
      0, 0,    1,  // Top 
      1, 1,    1,  // Top Fwd Left
      1, 0,    1,  // Top Fwd 
      1, -1,   1,  // Top Fwd Right
      0, 1,    1,  // Top Left
      0, -1,   1,  // Top Right
      -1, 1,   1,  // Top Bck Left
      -1, 0,   1,  // Top Bck 
      -1, -1,  1,  // Top Bck Right 

      // Mid Layer
      // {0, 0,    0}, // Mid 
      1, 1,    0, // Mid Fwd Left
      1, 0,    0, // Mid Fwd
      1, -1,   0, // Mid Fwd Right
      0, 1,    0, // Mid Left
      0, -1,   0, // Mid Right
      -1, 1,   0, // Mid Bck Left
      -1, 0,   0, // Mid Bck
      -1, -1,  0, // Mid Bck Right 

      // Btm Layer
      0, 0,    -1, // Btm 
      1, 1,    -1, // Btm Fwd Left
      1, 0,    -1, // Btm Fwd
      1, -1,   -1, // Btm Fwd Right
      0, 1,    -1, // Btm Left
      0, -1,   -1, // Btm Right
      -1, 1,   -1, // Btm Bck Left
      -1, 0,   -1, // Btm Bck
      -1, -1,  -1;// Btm Bck Right 

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

  Eigen::Matrix<int, 26, 3> nb_idx_8con_; 

  const double nb_8con_dist_l2_[26] = {
    // Top Layer
    1.0,  // Top 
    1.73205,    // Top Fwd Left
    1.414214,   // Top Fwd 
    1.73205,    // Top Fwd Right
    1.414214,   // Top Left
    1.414214,   // Top Right
    1.73205,    // Top Bck Left
    1.414214,   // Top Bck 
    1.73205,    // Top Bck Right 

    // Mid Layer
    // 0, // Mid 
    1.414214, // Mid Fwd Left
    1.0, // Mid Fwd
    1.414214, // Mid Fwd Right
    1.0, // Mid Left
    1.0, // Mid Right
    1.414214, // Mid Bck Left
    1.0, // Mid Bck
    1.414214, // Mid Bck Right 

    // Btm Layer
    1.0,  // Top 
    1.73205, // Btm Fwd Left
    1.414214, // Btm Fwd
    1.73205, // Btm Fwd Right
    1.414214, // Btm Left
    1.414214, // Btm Right
    1.73205, // Btm Bck Left
    1.414214, // Btm Bck
    1.73205 // Btm Bck Right 
  };

  const int nb_8con_dist_l1_[26] = {
    // Top Layer
    1,  // Top 
    3,    // Top Fwd Left
    2,   // Top Fwd 
    3,    // Top Fwd Right
    2,   // Top Left
    2,   // Top Right
    3,    // Top Bck Left
    2,   // Top Bck 
    3,    // Top Bck Right 

    // Mid Layer
    // 0, // Mid 
    2, // Mid Fwd Left
    1, // Mid Fwd
    2, // Mid Fwd Right
    1, // Mid Left
    1, // Mid Right
    2, // Mid Bck Left
    1, // Mid Bck
    2, // Mid Bck Right 

    // Btm Layer
    1,  // Top 
    3, // Btm Fwd Left
    2, // Btm Fwd
    3, // Btm Fwd Right
    2, // Btm Left
    2, // Btm Right
    3, // Btm Bck Left
    2, // Btm Bck
    3 // Btm Bck Right 
  };

  std::shared_ptr<GridMap> map_; 
};


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




// // Get euclidean distance between node_1 and node_2
// // NOTE: This is in units of indices
// inline double getL1Norm2D(const INTPOINT& a, const INTPOINT& b) {
//   return abs(a.x - b.x) + abs(a.y - b.y);
// }

// // Get euclidean distance between node_1 and node_2
// // NOTE: This is in units of indices
// inline double getL2Norm2D(const INTPOINT& a, const INTPOINT& b) {
//   double dx = abs(a.x - b.x);
//   double dy = abs(a.y - b.y);

//   return sqrt(dx*dx + dy*dy);
// }

// // // Get octile distance
// inline double getChebyshevDist2D(const INTPOINT& a, const INTPOINT& b)  {
//   double dx = abs(a.x - b.x);
//   double dy = abs(a.y - b.y);

//   return (dx + dy) - std::min(dx, dy); 
// }

// // // Get chebyshev distance
// inline double getOctileDist2D(const INTPOINT& a, const INTPOINT& b)  {
//   double dx = abs(a.x - b.x);
//   double dy = abs(a.y - b.y);

//   return (dx + dy) + (SQRT2 - 2) * std::min(dx, dy); 
// }






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



#endif // _PLANNER_COMMON_H_
