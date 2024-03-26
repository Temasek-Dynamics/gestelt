#ifndef _PLANNER_COMMON_H_
#define _PLANNER_COMMON_H_

// Common helper methods for planners
#include <grid_map/grid_map.h>
#include <limits>
#include <Eigen/Eigen>
#include <queue>

using namespace Eigen;
constexpr double infinity = std::numeric_limits<float>::infinity();
constexpr double epsilon = std::numeric_limits<double>::epsilon();

struct OccNode; //forward declration
typedef std::shared_ptr<OccNode> OccNodePtr;

#define SQRT2 1.4142135623

enum CellState
{
  OPEN = 1,
  CLOSED = 2,
  UNDEFINED = 3
};

struct OccNode
{
  OccNode(const size_t& x, const size_t& y, const size_t& z)
  {
    this->idx = Eigen::Vector3i{x, y, z};
  }

  OccNode(const Eigen::Vector3i& idx)
  {
    this->idx = idx;
  }

	Eigen::Vector3i idx;
	double g_cost{infinity}, f_cost{infinity};
	std::shared_ptr<OccNode> parent{nullptr};
  CellState state{CellState::UNDEFINED};

  // Equality
  bool operator==(const OccNode& node) const
  {
    if (this->idx(0) == node.idx(0) 
      && this->idx(1) == node.idx(1)
      && this->idx(2) == node.idx(2)){
      return true;
    } 
    return false;
  }

  // Hash generation
  struct ObjHash
  {
    size_t operator()(OccNode& node) const
    {
      // return (node.idx(0) * 7927 + node.idx(1)) * 7993 + node.idx(2);
      // Cantor pairing function
      size_t H_a_b = 0.5 * (node.idx(0) + node.idx(1))*(node.idx(0) + node.idx(1) + 1) + node.idx(1);
      return 0.5 * (H_a_b + node.idx(2))*(H_a_b + node.idx(2) + 1) + node.idx(2);
    }
  };

  // Equality between pointers
  struct PointedObjEq {
    bool operator () ( OccNodePtr l_node, OccNodePtr r_node) const {
      return *l_node == *r_node;
    }
  };

  // Hash generation for pointers
  struct PointedObjHash
  {
    size_t operator()(const OccNodePtr& node) const
    {
      // https://stackoverflow.com/questions/1358468/how-to-create-unique-integer-number-from-3-different-integers-numbers1-oracle-l
      // https://stackoverflow.com/questions/38965931/hash-function-for-3-integers
      // return (node->idx(0) * 7927 + node->idx(1)) * 7993 + node->idx(2);

      // Cantor pairing function
      size_t H_a_b = 0.5 * (node->idx(0) + node->idx(1))*(node->idx(0) + node->idx(1) + 1) + node->idx(1);
      return 0.5 * (H_a_b + node->idx(2))*(H_a_b + node->idx(2) + 1) + node->idx(2);
    }
  };

  // Comparison operator between pointers
  struct CompareCostPtr
  {
    bool operator()(const OccNodePtr& l_node, const OccNodePtr& r_node)
    {
      return l_node->f_cost > r_node->f_cost;
    }
  };

};

class OccMap {
public:
  OccMap(size_t sz_x, size_t sz_y, size_t sz_z)
    : sz_x_(sz_x), sz_y_(sz_y_), sz_z_(sz_z_)
  {
    data_.resize(sz_x * sz_y * sz_z);

    // for (size_t x = 0; x < sz_x; x++){
    //   for (size_t y = 0; y < sz_y; y++){
    //     for (size_t z = 0; z < sz_z; z++){

    //       data_.at((z * sz_y * sz_x) + (y * sz_x) + x) = std::make_shared<OccNode>(x, y, z);
    //       // data_.at(x * sz_y_ * sz_z_ + y * sz_z_ + z)->setIdx(x,y,z);
    //     }
    //   }
    // }

  }

  // Access node of occupancy map at (x,y,z)
  std::shared_ptr<OccNode>& operator()(const Eigen::Vector3i& idx) {
    return data_.at( (idx(2) * sz_y_ * sz_x_) + (idx(1) * sz_x_) + idx(0));
  }

  // Access node of occupancy map at (x,y,z)
  std::shared_ptr<OccNode>& operator()(size_t x, size_t y, size_t z) {
    return data_.at( (z * sz_y_ * sz_x_) + (y * sz_x_) + x);
  }

  // Access const node of occupancy map at (x,y,z)
  std::shared_ptr<OccNode> const& operator()(const Eigen::Vector3i& idx) const {
    return data_.at( (idx(2) * sz_y_ * sz_x_) + (idx(1) * sz_x_) + idx(0));
  }

  // Access const node of occupancy map at (x,y,z)
  std::shared_ptr<OccNode> const& operator()(size_t x, size_t y, size_t z) const {
    return data_.at( (z * sz_y_ * sz_x_) + (y * sz_x_) + x);
  }

  size_t getSize() const {
    return data_.size();
  }

private:
  size_t sz_x_, sz_y_, sz_z_; // Size of Occupancy Map
  std::vector<std::shared_ptr<OccNode>> data_; // Contiguous vector of occupancy nodes
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

  // /**
  //  * @brief Get the Neighbors Idx object
  //  * 
  //  * @param cur_node 
  //  * @param neighbors 
  //  * @param nb_8con_idxs Index of 8 con neighbor lookup
  //  */
  // void getNeighbors(OccNodePtr cur_node, std::vector<OccNodePtr>& neighbors, std::vector<int>& nb_8con_idxs){
  //   neighbors.clear();
  //   nb_8con_idxs.clear();

  //   for (int i = 0; i < nb_idx_8con_.rows(); i++){
  //     Eigen::Vector3i nb_3d_idx = cur_node->idx + nb_idx_8con_.row(i).transpose();
  //                                 // + Eigen::Vector3i{nb_idx_8con_.row(i)(0), nb_idx_8con_.row(i)(1), nb_idx_8con_.row(i)(2)};

  //     if (getOccupancy(nb_3d_idx)){
  //       // Skip if current index is occupied
  //       continue;
  //     }

  //     neighbors.push_back(std::make_shared<OccNode>(nb_3d_idx));
  //     nb_8con_idxs.push_back(i);
  //   }

  // }

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

// ??? 
// double getDiagCost(OccNodePtr node_1, OccNodePtr node_2) {
//   double dx = abs(node_1->idx(0) - node_2->idx(0));
//   double dy = abs(node_1->idx(1) - node_2->idx(1));
//   double dz = abs(node_1->idx(2) - node_2->idx(2));

//   double h = 0.0;
//   int diag = std::min(std::min(dx, dy), dz);
//   dx -= diag;
//   dy -= diag;
//   dz -= diag;

//   if (dx == 0)
//   {
//     h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * std::min(dy, dz) + 1.0 * abs(dy - dz);
//   }
//   if (dy == 0)
//   {
//     h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * std::min(dx, dz) + 1.0 * abs(dx - dz);
//   }
//   if (dz == 0)
//   {
//     h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * std::min(dx, dy) + 1.0 * abs(dx - dy);
//   }
//   return h;
// }

#endif // _PLANNER_COMMON_H_
