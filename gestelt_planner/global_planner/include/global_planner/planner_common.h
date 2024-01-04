#ifndef _PLANNER_COMMON_H_
#define _PLANNER_COMMON_H_

// Common helper methods for planners
#include <grid_map/grid_map.h>
#include <limits>
#include <Eigen/Eigen>

using namespace Eigen;
constexpr double inf = std::numeric_limits<float>::infinity();
constexpr double epsilon = std::numeric_limits<double>::epsilon();

struct GridNode; //forward declration
typedef std::shared_ptr<GridNode> GridNodePtr;

enum CellState
{
  OPEN = 1,
  CLOSED = 2,
  UNDEFINED = 3
};

struct GridNode
{
  GridNode(const Eigen::Vector3i& idx)
  {
    this->idx = idx;
  }

	Eigen::Vector3i idx;
	double g_cost{inf}, f_cost{inf};
	std::shared_ptr<GridNode> parent{nullptr};
  CellState state{CellState::UNDEFINED};

  // Equality
  bool operator==(const GridNode& node) const
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
    size_t operator()(GridNode& node) const
    {
      return (node.idx(0) * 7927 + node.idx(1)) * 7993 + node.idx(2);
    }
  };

  // Equality between pointers
  struct PointedObjEq {
    bool operator () ( GridNodePtr l_node, GridNodePtr r_node) const {
      return *l_node == *r_node;
    }
  };

  // Hash generation for pointers
  struct PointedObjHash
  {
    size_t operator()(const GridNodePtr& node) const
    {
      // https://stackoverflow.com/questions/1358468/how-to-create-unique-integer-number-from-3-different-integers-numbers1-oracle-l
      // https://stackoverflow.com/questions/38965931/hash-function-for-3-integers
      return (node->idx(0) * 7927 + node->idx(1)) * 7993 + node->idx(2);
    }
  };

  // Comparison operator between pointers
  struct CompareCostPtr
  {
    bool operator()(const GridNodePtr& l_node, const GridNodePtr& r_node)
    {
      return l_node->f_cost > r_node->f_cost;
    }
  };

};

class PlannerCommon {
/**
 * PlannerCommon acts a wrapper to the underlying obstacle map and provides commonly
 * used methods for search-based planners
 * */ 
public:
  PlannerCommon(std::shared_ptr<GridMap> grid_map)
  : map_(grid_map)
  {}

  std::vector<GridNodePtr> getNeighbors(GridNodePtr cur_node) {
    std::vector<GridNodePtr> neighbors;

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

          Eigen::Vector3i nb_idx{
            cur_node->idx(0) + dx,
            cur_node->idx(1) + dy,
            cur_node->idx(2) + dz,
          };
          
          if (getOccupancy(nb_idx)){
            // Skip if current index is occupied
            continue;
          }

          GridNodePtr nb_node = std::make_shared<GridNode>(nb_idx);
          // ROS_INFO("getNeighbors: Pushed back (%d, %d, %d)", nb_idx(0), nb_idx(1), nb_idx(2));

          neighbors.push_back(nb_node);
        }
      }
    }

    return neighbors;
  } 

  // Get euclidean distance between node_1 and node_2
  // NOTE: This is in units of indices
  double getL2Norm(GridNodePtr node_1, GridNodePtr node_2) {
    return (node_2->idx - node_1->idx).norm();
  }

  // Get euclidean distance between node_1 and node_2
  // NOTE: This is in units of indices
  double getL1Norm(GridNodePtr node_1, GridNodePtr node_2) {
    return (node_1->idx - node_2->idx).lpNorm<1>();
  }

  // ??? 
  double getDiagCost(GridNodePtr node_1, GridNodePtr node_2) {
    double dx = abs(node_1->idx(0) - node_2->idx(0));
    double dy = abs(node_1->idx(1) - node_2->idx(1));
    double dz = abs(node_1->idx(2) - node_2->idx(2));

    double h = 0.0;
    int diag = std::min(std::min(dx, dy), dz);
    dx -= diag;
    dy -= diag;
    dz -= diag;

    if (dx == 0)
    {
      h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * std::min(dy, dz) + 1.0 * abs(dy - dz);
    }
    if (dy == 0)
    {
      h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * std::min(dx, dz) + 1.0 * abs(dx - dz);
    }
    if (dz == 0)
    {
      h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * std::min(dx, dy) + 1.0 * abs(dx - dy);
    }
    return h;
  }

  // Get index of grid node in string 
  std::string getIndexStr(GridNodePtr node){
    return "(" + std::to_string(node->idx(0)) + ", " + std::to_string(node->idx(1)) + ", " +  std::to_string(node->idx(2)) + ")";
  }

  // Get position of grid node in string 
  std::string getPosStr(GridNodePtr node){
    Eigen::Vector3d pos;
    idxToPos(node->idx, pos);
    return "(" + std::to_string(pos(0)) + ", " + std::to_string(pos(1)) + ", " +  std::to_string(pos(2)) + ")";
  }

  // Convert from 3d position to gridmap index
  void posToIdx(const Eigen::Vector3d& pos, Eigen::Vector3i& idx) {
    idx = ((pos - map_->getOrigin()) / map_->getResolution()).array().floor().cast<int>();
  }

  // Convert from gridmap index to 3d position
  void idxToPos(const Eigen::Vector3i& idx, Eigen::Vector3d& pos){
    pos = idx.cast<double>() * map_->getResolution() + map_->getOrigin();
  }

  bool isInGlobalMap(const Eigen::Vector3i& idx){
    Eigen::Vector3d pos;
    idxToPos(idx, pos);
    return map_->isInGlobalMap(pos);
  }

  bool isInGlobalMap(const Eigen::Vector3d& pos){
    return map_->isInGlobalMap(pos);
  }

  int getOccupancy(const Eigen::Vector3i& idx){
    Eigen::Vector3d pos;
    idxToPos(idx, pos);
    return (map_->getInflateOccupancy(pos) != 0);
  }

  int getOccupancy(const Eigen::Vector3d& pos){
    return (map_->getInflateOccupancy(pos) != 0);
  }

private:
  std::shared_ptr<GridMap> map_; 
};

#endif // _PLANNER_COMMON_H_
