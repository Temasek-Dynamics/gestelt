// Common helper methods for planners
#include <plan_env/grid_map.h>
#include <Eigen/Eigen>

using namespace Eigen;
double inf = numeric_limits<float>::infinity();

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
	// int rounds{0}; // Distinguish every call

  CellState state{CellState::UNDEFINED};

	Eigen::Vector3i idx;

	double g_cost{inf}, f_cost{inf};

	std::shared_ptr<GridNode> parent{nullptr};
};

class CompareCost
{
public:
	bool operator()(GridNodePtr node_1, GridNodePtr node_2)
	{
		return node_1->f_cost > node_2->f_cost;
	}
};

class PlannerCommon {
/**
 * PlannerCommon acts a wrapper to the underlying obstacle map and provides commonly
 * used methods for search-based planners
 * */ 
public:
  PlannerCommon(GridMap::Ptr occ_map) {
    occ_map_ = occ_map;
  }

  std::vector<GridNodePtr> getNeighbors(GridNodePtr cur_node) {
    std::vector<GridNodePtr> neighbors;

    for (int dx = -1; dx <= 1; dx++){
      for (int dy = -1; dy <= 1; dy++){
        for (int dz = -1; dz <= 1; dz++)
        {
          if (dx == 0 && dy == 0 && dz == 0)
            continue;

          Vector3i nb_idx;
          nb_idx(0) = (cur_node->idx)(0) + dx;
          nb_idx(1) = (cur_node->idx)(1) + dy;
          nb_idx(2) = (cur_node->idx)(2) + dz;
          
          if (!isInMap(nb_idx)){
            continue;
          }

          GridNodePtr nb_node = std::make_shared<GridNode>();

          nb_node->idx = nb_idx;

          neighbors.push_back(nb_node);
        }
      }
    }

    return neighbors;
  } 

  // Get euclidean distance between node_1 and node_2
  double get_euclidean_cost(GridNodePtr node_1, GridNodePtr node_2) {
    return (node_2->idx - node_1->idx).norm();
  }

  // Get euclidean distance between node_1 and node_2
  double get_manhattan_cost(GridNodePtr node_1, GridNodePtr node_2) {
    double dx = abs(node_1->idx(0) - node_2->idx(0));
    double dy = abs(node_1->idx(1) - node_2->idx(1));
    double dz = abs(node_1->idx(2) - node_2->idx(2));

    return dx + dy + dz;
  }

  // Get euclidean distance between node_1 and node_2
  double get_diag_cost(GridNodePtr node_1, GridNodePtr node_2) {
    double dx = abs(node_1->idx(0) - node_2->idx(0));
    double dy = abs(node_1->idx(1) - node_2->idx(1));
    double dz = abs(node_1->idx(2) - node_2->idx(2));

    double h = 0.0;
    int diag = min(min(dx, dy), dz);
    dx -= diag;
    dy -= diag;
    dz -= diag;

      if (dx == 0)
      {
          h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dy, dz) + 1.0 * abs(dy - dz);
      }
      if (dy == 0)
      {
          h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dz) + 1.0 * abs(dx - dz);
      }
      if (dz == 0)
      {
          h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + 1.0 * abs(dx - dy);
      }
      return h;
  }

  // Convert from 3d position to gridmap index
  bool PosToIdx(const Vector3d& pos, Vector3i& idx) {
    if (!isInMap(pos)){
      return false;
    }
    occ_map_->posToIndex(pos, idx);
    return true;
  }

  // Check if position is occupied
  bool isOccupied(const Vector3d& pos){
    return occ_map_->getInflateOccupancy(pos) == 1 ? true : false;
  }

  bool isInMap(const Vector3i& idx){
    return occ_map_->isInMap(idx) ;
  }

  bool isInMap(const Vector3d& pos){
    return occ_map_->isInMap(pos) ;
  }

private:
  GridMap::Ptr occ_map_;
};

