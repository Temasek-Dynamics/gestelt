// Common helper methods for planners
#include <plan_env/grid_map.h>
#include <Eigen/Eigen>

using namespace Eigen;

std::vector<GridNode *> get_neighbors(GridMap::Ptr occ_map) {
  std::vector<GridNode* > neighbors;
  return neighbors;
} 

// Get euclidean distance between node_1 and node_2
double get_euclidean_cost(GridNode * node_1, GridNode * node_2) {
  return (node_2->index - node_1->index).norm();
}

// Get euclidean distance between node_1 and node_2
double get_manhattan_cost(GridNode * node_1, GridNode * node_2) {
  double dx = abs(node_1->index(0) - node_2->index(0));
  double dy = abs(node_1->index(1) - node_2->index(1));
  double dz = abs(node_1->index(2) - node_2->index(2));

  return dx + dy + dz;
}

// Get euclidean distance between node_1 and node_2
double get_diag_cost(GridNode * node_1, GridNode * node_2) {
  double dx = abs(node_1->index(0) - node_2->index(0));
  double dy = abs(node_1->index(1) - node_2->index(1));
  double dz = abs(node_1->index(2) - node_2->index(2));

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
double pos_to_idx(Vector3d pos, Vector3i &idx) {

  // idx = 
  // if (!isInMap(pos, idx)){
  //   return false;
  // }
  
  int occ;
  // Check if it is occupied

}