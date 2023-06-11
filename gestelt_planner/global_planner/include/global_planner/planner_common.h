// Common helper methods for planners

#include <plan_env/grid_map.h>

std::vector<GridNodePtr> get_neighbors(gridmap) {

} 

// Get euclidean distance between node_1 and node_2
double get_euclidean_cost(GridNodePtr node_1, GridNodePtr node_2) {
  return (node2->index - node1->index).norm();
}

// Get euclidean distance between node_1 and node_2
double get_manhattan_cost(GridNodePtr node_1, GridNodePtr node_2) {
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));

    return dx + dy + dz;
}

