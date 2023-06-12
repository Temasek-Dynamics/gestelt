#include <global_planner/planner_common.h>
#include <Eigen/Eigen>
#include <limits>

using namespace Eigen;

class PlannerBase
{
public:
  PlannerBase(){
    
  }

  // Adds pointer reference to gridmap
  void addGridMap(GridMap::Ptr occ_map){
    occ_map_ = occ_map;

    common_.reset(new PlannerCommon(occ_map));
  }

  bool generate_plan(Vector3d start_pos, Vector3d end_pos){
    // TODO: Convert start and end pos (in world frame) to the UAV's frame
    Vector3i start_idx, end_idx;

    if (!common_->PosToIdx(start_pos, start_idx) || common_->PosToIdx(end_pos, end_idx)){
      return false;
    }
    openlist_ = std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, CompareCost>();

    GridNodePtr start_node = std::make_shared<GridNode>();
    GridNodePtr end_node = std::make_shared<GridNode>();

    end_node->idx = end_idx;

    start_node->idx = start_idx;
    start_node->g_cost = 0.0;
    start_node->f_cost = common_->get_diag_cost(start_node, end_node);
    start_node->parent = nullptr;
    
    addToOpenlist(start_node);

    int num_iter = 0;
    while (!openlist_.empty()) 
    {
      GridNodePtr cur_node = popOpenlist();

      if (cur_node->idx(0) == end_node->idx(0) 
        && cur_node->idx(1) == end_node->idx(1) 
        && cur_node->idx(2) == end_node->idx(2))
      {
        // Goal reached
        // TODO get path
        return true;
      }
      // Explore neighbors
      std::vector<GridNodePtr> nb_nodes = common_->getNeighbors(cur_node);
      

      num_iter++;
    }

    // Create open list
    return true;
  }

  void addToOpenlist(GridNodePtr node){
    node->state = CellState::OPEN;
    openlist_.push(node);
  }

  GridNodePtr popOpenlist(){
    GridNodePtr node = openlist_.top();
    openlist_.pop();
    node->state = CellState::CLOSED; //move current node to closed set.

    return node;
  }


  // std::vector<GridNode *> getPath() {
  // }

private: 
  GridMap::Ptr occ_map_;
  std::unique_ptr<PlannerCommon> common_;

  std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, CompareCost> openlist_;

};