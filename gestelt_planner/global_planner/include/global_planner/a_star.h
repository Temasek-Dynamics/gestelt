#ifndef _A_STAR_PLANNER_H_
#define _A_STAR_PLANNER_H_

#include <global_planner/planner_base.h>

#include <unordered_set>
#include <queue>

class AStarPlanner : public PlannerBase
{
public:
  AStarPlanner(std::shared_ptr<GridMap> grid_map);

  /**
   * @brief Clear closed, open list and reset planning_successful flag for new plan generation
   * 
   */
  void reset();

  /**
   * @brief Generate a new plan. 
   * 
   * @param start_pos 
   * @param goal_pos 
   * @return true 
   * @return false 
   */
  bool generatePlan(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& goal_pos);

  /**
   * @brief Get successful plan in terms of path positions
   * 
   * @return std::vector<Eigen::Vector3d> 
   */
  std::vector<Eigen::Vector3d> getPathPos();

  /**
   * @brief Get successful plan in terms of path positions
   * 
   * @return std::vector<Eigen::Vector3d> 
   */
  std::vector<Eigen::Vector3d> getClosedList();

private:

  void addToOpenlist(GridNodePtr node);

  GridNodePtr popOpenlist();

  void addToClosedlist(GridNodePtr node);

  bool isInClosedList(GridNodePtr node);

  void tracePath(GridNodePtr node);

private: 
  std::vector<GridNodePtr> path_gridnode_; // Path in terms of Grid Nodes
  std::vector<Eigen::Vector3i> path_idx_; // Path in terms of indices
  std::vector<Eigen::Vector3d> path_pos_; // Path in terms of 3d position

  /* Params */
  const double tie_breaker_ = 1.0 + 1.0 / 10000;

  /* Path planner data structures */

  // class for commonly used helper functions 
  std::unique_ptr<PlannerCommon> common_; 

  // open list stores nodes yet to be visited
  std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, GridNode::CompareCostPtr> open_list_; 
  // closed list stores nodes that have been visited
  std::unordered_set<GridNodePtr, GridNode::PointedObjHash, GridNode::PointedObjEq> closed_list_;

};

#endif // _A_STAR_PLANNER_H_