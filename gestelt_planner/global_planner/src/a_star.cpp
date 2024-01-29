#include <global_planner/a_star.h>

AStarPlanner::AStarPlanner(std::shared_ptr<GridMap> grid_map, const AStarParams& astar_params):
    astar_params_(astar_params)
{
    common_.reset(new PlannerCommon(grid_map));
}

void AStarPlanner::reset()
{
    closed_list_.clear();
    open_list_ = std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, GridNode::CompareCostPtr>();
}

bool AStarPlanner::generatePlan(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &goal_pos)
{
    reset();

    // Search takes place in index space. So we first convert 3d real world positions into indices
    if (!common_->isInGlobalMap(start_pos) || !common_->isInGlobalMap(goal_pos))
    {
        ROS_ERROR("Start or goal position not in global map!");
        return false;
    }

    if (common_->getOccupancy(start_pos)){
        ROS_ERROR("Start position in obstacle!");
        return false;
    }
    if (common_->getOccupancy(goal_pos)){
        ROS_ERROR("Goal position in obstacle!");
        return false;
    }

    Vector3i start_idx, goal_idx;
    common_->posToIdx(start_pos, start_idx);
    common_->posToIdx(goal_pos, goal_idx);

    GridNodePtr start_node = std::make_shared<GridNode>(start_idx);
    GridNodePtr goal_node = std::make_shared<GridNode>(goal_idx);

    start_node->g_cost = 0.0;
    start_node->f_cost = astar_params_.tie_breaker * common_->getL2Norm(start_node, goal_node);
    start_node->parent = nullptr;

    addToOpenlist(start_node);

    int num_iter = 0;

    while (!open_list_.empty())
    {
        if (num_iter%1000 == 0){
            ROS_INFO("[a_star] Iteration %d", num_iter);

            publishVizPoints(getClosedList(), closed_list_viz_pub_);
        }
        GridNodePtr cur_node = popOpenlist();
        addToClosedlist(cur_node); // Mark as visited

        if (*cur_node == *goal_node)
        {
            ROS_INFO("[a_star] Goal found!");
            // Goal reached, terminate search and obtain path
            tracePath(cur_node);
            return true;
        }

        // Explore neighbors of current node
        for (GridNodePtr nb_node : common_->getNeighbors(cur_node))
        {
            // ROS_INFO("Exploring nb (%s) [%s]", common_->getPosStr(nb_node).c_str(), common_->getIndexStr(nb_node).c_str());
            double tent_g_cost = cur_node->g_cost + common_->getL1Norm(cur_node, nb_node);

            // If tentative cost is better than previously computed cost, then update the g and f cost
            if (tent_g_cost < nb_node->g_cost)
            {
                nb_node->g_cost = tent_g_cost;
                // The tie_breaker is used to assign a larger weight to the h_cost and favour 
                // expanding nodes closer towards the goal
                nb_node->f_cost = tent_g_cost + astar_params_.tie_breaker * common_->getL1Norm(nb_node, goal_node);
            }

            // If not already in closed list: set parent and add to open list
            if (!isInClosedList(nb_node)) 
            {
                nb_node->parent = cur_node;
                addToOpenlist(nb_node);
            }
            // No need to update parents for nodes already in closed list, paths leading up to current node is alr the most optimal
        }
        num_iter++;
    }
    // ROS_INFO("[a_star] Iterations required: %d", num_iter);

    return false;
}

/**
 * @brief Get successful plan in terms of path positions
 *
 * @return std::vector<Eigen::Vector3d>
 */
std::vector<Eigen::Vector3d> AStarPlanner::getPathPos()
{
    return path_pos_;
}

/**
 * @brief Get successful plan in terms of path positions
 *
 * @return std::vector<Eigen::Vector3d>
 */
std::vector<Eigen::Vector3d> AStarPlanner::getClosedList()
{
    std::vector<Eigen::Vector3d> closed_list_pos;
    for (auto itr = closed_list_.begin(); itr != closed_list_.end(); ++itr) {
      Eigen::Vector3d node_pos;
      common_->idxToPos((*itr)->idx, node_pos);
      closed_list_pos.push_back(node_pos);
    }

    return closed_list_pos;
}

void AStarPlanner::addToOpenlist(GridNodePtr node)
{
    node->state = CellState::OPEN;
    open_list_.push(node);
}

GridNodePtr AStarPlanner::popOpenlist()
{
    GridNodePtr node = open_list_.top();
    open_list_.pop();
    return node;
}

void AStarPlanner::addToClosedlist(GridNodePtr node)
{
    node->state = CellState::CLOSED; // move current node to closed set.
    closed_list_.insert(node);
}

bool AStarPlanner::isInClosedList(GridNodePtr node)
{
    if (closed_list_.find(node) != closed_list_.end())
    {
        return true;
    }
    return false;
}

void AStarPlanner::tracePath(GridNodePtr node)
{
    // Clear existing data structures
    path_gridnode_.clear();
    path_idx_.clear();
    path_pos_.clear();

    // Trace back the nodes through the pointer to their parent
    GridNodePtr cur_node = node;
    while (cur_node->parent != nullptr)
    {
        path_gridnode_.push_back(cur_node);
        cur_node = cur_node->parent;
    }
    // Push back the start node
    path_gridnode_.push_back(cur_node);

    // Reverse the order of the path so that it goes from start to goal
    std::reverse(path_gridnode_.begin(), path_gridnode_.end());

    // For each gridnode, get the position and index,
    // So we can obtain a path in terms of indices and positions
    for (auto gridnode_ptr : path_gridnode_)
    {
        Eigen::Vector3d gridnode_pos;
        common_->idxToPos(gridnode_ptr->idx, gridnode_pos);

        path_pos_.push_back(gridnode_pos);
        path_idx_.push_back(gridnode_ptr->idx);
    }
}
