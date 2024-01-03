#include <global_planner/a_star.h>

AStarPlanner::AStarPlanner(std::shared_ptr<GridMap> grid_map)
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

    Vector3i start_idx, goal_idx;

    if (!common_->posToIdx(start_pos, start_idx) || !common_->posToIdx(goal_pos, goal_idx))
    {
        return false;
    }

    GridNodePtr start_node = std::make_shared<GridNode>(start_idx);
    GridNodePtr goal_node = std::make_shared<GridNode>(goal_idx);

    start_node->g_cost = 0.0;
    start_node->f_cost = tie_breaker_ * common_->getL2Norm(start_node, goal_node);
    start_node->parent = nullptr;

    addToOpenlist(start_node);

    int num_iter = 0;

    while (!open_list_.empty())
    {
        GridNodePtr cur_node = popOpenlist();
        addToClosedlist(cur_node); // Mark as visited
        ROS_INFO("Expanding (%f, %f, %f)", cur_node->idx(0), cur_node->idx(1), cur_node->idx(2));

        if (*cur_node == *goal_node)
        {
            ROS_INFO("[global_planner] Goal found!");
            // Goal reached, terminate search and obtain path
            tracePath(cur_node);
            return true;
        }

        // Explore neighbors
        for (GridNodePtr nb_node : common_->getNeighbors(cur_node))
        {
            double tent_g_cost = cur_node->g_cost + common_->getL2Norm(cur_node, nb_node);

            if (isInClosedList(nb_node))
            {
                // Update explored cost
                if (tent_g_cost < nb_node->g_cost)
                {
                    nb_node->g_cost = tent_g_cost;
                    nb_node->f_cost = tent_g_cost + tie_breaker_ * common_->getL2Norm(nb_node, goal_node);
                }
                continue;
            }

            if (tent_g_cost < nb_node->g_cost)
            {
                nb_node->g_cost = tent_g_cost;
                nb_node->f_cost = tent_g_cost + tie_breaker_ * common_->getL2Norm(nb_node, goal_node);

                nb_node->parent = cur_node;
                addToOpenlist(nb_node);
            }
        }
        num_iter++;
    }

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
