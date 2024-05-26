#include <global_planner/a_star.h>

#define L2_cost

AStarPlanner::AStarPlanner(std::shared_ptr<GridMap> grid_map, const AStarParams& astar_params):
    astar_params_(astar_params)
{
    common_.reset(new PlannerCommon(grid_map));
}

void AStarPlanner::reset()
{
    // Clear planning data
    closed_list_.clear();
    open_list_.clear();
    g_cost_.clear();
    came_from_.clear();

    // Clear visualizations
    clearVisualizations();
}

void AStarPlanner::addPublishers(
    std::unordered_map<std::string, ros::Publisher> &publisher_map)
{
    closed_list_viz_pub_ = publisher_map["front_end/closed_list"];
}


void AStarPlanner::clearVisualizations()
{
    visualization_msgs::Marker marker;

    marker.id = 0; 
    marker.ns = "closed_list"; 
    marker.action = visualization_msgs::Marker::DELETEALL;

    closed_list_viz_pub_.publish(marker);
}

bool AStarPlanner::generatePlan(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &goal_pos)
{

    std::function<double(const PosIdx&, const PosIdx&)> cost_function;

    switch ( astar_params_.cost_function_type ) {
        case 0:
            // std::cout << "[AStar]: Using octile distance cost " << std::endl; 
            cost_function = getOctileDist;
            break;
        case 1:
            // std::cout << "[AStar]: Using L1 Norm" << std::endl; 
            cost_function = getL1Norm; 
            break;
        case 2:
            // std::cout << "[AStar]: Using L2 Norm " << std::endl; 
            cost_function = getL2Norm;
            break;
        case 3:
            // std::cout << "[AStar]: Using Chebyshev Distance" << std::endl;
            cost_function = getChebyshevDist;
            break;
        default: 
            // std::cout << "[AStar]: Using Octile Distance" << std::endl;
            cost_function = getOctileDist;
            break;
    }
    
    return generatePlan(start_pos, goal_pos, cost_function);
}

bool AStarPlanner::generatePlan(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &goal_pos, 
                                std::function<double(const PosIdx&, const PosIdx&)> cost_function)
{
    auto total_loop_start = std::chrono::high_resolution_clock::now();
    auto preloop_start = std::chrono::high_resolution_clock::now();

    reset();

    // Search takes place in index space. So we first convert 3d real world positions into indices
    if (!common_->isInGlobalMap(start_pos) || !common_->isInGlobalMap(goal_pos))
    {
        std::cerr << "[a_star] Start or goal position not in global map!" << std::endl;
        return false;
    }

    if (common_->getOccupancy(start_pos)){
        std::cerr << "[a_star] Start position in obstacle!" << std::endl;
        return false;
    }
    if (common_->getOccupancy(goal_pos)){
        std::cerr << "[a_star] Goal position in obstacle!" << std::endl;
        return false;
    }

    PosIdx start_node, goal_node;
    common_->posToIdx(start_pos, start_node);
    common_->posToIdx(goal_pos, goal_node);

    came_from_[start_node] = start_node;
    g_cost_[start_node] = 0;

    open_list_.put(start_node, 0);

    int num_iter = 0;

    auto preloop_end = std::chrono::high_resolution_clock::now();
    auto preloop_dur = std::chrono::duration_cast<std::chrono::duration<double>>(
        preloop_end - preloop_start).count();
    double explore_nb_durs = 0;
    double loop_durs = 0;
    double chkpt_1_durs = 0;
    double get_nb_durs = 0;
    double get_occ_durs = 0;

    std::vector<PosIdx> neighbours; // 3d indices of neighbors

    while (!open_list_.empty() && num_iter < astar_params_.max_iterations)
    {
        auto loop_start = std::chrono::high_resolution_clock::now();

        if (num_iter%1000 == 1){
            // std::cout << "[a_star] Iteration " << num_iter << std::endl;

            publishVizPoints(getClosedList(), closed_list_viz_pub_);
        }

        auto chkpt_1_start = std::chrono::high_resolution_clock::now();

        PosIdx cur_node = open_list_.get();
        closed_list_.insert(cur_node);

        auto chkpt_1_end = std::chrono::high_resolution_clock::now();
        
        if (cur_node == goal_node)
        {
            auto total_loop_end = std::chrono::high_resolution_clock::now();
            auto total_loop_dur = std::chrono::duration_cast<std::chrono::duration<double>>(
                total_loop_end - total_loop_start).count();
            
            if (astar_params_.debug_viz){
                // std::cout << "Total dur: " << total_loop_dur*1000                 << std::endl;
                // std::cout << "Preloop dur: " << preloop_dur  *1000                  << ", pct:" << preloop_dur / total_loop_dur * 100 << "%" << std::endl;
                // std::cout << "loop dur: " << loop_durs         *1000       << ", pct:" << loop_durs / total_loop_dur * 100<< "%" << std::endl;
                // std::cout << "  explore_nb dur: " << explore_nb_durs *1000 << ", pct:" << explore_nb_durs / total_loop_dur * 100 << "%" << std::endl;
                // std::cout << "  chkpt_1 dur: " << chkpt_1_durs *1000       << ", pct:" << chkpt_1_durs / total_loop_dur * 100<< "%" << std::endl;
                // std::cout << "  get_nb dur: " << get_nb_durs   *1000       << ", pct:" << get_nb_durs / total_loop_dur * 100<< "%" << std::endl;
                
                // std::cout << "  get_occ dur: " << get_occ_durs *1000         << ", pct:" << get_occ_durs / total_loop_dur * 100<< "%" << std::endl;

                // std::cout << "[a_star] Goal found at iteration " << num_iter << std::endl;
            }

            // Goal reached, terminate search and obtain path
            tracePath(cur_node);

            return true;
        }

        auto get_nb_start = std::chrono::high_resolution_clock::now();
        
        // std::cout << "Before getNeighbours" << std::endl;

        common_->getNeighbours(cur_node, neighbours);

        // std::cout << "After getNeighbours" << std::endl;

        auto get_nb_end = std::chrono::high_resolution_clock::now();
        auto explore_nb_start = std::chrono::high_resolution_clock::now();

        // Explore neighbors of current node
        for (auto nb_node : neighbours)
        {
            // std::cout << "[a_star] Exploring neighbor " << common_->getPosStr(nb_node).c_str() << std::endl;
            double tent_g_cost = g_cost_[cur_node] + cost_function(cur_node, nb_node);

            // If tentative cost is better than previously computed cost, then update costs
            if (g_cost_.find(nb_node) == g_cost_.end() || tent_g_cost < g_cost_[nb_node])
            {
                g_cost_[nb_node] = tent_g_cost;
                // The tie_breaker is used to assign a larger weight to the h_cost and favour expanding nodes closer towards the goal
                double f_cost = g_cost_[nb_node] + astar_params_.tie_breaker * cost_function(nb_node, goal_node);

                // If not in closed list: set parent and add to open list
                if (closed_list_.find(nb_node) == closed_list_.end()) 
                {
                    came_from_[nb_node] = cur_node;
                    open_list_.put(nb_node, f_cost);
                }
                // No need to update parents for nodes already in closed list, paths leading up to current node is alr the most optimal
            }
        }
        num_iter++;

        auto explore_nb_end = std::chrono::high_resolution_clock::now();
        auto loop_end = std::chrono::high_resolution_clock::now();
        explore_nb_durs += std::chrono::duration_cast<std::chrono::duration<double>>(
            explore_nb_end - explore_nb_start).count();
        loop_durs += std::chrono::duration_cast<std::chrono::duration<double>>(
            loop_end - loop_start).count();
        chkpt_1_durs += std::chrono::duration_cast<std::chrono::duration<double>>(
            chkpt_1_end - chkpt_1_start).count();
        get_nb_durs += std::chrono::duration_cast<std::chrono::duration<double>>(
            get_nb_end - get_nb_start).count();
    }

    std::cerr << "[a_star] Unable to find goal with maximum iteration " << num_iter << std::endl;

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
std::vector<Eigen::Vector3d> AStarPlanner::getPathPosRaw()
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
      common_->idxToPos(*itr, node_pos);
      closed_list_pos.push_back(node_pos);
    }

    return closed_list_pos;
}

void AStarPlanner::tracePath(PosIdx final_node)
{
    // Clear existing data structures
    path_idx_.clear();
    path_pos_.clear();

    // Trace back the nodes through the pointer to their parent
    PosIdx cur_node = final_node;
    while (!(cur_node == came_from_[cur_node]))
    {
        path_idx_.push_back(cur_node);
        cur_node = came_from_[cur_node];
    }
    // Push back the start node
    path_idx_.push_back(cur_node);

    // Reverse the order of the path so that it goes from start to goal
    std::reverse(path_idx_.begin(), path_idx_.end());

    // For each gridnode, get the position and index,
    // So we can obtain a path in terms of indices and positions
    for (auto idx : path_idx_)
    {
        Eigen::Vector3d gridnode_pos;
        common_->idxToPos(idx, gridnode_pos);

        path_pos_.push_back(gridnode_pos);
    }
}
