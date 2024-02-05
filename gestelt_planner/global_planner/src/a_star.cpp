#include <global_planner/a_star.h>

#define L2_cost

AStarPlanner::AStarPlanner(std::shared_ptr<GridMap> grid_map, const AStarParams& astar_params):
    astar_params_(astar_params)
{
    common_.reset(new PlannerCommon(grid_map));
    occ_map_ = std::make_unique<OccMap<OccNodePtr>>(
        common_->map_.global_map_num_cells_(0), 
        common_->map_.global_map_num_cells_(1), 
        common_->map_.global_map_num_cells_(2));
}

void AStarPlanner::reset()
{
    closed_list_.clear();
    open_list_ = std::priority_queue<OccNodePtr, std::vector<OccNodePtr>, OccNode::CompareCostPtr>();
}

bool AStarPlanner::generatePlan(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &goal_pos)
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

    Vector3i start_idx, goal_idx;
    common_->posToIdx(start_pos, start_idx);
    common_->posToIdx(goal_pos, goal_idx);

    OccNodePtr start_node = occ_map_(start_idx);
    OccNodePtr goal_node = occ_map_(goal_idx);

    start_node->g_cost = 0.0;

    #if defined(L1_cost)
        start_node->f_cost = astar_params_.tie_breaker * common_->getL1Norm(start_node, goal_node);
    #elif defined(L2_cost)
        start_node->f_cost = astar_params_.tie_breaker * common_->getL2Norm(start_node, goal_node);
    #else 
        start_node->f_cost = astar_params_.tie_breaker * common_->getL2Norm(start_node, goal_node);
    #endif

    start_node->parent = nullptr;

    addToOpenlist(start_node);

    int num_iter = 0;

    std::vector<OccNodePtr> nb_nodes; // 3d indices of neighbors

    auto preloop_end = std::chrono::high_resolution_clock::now();
    auto preloop_dur = std::chrono::duration_cast<std::chrono::duration<double>>(
        preloop_end - preloop_start).count();
    double explore_nb_durs = 0;
    double loop_durs = 0;
    double chkpt_1_durs = 0;
    double get_nb_durs = 0;
    double get_occ_durs = 0;

    while (!open_list_.empty())
    {
        auto loop_start = std::chrono::high_resolution_clock::now();

        if (num_iter%1000 == 1){
            std::cout << "[a_star] Iteration " << num_iter << std::endl;

            publishVizPoints(getClosedList(), closed_list_viz_pub_);
        }

        auto chkpt_1_start = std::chrono::high_resolution_clock::now();

        OccNodePtr cur_node = popOpenlist();
        addToClosedlist(cur_node); // Mark as visited

        auto chkpt_1_end = std::chrono::high_resolution_clock::now();

        if (*cur_node == *goal_node)
        {
            auto total_loop_end = std::chrono::high_resolution_clock::now();
            auto total_loop_dur = std::chrono::duration_cast<std::chrono::duration<double>>(
                total_loop_end - total_loop_start).count();

            std::cout << "Total dur: " << total_loop_dur                 << std::endl;
            std::cout << "Preloop dur: " << preloop_dur                    << "s, pct:" << preloop_dur / total_loop_dur * 100 << "%" << std::endl;
            std::cout << "loop dur: " << loop_durs                << "s, pct:" << loop_durs / total_loop_dur * 100<< "%" << std::endl;
            std::cout << "  explore_nb dur: " << explore_nb_durs  << "s, pct:" << explore_nb_durs / total_loop_dur * 100 << "%" << std::endl;
            std::cout << "  chkpt_1 dur: " << chkpt_1_durs        << "s, pct:" << chkpt_1_durs / total_loop_dur * 100<< "%" << std::endl;
            std::cout << "  get_nb dur: " << get_nb_durs          << "s, pct:" << get_nb_durs / total_loop_dur * 100<< "%" << std::endl;
            
            std::cout << "  get_occ dur: " << get_occ_durs          << "s, pct:" << get_occ_durs / total_loop_dur * 100<< "%" << std::endl;

            // std::cout << "[a_star] Goal found at iteration " << num_iter << std::endl;
            // Goal reached, terminate search and obtain path
            tracePath(cur_node);

            return true;
        }

        auto get_nb_start = std::chrono::high_resolution_clock::now();
        
        common_->getNeighborsOG(cur_node, nb_nodes);

        auto get_nb_end = std::chrono::high_resolution_clock::now();

        auto explore_nb_start = std::chrono::high_resolution_clock::now();

        // Explore neighbors of current node
        for (size_t i = 0; i < nb_nodes.size(); i++)
        {
            // std::cout << "[a_star] Exploring neighbor " << common_->getPosStr(nb_node).c_str() << std::endl;
            #if defined(L1_cost)
                // double tent_g_cost = cur_node->g_cost + common_->nb_8con_dist_l1_[nb_8con_idxs[i]];
                double tent_g_cost = cur_node->g_cost + common_->getL1Norm(cur_node, nb_nodes[i]);
            #elif defined(L2_cost)
                double tent_g_cost = cur_node->g_cost + common_->getL2Norm(cur_node, nb_nodes[i]);
            #else 
                double tent_g_cost = cur_node->g_cost + common_->nb_8con_dist_l2_[nb_8con_idxs[i]];
            #endif

            // If tentative cost is better than previously computed cost, then update the g and f cost
            if (tent_g_cost < nb_nodes[i]->g_cost)
            {
                nb_nodes[i]->g_cost = tent_g_cost;
                // The tie_breaker is used to assign a larger weight to the h_cost and favour 
                // expanding nodes closer towards the goal
                #if defined(L1_cost)
                    nb_nodes[i]->f_cost = tent_g_cost + astar_params_.tie_breaker * common_->getL1Norm(nb_nodes[i], goal_node);
                #elif defined(L2_cost)
                    nb_nodes[i]->f_cost = tent_g_cost + astar_params_.tie_breaker * common_->getL2Norm(nb_nodes[i], goal_node);
                #else 
                    nb_nodes[i]->f_cost = tent_g_cost + astar_params_.tie_breaker * common_->getL2Norm(nb_nodes[i], goal_node);
                #endif
            }

            // If not already in closed list: set parent and add to open list
            if (!isInClosedList(nb_nodes[i])) 
            {
                nb_nodes[i]->parent = cur_node;
                addToOpenlist(nb_nodes[i]);
            }
            // No need to update parents for nodes already in closed list, paths leading up to current node is alr the most optimal
        }

        auto explore_nb_end = std::chrono::high_resolution_clock::now();

        num_iter++;

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

// bool AStarPlanner::generatePlan(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &goal_pos)
// {
//     auto total_loop_start = std::chrono::high_resolution_clock::now();

//     auto preloop_start = std::chrono::high_resolution_clock::now();

//     reset();

//     // Search takes place in index space. So we first convert 3d real world positions into indices
//     if (!common_->isInGlobalMap(start_pos) || !common_->isInGlobalMap(goal_pos))
//     {
//         std::cerr << "[a_star] Start or goal position not in global map!" << std::endl;
//         return false;
//     }

//     if (common_->getOccupancy(start_pos)){
//         std::cerr << "[a_star] Start position in obstacle!" << std::endl;
//         return false;
//     }
//     if (common_->getOccupancy(goal_pos)){
//         std::cerr << "[a_star] Goal position in obstacle!" << std::endl;
//         return false;
//     }

//     Vector3i start_idx, goal_idx;
//     common_->posToIdx(start_pos, start_idx);
//     common_->posToIdx(goal_pos, goal_idx);

//     OccNodePtr start_node = std::make_shared<OccNode>(start_idx);
//     OccNodePtr goal_node = std::make_shared<OccNode>(goal_idx);

//     start_node->g_cost = 0.0;

//     #if defined(L1_cost)
//         start_node->f_cost = astar_params_.tie_breaker * common_->getL1Norm(start_node, goal_node);
//     #elif defined(L2_cost)
//         start_node->f_cost = astar_params_.tie_breaker * common_->getL2Norm(start_node, goal_node);
//     #else 
//         start_node->f_cost = astar_params_.tie_breaker * common_->getL2Norm(start_node, goal_node);
//     #endif

//     start_node->parent = nullptr;

//     addToOpenlist(start_node);

//     int num_iter = 0;

//     std::vector<OccNodePtr> nb_nodes; // 3d indices of neighbors

//     auto preloop_end = std::chrono::high_resolution_clock::now();

//     auto preloop_dur = std::chrono::duration_cast<std::chrono::duration<double>>(
//         preloop_end - preloop_start).count();
    
//     double explore_nb_durs = 0;
//     double loop_durs = 0;
//     double chkpt_1_durs = 0;
//     double get_nb_durs = 0;

//     double get_occ_durs = 0;

//     while (!open_list_.empty())
//     {
//         auto loop_start = std::chrono::high_resolution_clock::now();

//         if (num_iter%1000 == 1){
//             std::cout << "[a_star] Iteration " << num_iter << std::endl;

//             publishVizPoints(getClosedList(), closed_list_viz_pub_);
//         }

//         auto chkpt_1_start = std::chrono::high_resolution_clock::now();

//         OccNodePtr cur_node = popOpenlist();
//         addToClosedlist(cur_node); // Mark as visited

//         auto chkpt_1_end = std::chrono::high_resolution_clock::now();

//         if (*cur_node == *goal_node)
//         {
//             auto total_loop_end = std::chrono::high_resolution_clock::now();
//             auto total_loop_dur = std::chrono::duration_cast<std::chrono::duration<double>>(
//                 total_loop_end - total_loop_start).count();

//             std::cout << "Total dur: " << total_loop_dur                 << std::endl;
//             std::cout << "Preloop dur: " << preloop_dur                    << "s, pct:" << preloop_dur / total_loop_dur * 100 << "%" << std::endl;
//             std::cout << "loop dur: " << loop_durs                << "s, pct:" << loop_durs / total_loop_dur * 100<< "%" << std::endl;
//             std::cout << "  explore_nb dur: " << explore_nb_durs  << "s, pct:" << explore_nb_durs / total_loop_dur * 100 << "%" << std::endl;
//             std::cout << "  chkpt_1 dur: " << chkpt_1_durs        << "s, pct:" << chkpt_1_durs / total_loop_dur * 100<< "%" << std::endl;
//             std::cout << "  get_nb dur: " << get_nb_durs          << "s, pct:" << get_nb_durs / total_loop_dur * 100<< "%" << std::endl;
            
//             std::cout << "  get_occ dur: " << get_occ_durs          << "s, pct:" << get_occ_durs / total_loop_dur * 100<< "%" << std::endl;

//             // std::cout << "[a_star] Goal found at iteration " << num_iter << std::endl;
//             // Goal reached, terminate search and obtain path
//             tracePath(cur_node);

//             return true;
//         }

//         auto get_nb_start = std::chrono::high_resolution_clock::now();
        
//         common_->getNeighborsOG(cur_node, nb_nodes);

//         auto get_nb_end = std::chrono::high_resolution_clock::now();

//         auto explore_nb_start = std::chrono::high_resolution_clock::now();

//         // Explore neighbors of current node
//         for (size_t i = 0; i < nb_nodes.size(); i++)
//         {
//             // std::cout << "[a_star] Exploring neighbor " << common_->getPosStr(nb_node).c_str() << std::endl;
//             #if defined(L1_cost)
//                 // double tent_g_cost = cur_node->g_cost + common_->nb_8con_dist_l1_[nb_8con_idxs[i]];
//                 double tent_g_cost = cur_node->g_cost + common_->getL1Norm(cur_node, nb_nodes[i]);
//             #elif defined(L2_cost)
//                 double tent_g_cost = cur_node->g_cost + common_->getL2Norm(cur_node, nb_nodes[i]);
//             #else 
//                 double tent_g_cost = cur_node->g_cost + common_->nb_8con_dist_l2_[nb_8con_idxs[i]];
//             #endif

//             // If tentative cost is better than previously computed cost, then update the g and f cost
//             if (tent_g_cost < nb_nodes[i]->g_cost)
//             {
//                 nb_nodes[i]->g_cost = tent_g_cost;
//                 // The tie_breaker is used to assign a larger weight to the h_cost and favour 
//                 // expanding nodes closer towards the goal
//                 #if defined(L1_cost)
//                     nb_nodes[i]->f_cost = tent_g_cost + astar_params_.tie_breaker * common_->getL1Norm(nb_nodes[i], goal_node);
//                 #elif defined(L2_cost)
//                     nb_nodes[i]->f_cost = tent_g_cost + astar_params_.tie_breaker * common_->getL2Norm(nb_nodes[i], goal_node);
//                 #else 
//                     nb_nodes[i]->f_cost = tent_g_cost + astar_params_.tie_breaker * common_->getL2Norm(nb_nodes[i], goal_node);
//                 #endif
//             }

//             // If not already in closed list: set parent and add to open list
//             if (!isInClosedList(nb_nodes[i])) 
//             {
//                 nb_nodes[i]->parent = cur_node;
//                 addToOpenlist(nb_nodes[i]);
//             }
//             // No need to update parents for nodes already in closed list, paths leading up to current node is alr the most optimal
//         }

//         auto explore_nb_end = std::chrono::high_resolution_clock::now();

//         num_iter++;

//         auto loop_end = std::chrono::high_resolution_clock::now();

//         explore_nb_durs += std::chrono::duration_cast<std::chrono::duration<double>>(
//             explore_nb_end - explore_nb_start).count();
//         loop_durs += std::chrono::duration_cast<std::chrono::duration<double>>(
//             loop_end - loop_start).count();

//         chkpt_1_durs += std::chrono::duration_cast<std::chrono::duration<double>>(
//             chkpt_1_end - chkpt_1_start).count();

//         get_nb_durs += std::chrono::duration_cast<std::chrono::duration<double>>(
//             get_nb_end - get_nb_start).count();
//     }

//     std::cerr << "[a_star] Unable to find goal with maximum iteration " << num_iter << std::endl;


//     return false;
// }


// bool AStarPlanner::generatePlan(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &goal_pos)
// {
//     reset();

//     // Search takes place in index space. So we first convert 3d real world positions into indices
//     if (!common_->isInGlobalMap(start_pos) || !common_->isInGlobalMap(goal_pos))
//     {
//         std::cerr << "[a_star] Start or goal position not in global map!" << std::endl;
//         return false;
//     }

//     if (common_->getOccupancy(start_pos)){
//         std::cerr << "[a_star] Start position in obstacle!" << std::endl;
//         return false;
//     }
//     if (common_->getOccupancy(goal_pos)){
//         std::cerr << "[a_star] Goal position in obstacle!" << std::endl;
//         return false;
//     }

//     Vector3i start_idx, goal_idx;
//     common_->posToIdx(start_pos, start_idx);
//     common_->posToIdx(goal_pos, goal_idx);

//     OccNodePtr start_node = std::make_shared<OccNode>(start_idx);
//     OccNodePtr goal_node = std::make_shared<OccNode>(goal_idx);

//     start_node->g_cost = 0.0;

//     #if defined(L1_cost)
//         start_node->f_cost = astar_params_.tie_breaker * common_->getL1Norm(start_node, goal_node);
//     #elif defined(L2_cost)
//         start_node->f_cost = astar_params_.tie_breaker * common_->getL2Norm(start_node, goal_node);
//     #else 
//         start_node->f_cost = astar_params_.tie_breaker * common_->getL2Norm(start_node, goal_node);
//     #endif

//     start_node->parent = nullptr;

//     addToOpenlist(start_node);

//     int num_iter = 0;

//     std::vector<OccNodePtr> nb_idxs; // 3d indices of neighbors
//     std::vector<int> nb_8con_idxs; // indices of neighbors in 8 connected array

//     while (!open_list_.empty())
//     {
//         if (num_iter%1000 == 0){
//             std::cout << "[a_star] Iteration " << num_iter << std::endl;

//             publishVizPoints(getClosedList(), closed_list_viz_pub_);
//         }
//         OccNodePtr cur_node = popOpenlist();
//         addToClosedlist(cur_node); // Mark as visited

//         if (*cur_node == *goal_node)
//         {
//             // std::cout << "[a_star] Goal found at iteration " << num_iter << std::endl;
//             // Goal reached, terminate search and obtain path
//             tracePath(cur_node);
//             return true;
//         }
//             common_->getNeighbors(cur_node, nb_idxs, nb_8con_idxs);

//         // Explore neighbors of current node
//         for (size_t i = 0; i < nb_idxs.size(); i++)
//         {
//             // std::cout << "[a_star] Exploring neighbor " << common_->getPosStr(nb_node).c_str() << std::endl;
//             #if defined(L1_cost)
//                 double tent_g_cost = cur_node->g_cost + common_->nb_8con_dist_l1_[nb_8con_idxs[i]];
//             #elif defined(L2_cost)
//                 double tent_g_cost = cur_node->g_cost + common_->nb_8con_dist_l2_[nb_8con_idxs[i]];
//             #else 
//                 double tent_g_cost = cur_node->g_cost + common_->nb_8con_dist_l2_[nb_8con_idxs[i]];
//             #endif

//             // If tentative cost is better than previously computed cost, then update the g and f cost
//             if (tent_g_cost < nb_idxs[i]->g_cost)
//             {
//                 nb_idxs[i]->g_cost = tent_g_cost;
//                 // The tie_breaker is used to assign a larger weight to the h_cost and favour 
//                 // expanding nodes closer towards the goal
//                 #if defined(L1_cost)
//                     nb_idxs[i]->f_cost = tent_g_cost + astar_params_.tie_breaker * common_->getL1Norm(nb_idxs[i], goal_node);
//                 #elif defined(L2_cost)
//                     nb_idxs[i]->f_cost = tent_g_cost + astar_params_.tie_breaker * common_->getL2Norm(nb_idxs[i], goal_node);
//                 #else 
//                     nb_idxs[i]->f_cost = tent_g_cost + astar_params_.tie_breaker * common_->getL2Norm(nb_idxs[i], goal_node);
//                 #endif
//             }

//             // If not already in closed list: set parent and add to open list
//             if (!isInClosedList(nb_idxs[i])) 
//             {
//                 nb_idxs[i]->parent = cur_node;
//                 addToOpenlist(nb_idxs[i]);
//             }
//             // No need to update parents for nodes already in closed list, paths leading up to current node is alr the most optimal
//         }
//         // for (OccNodePtr nb_node : nb_3d_idx)
//         // {

//         // }
//         num_iter++;
//     }

//     std::cerr << "[a_star] Unable to find goal with maximum iteration " << num_iter << std::endl;

//     return false;
// }

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

void AStarPlanner::addToOpenlist(OccNodePtr node)
{
    node->state = CellState::OPEN;
    open_list_.push(node);
}

OccNodePtr AStarPlanner::popOpenlist()
{
    OccNodePtr node = open_list_.top();
    open_list_.pop();
    return node;
}

void AStarPlanner::addToClosedlist(OccNodePtr node)
{
    node->state = CellState::CLOSED; // move current node to closed set.
    closed_list_.insert(node);
}

bool AStarPlanner::isInClosedList(OccNodePtr node)
{
    if (closed_list_.find(node) != closed_list_.end())
    {
        return true;
    }
    return false;
}

void AStarPlanner::tracePath(OccNodePtr node)
{
    // Clear existing data structures
    path_gridnode_.clear();
    path_idx_.clear();
    path_pos_.clear();

    // Trace back the nodes through the pointer to their parent
    OccNodePtr cur_node = node;
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
