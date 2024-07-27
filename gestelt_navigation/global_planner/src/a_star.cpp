#include <global_planner/a_star.h>

#define L2_cost

AStarPlanner::AStarPlanner(std::shared_ptr<GridMap> grid_map, const AStarParams& astar_params):
    astar_params_(astar_params)
{
    common_.reset(new PlannerCommon(grid_map));
}

AStarPlanner::AStarPlanner( const std::map<int, std::shared_ptr<DynamicVoronoi>>& dyn_voro_arr, 
                            const int& z_separation_cm,
                            const AStarParams& astar_params,
                            std::shared_ptr<std::unordered_set<VCell_T>> resrv_tbl
                            )
                            : astar_params_(astar_params)
{
    z_separation_cm_ = z_separation_cm;
    dyn_voro_arr_ = dyn_voro_arr;
    resrv_tbl_ = resrv_tbl_;
}

void AStarPlanner::reset()
{
    // Clear planning data
    closed_list_.clear();
    closed_list_v_.clear();
    open_list_.clear();
    came_from_.clear();
    g_cost_.clear();

    // Voronoi planning data structs
    open_list_v_.clear();
    g_cost_v_.clear();
    came_from_v_.clear();

    marked_bubble_cells_.clear();

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

            publishClosedList(getClosedList(), closed_list_viz_pub_);
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

        common_->getNeighbours(cur_node, neighbours);

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

void AStarPlanner::expandVoronoiBubble(
    const VCell& cell, 
    const bool& makeGoalBubble)
{
    std::queue<VCell> q;
    q.push(cell);

    IntPoint grid(cell.x, cell.y);
    marked_bubble_cells_[cell.z_cm].insert(grid);

    while(!q.empty()) {
        VCell p = q.front();
        q.pop();

        for (int dx=-1; dx<=1; dx++) {
            int nx = p.x + dx;
            if (nx<0 || nx >= dyn_voro_arr_[cell.z_cm]->getSizeX()) {
                continue; // Skip if outside map
            }
            for (int dy=-1; dy<=1; dy++) {
                int ny = p.y + dy;
                if (dx && dy) {
                    continue; // skip if not 4 connected cell connection
                }
                if (ny<0 || ny >= dyn_voro_arr_[cell.z_cm]->getSizeY()){
                    continue; // Skip if outside map
                } 

                IntPoint nb_grid(nx, ny);

                if (marked_bubble_cells_[cell.z_cm].find(nb_grid) != marked_bubble_cells_[cell.z_cm].end()) 
                    continue; // If already added to marked bubble list, then skip

                if (dyn_voro_arr_[cell.z_cm]->getSqrDistance(nx,ny)<1) 
                    continue;   // Skip if occupied or near obstacle

                // Node *nd = MemoryManager<Node>::getNew();
                // g_cost_v_[nb_grid] = std::numeric_limits<double>::max();
                // double h_cost = getL2Norm(nb_grid, goal);
                // open_list_v_.put(nb_grid, std::numeric_limits<double>::max());
                
                // mark as closed
                marked_bubble_cells_[cell.z_cm].insert(nb_grid);

                if (!dyn_voro_arr_[cell.z_cm]->isVoronoi(nx,ny)){
                    VCell nb_grid_3d(nx, ny, cell.z_cm);
                    q.push(nb_grid_3d); // If not voronoi then push to list
                }
            }

        }
    }
}

bool AStarPlanner::generatePlanVoronoi(const Eigen::Vector3d& start_pos_3d, const Eigen::Vector3d& goal_pos_3d)
{
    std::function<double(const VCell&, const VCell&)> cost_function;

    switch ( astar_params_.cost_function_type ) {
        case 0:
            // std::cout << "[AStar]: Using octile distance cost " << std::endl; 
            cost_function = getOctileDistV;
            break;
        case 1:
            // std::cout << "[AStar]: Using L1 Norm" << std::endl; 
            cost_function = getL1NormV; 
            break;
        case 2:
            // std::cout << "[AStar]: Using L2 Norm " << std::endl; 
            cost_function = getL2NormV;
            break;
        case 3:
            // std::cout << "[AStar]: Using Chebyshev Distance" << std::endl;
            cost_function = getChebyshevDistV;
            break;
        default: 
            // std::cout << "[AStar]: Using Octile Distance" << std::endl;
            cost_function = getOctileDistV;
            break;
    }

    return generatePlanVoronoi(start_pos_3d, goal_pos_3d, cost_function);
}

bool AStarPlanner::generatePlanVoronoi(const Eigen::Vector3d& start_pos_3d, const Eigen::Vector3d& goal_pos_3d, 
                                std::function<double(const VCell&, const VCell&)> cost_function)
{
    reset();

    int start_z_cm = roundUpMult((int) (start_pos_3d(2) * 100), z_separation_cm_);
    int goal_z_cm = roundUpMult((int) (goal_pos_3d(2) * 100), z_separation_cm_);
    std::cout << "start_z: " <<  start_pos_3d(2) << " m rounded to " << start_z_cm << " cm" << std::endl;
    std::cout << "goal_z: " <<  goal_pos_3d(2) << " m rounded to " << goal_z_cm << " cm" << std::endl;

    INTPOINT start_node_2d, goal_node_2d;
    // Search takes place in index space. So we first convert 3d real world positions into indices
    if (!dyn_voro_arr_[start_z_cm]->posToIdx(DblPoint(start_pos_3d(0), start_pos_3d(1)), start_node_2d) 
        || !dyn_voro_arr_[goal_z_cm]->posToIdx(DblPoint(goal_pos_3d(0), goal_pos_3d(1)), goal_node_2d))
    {   
        std::cerr << "[a_star] Start or goal position is not within map bounds!" << std::endl;
        return false;
    }

    if (dyn_voro_arr_[start_z_cm]->isOccupied(start_node_2d)){
        std::cerr << "[a_star] Start position in obstacle!" << std::endl;
        return false;
    }
    if (dyn_voro_arr_[goal_z_cm]->isOccupied(goal_node_2d)){
        std::cerr << "[a_star] Goal position in obstacle!" << std::endl;
        return false;
    }

    VCell start_node(start_node_2d.x, start_node_2d.y, start_z_cm);
    VCell goal_node(goal_node_2d.x, goal_node_2d.y, goal_z_cm);

    // set start and goal cell as obstacle
    dyn_voro_arr_[start_node.z_cm]->setObstacle(start_node.x, start_node.y);
    dyn_voro_arr_[goal_node.z_cm]->setObstacle(goal_node.x, goal_node.y);

    dyn_voro_arr_[start_node.z_cm]->update(); // update distance map and Voronoi diagram
    dyn_voro_arr_[goal_node.z_cm]->update(); // update distance map and Voronoi diagram

    // Create voronoi bubble around start and goal
    expandVoronoiBubble(start_node,  false );
    expandVoronoiBubble(goal_node, true );

    dyn_voro_arr_[start_node.z_cm]->removeObstacle(start_node.x, start_node.y);
    dyn_voro_arr_[goal_node.z_cm]->removeObstacle(goal_node.x, goal_node.y  );

    came_from_v_[start_node] = start_node;
    g_cost_v_[start_node] = 0;

    open_list_v_.put(start_node, 0); // start_node has 0 f cost

    int num_iter = 0;
    std::vector<Eigen::Vector3i> neighbours; // 3d indices of neighbors
    std::unordered_set<IntPoint> marked_bbl_cells_; // Cells that are marked as part of the voronoi bubble

    while (!open_list_v_.empty() && num_iter < astar_params_.max_iterations)
    {
        // if (num_iter%100 == 1){
        //     std::cout << "[a_star] Iteration " << num_iter << std::endl;
        //     publishClosedList(getClosedListVoronoi(), closed_list_viz_pub_, "local_map_origin");
        //     // ros::Duration(0.1).sleep();
        // }

        VCell cur_node = open_list_v_.get();
        closed_list_v_.insert(cur_node);

        if (cur_node == goal_node)
        {
            // Goal reached, terminate search and obtain path
            tracePathVoronoi(cur_node);

            return true;
        }

        IntPoint cur_node_2d(cur_node.x, cur_node.y);

        // Get neighbours that are within the map
        dyn_voro_arr_[cur_node.z_cm]->getVoroNeighbors(cur_node_2d, neighbours, marked_bubble_cells_[cur_node.z_cm]);

        // Explore neighbors of current node
        for (const Eigen::Vector3i& nb_node_eig : neighbours)
        {
            VCell nb_node(nb_node_eig(0), nb_node_eig(1), nb_node_eig(2));
            // std::cout << "[a_star] Exploring neighbor " << common_->getPosStr(nb_node).c_str() << std::endl;
            double tent_g_cost = g_cost_v_[cur_node] + cost_function(cur_node, nb_node);

            // If g_cost is not found or tentative cost is better than previously computed cost, then update costs
            if (g_cost_v_.find(nb_node) == g_cost_v_.end() || tent_g_cost < g_cost_v_[nb_node])
            {
                g_cost_v_[nb_node] = tent_g_cost;
                // The tie_breaker is used to assign a larger weight to the h_cost and favour expanding nodes closer towards the goal
                double f_cost = g_cost_v_[nb_node] + astar_params_.tie_breaker * cost_function(nb_node, goal_node);

                // If not in closed list: set parent and add to open list
                if (closed_list_v_.find(nb_node) == closed_list_v_.end()) 
                {
                    came_from_v_[nb_node] = cur_node;
                    open_list_v_.put(nb_node, f_cost);
                }
                // No need to update parents for nodes already in closed list, paths leading up to current node is alr the most optimal
            }
        }
        num_iter++;
    }

    std::cerr   << "[a_star] Unable to find goal node ("<< goal_node.x << ", " << goal_node.y 
                << ") with maximum iteration " << num_iter << std::endl;

    return false;
}

bool AStarPlanner::generatePlanVoroT(  const Eigen::Vector3d& start_pos_3d, 
                                        const Eigen::Vector3d& goal_pos_3d)
{
    std::function<double(const VCell_T&, const VCell_T&)> cost_function;

    switch ( astar_params_.cost_function_type ) {
        case 0:
            // std::cout << "[AStar]: Using octile distance cost " << std::endl; 
            cost_function = getOctileDistVT;
            break;
        case 1:
            // std::cout << "[AStar]: Using L1 Norm" << std::endl; 
            cost_function = getL1NormVT; 
            break;
        case 2:
            // std::cout << "[AStar]: Using L2 Norm " << std::endl; 
            cost_function = getL2NormVT;
            break;
        case 3:
            // std::cout << "[AStar]: Using Chebyshev Distance" << std::endl;
            cost_function = getChebyshevDistVT;
            break;
        default: 
            // std::cout << "[AStar]: Using Octile Distance" << std::endl;
            cost_function = getOctileDistVT;
            break;
    }

    // return generatePlanVoroT(start_pos_3d, goal_pos_3d, cost_function);
}

// bool AStarPlanner::generatePlanVoroT(   const Eigen::Vector3d& start_pos_3d, 
//                                         const Eigen::Vector3d& goal_pos_3d, 
//                                         std::function<double(const VCellT&, const VCellT&)> cost_function)
// {
//     reset();

//     int start_z_cm = roundUpMult((int) (start_pos_3d(2) * 100), z_separation_cm_);
//     int goal_z_cm = roundUpMult((int) (goal_pos_3d(2) * 100), z_separation_cm_);
//     std::cout << "start_z: " <<  start_pos_3d(2) << " m rounded to " << start_z_cm << " cm" << std::endl;
//     std::cout << "goal_z: " <<  goal_pos_3d(2) << " m rounded to " << goal_z_cm << " cm" << std::endl;

//     INTPOINT start_node_2d, goal_node_2d;
//     // Search takes place in index space. So we first convert 3d real world positions into indices
//     if (!dyn_voro_arr_[start_z_cm]->posToIdx(DblPoint(start_pos_3d(0), start_pos_3d(1)), start_node_2d) 
//         || !dyn_voro_arr_[goal_z_cm]->posToIdx(DblPoint(goal_pos_3d(0), goal_pos_3d(1)), goal_node_2d))
//     {   
//         std::cerr << "[a_star] Start or goal position is not within map bounds!" << std::endl;
//         return false;
//     }

//     if (dyn_voro_arr_[start_z_cm]->isOccupied(start_node_2d)){
//         std::cerr << "[a_star] Start position in obstacle!" << std::endl;
//         return false;
//     }
//     if (dyn_voro_arr_[goal_z_cm]->isOccupied(goal_node_2d)){
//         std::cerr << "[a_star] Goal position in obstacle!" << std::endl;
//         return false;
//     }

//     VCell start_node(start_node_2d.x, start_node_2d.y, start_z_cm);
//     VCell goal_node(goal_node_2d.x, goal_node_2d.y, goal_z_cm);

//     // set start and goal cell as obstacle
//     dyn_voro_arr_[start_node.z_cm]->setObstacle(start_node.x, start_node.y);
//     dyn_voro_arr_[goal_node.z_cm]->setObstacle(goal_node.x, goal_node.y);

//     dyn_voro_arr_[start_node.z_cm]->update(); // update distance map and Voronoi diagram
//     dyn_voro_arr_[goal_node.z_cm]->update(); // update distance map and Voronoi diagram

//     // Create voronoi bubble around start and goal
//     expandVoronoiBubble(start_node,  false );
//     expandVoronoiBubble(goal_node, true );

//     dyn_voro_arr_[start_node.z_cm]->removeObstacle(start_node.x, start_node.y);
//     dyn_voro_arr_[goal_node.z_cm]->removeObstacle(goal_node.x, goal_node.y  );

//     came_from_v_[start_node] = start_node;
//     g_cost_v_[start_node] = 0;

//     open_list_v_.put(start_node, 0); // start_node has 0 f cost

//     int num_iter = 0;
//     std::vector<Eigen::Vector3i> neighbours; // 3d indices of neighbors
//     std::unordered_set<IntPoint> marked_bbl_cells_; // Cells that are marked as part of the voronoi bubble

//     while (!open_list_v_.empty() && num_iter < astar_params_.max_iterations)
//     {
//         // if (num_iter%100 == 1){
//         //     std::cout << "[a_star] Iteration " << num_iter << std::endl;
//         //     publishClosedList(getClosedListVoronoi(), closed_list_viz_pub_, "local_map_origin");
//         //     // ros::Duration(0.1).sleep();
//         // }

//         VCell cur_node = open_list_v_.get();
//         closed_list_v_.insert(cur_node);

//         if (cur_node == goal_node)
//         {
//             // Goal reached, terminate search and obtain path
//             tracePathVoronoi(cur_node);

//             return true;
//         }

//         IntPoint cur_node_2d(cur_node.x, cur_node.y);

//         // Get neighbours that are within the map
//         dyn_voro_arr_[cur_node.z_cm]->getVoroNeighbors(cur_node_2d, neighbours, marked_bubble_cells_[cur_node.z_cm]);

//         // Explore neighbors of current node
//         for (const Eigen::Vector3i& nb_node_eig : neighbours)
//         {
//             VCell nb_node(nb_node_eig(0), nb_node_eig(1), nb_node_eig(2));
//             // std::cout << "[a_star] Exploring neighbor " << common_->getPosStr(nb_node).c_str() << std::endl;
//             double tent_g_cost = g_cost_v_[cur_node] + cost_function(cur_node, nb_node);

//             // If g_cost is not found or tentative cost is better than previously computed cost, then update costs
//             if (g_cost_v_.find(nb_node) == g_cost_v_.end() || tent_g_cost < g_cost_v_[nb_node])
//             {
//                 g_cost_v_[nb_node] = tent_g_cost;
//                 // The tie_breaker is used to assign a larger weight to the h_cost and favour expanding nodes closer towards the goal
//                 double f_cost = g_cost_v_[nb_node] + astar_params_.tie_breaker * cost_function(nb_node, goal_node);

//                 // If not in closed list: set parent and add to open list
//                 if (closed_list_v_.find(nb_node) == closed_list_v_.end()) 
//                 {
//                     came_from_v_[nb_node] = cur_node;
//                     open_list_v_.put(nb_node, f_cost);
//                 }
//                 // No need to update parents for nodes already in closed list, paths leading up to current node is alr the most optimal
//             }
//         }
//         num_iter++;
//     }

//     std::cerr   << "[a_star] Unable to find goal node ("
//                 << goal_node.x << ", " << goal_node.y 
//                 << ") with maximum iteration " << num_iter << std::endl;

//     return false;
// }