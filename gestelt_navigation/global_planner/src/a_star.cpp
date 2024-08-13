#include <global_planner/a_star.h>

#define L2_cost

AStarPlanner::AStarPlanner( const AStarParams& astar_params,
                            std::shared_ptr<std::unordered_set<Eigen::Vector4i>> resrv_tbl
                            ): astar_params_(astar_params)
{
    resrv_tbl_ = resrv_tbl;
}

void AStarPlanner::assignVoroMap(const std::map<int, std::shared_ptr<DynamicVoronoi>>& dyn_voro_arr,
                                const int& z_separation_cm,
                                const double& local_origin_x,
                                const double& local_origin_y,
                                const double& max_height)
{
    dyn_voro_arr_ = dyn_voro_arr;
    z_separation_cm_ = z_separation_cm;
    local_origin_x_ = local_origin_x;
    local_origin_y_ = local_origin_y;
    max_height_ = max_height;
}

void AStarPlanner::reset()
{
    // Clear planning data

    // Voronoi planning data structs

    marked_bubble_cells_.clear();

    // Space time Voronoi planning data structs
    g_cost_v_.clear();
    came_from_vt_.clear();
    open_list_vt_.clear();
    closed_list_vt_.clear();

}

void AStarPlanner::expandVoronoiBubbleT(
    const VCell_T& origin_cell)
{
    std::queue<IntPoint> q;
    q.push(IntPoint(origin_cell.x, origin_cell.y));

    IntPoint grid(origin_cell.x, origin_cell.y);
    marked_bubble_cells_[origin_cell.z_cm] = std::unordered_set<IntPoint>();

    marked_bubble_cells_[origin_cell.z_cm].insert(grid);

    while(!q.empty()) {
        IntPoint cur_cell = q.front();
        q.pop();

        for (int dx=-1; dx<=1; dx++) {
            int nx = cur_cell.x + dx;
            if (nx<0 || nx >= (int) dyn_voro_arr_[origin_cell.z_cm]->getSizeX()) {
                continue; // Skip if outside map
            }
            for (int dy=-1; dy<=1; dy++) {
                int ny = cur_cell.y + dy;
                if (dx && dy) {
                    continue; // skip if not 4 connected connection
                }
                if (ny<0 || ny >= (int) dyn_voro_arr_[origin_cell.z_cm]->getSizeY()){
                    continue; // Skip if outside map
                } 

                IntPoint nb_grid(nx, ny);

                if (marked_bubble_cells_[origin_cell.z_cm].find(nb_grid) != marked_bubble_cells_[origin_cell.z_cm].end()) 
                {
                    continue; // If already added to marked bubble list, then skip
                }

                if (dyn_voro_arr_[origin_cell.z_cm]->getSqrDistance(nx,ny) < 1) 
                {
                    continue;   // Skip if occupied or near obstacle
                }

                // mark as closed bubble cells
                marked_bubble_cells_[origin_cell.z_cm].insert(nb_grid);

                if (!dyn_voro_arr_[origin_cell.z_cm]->isVoronoi(nx,ny)){ 
                    // if 4-con neighbor is not voronoi then push to list
                    q.push(nb_grid); 
                }
            }

        }
    }
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

    return generatePlanVoroT(start_pos_3d, goal_pos_3d, cost_function);
}


bool AStarPlanner::generatePlanVoroT(   const Eigen::Vector3d& start_pos_3d, 
                                        const Eigen::Vector3d& goal_pos_3d, 
                                        std::function<double(const VCell_T&, const VCell_T&)> cost_function)
{
    reset();
    
    int start_z_cm = roundToMultInt((int) (start_pos_3d(2) * 100), z_separation_cm_);
    int goal_z_cm = roundToMultInt((int) (goal_pos_3d(2) * 100), z_separation_cm_);
    // std::cout << astar_params_.drone_id << ": start_z: " <<  start_pos_3d(2) << " m rounded to " << start_z_cm << " cm" << std::endl;
    // std::cout << astar_params_.drone_id << ": goal_z: " <<  goal_pos_3d(2) << " m rounded to " << goal_z_cm << " cm" << std::endl;
    
    INTPOINT start_node_2d, goal_node_2d;

    // Search takes place in index space. So we first convert 3d real world positions into indices
    if (!dyn_voro_arr_[start_z_cm]->posToIdx(DblPoint(start_pos_3d(0), start_pos_3d(1)), start_node_2d) 
        || !dyn_voro_arr_[goal_z_cm]->posToIdx(DblPoint(goal_pos_3d(0), goal_pos_3d(1)), goal_node_2d))
    {   
        std::cerr << "[a_star] Start or goal position is not within map bounds!" << std::endl;
        return false;
    }

    if (dyn_voro_arr_[start_z_cm]->isOccupied(start_node_2d)){
        std::cerr << "[space-time A*] Start position in obstacle!" << std::endl;
        return false;
    }
    if (dyn_voro_arr_[goal_z_cm]->isOccupied(goal_node_2d)){
        std::cerr << "[space-time A*] Goal position in obstacle!" << std::endl;
        return false;
    }

    VCell_T start_node(start_node_2d.x, start_node_2d.y, start_z_cm, 0);
    VCell_T goal_node(goal_node_2d.x, goal_node_2d.y, goal_z_cm, -1);

    ////////
    // Method A: use voronoi bubble expansion
    ////////

    // set start and goal cell as obstacle

    dyn_voro_arr_[start_node.z_cm]->setObstacle(start_node.x, start_node.y);
    dyn_voro_arr_[goal_node.z_cm]->setObstacle(goal_node.x, goal_node.y);

    dyn_voro_arr_[start_node.z_cm]->update(); // update distance map and Voronoi diagram
    dyn_voro_arr_[goal_node.z_cm]->update(); // update distance map and Voronoi diagram

    std::cout << astar_params_.drone_id << ": before expandVoronoiBubbleT" << std::endl;

    // Create voronoi bubble around start and goal
    expandVoronoiBubbleT(start_node);
    expandVoronoiBubbleT(goal_node);

    std::cout << astar_params_.drone_id << ": before removeObstacle" << std::endl;

    dyn_voro_arr_[start_node.z_cm]->removeObstacle(start_node.x, start_node.y);
    dyn_voro_arr_[goal_node.z_cm]->removeObstacle(goal_node.x, goal_node.y);

    ////////
    // Method B: Straight path to nearest voronoi cell
    ////////
    // INTPOINT start_nearest_voro_cell, goal_nearest_voro_cell; 
    // dyn_voro_arr_[start_z_cm]->getNearestVoroCell(start_node_2d, start_nearest_voro_cell);
    // dyn_voro_arr_[goal_z_cm]->getNearestVoroCell(goal_node_2d, goal_nearest_voro_cell);

    came_from_vt_[start_node] = start_node;
    VCell start_node_3d(start_node_2d.x, start_node_2d.y, start_z_cm);
    g_cost_v_[start_node_3d] = 0;

    open_list_vt_.put(start_node, 0); // start_node has 0 f cost

    int num_iter = 0;
    std::vector<Eigen::Vector3i> neighbours; // 3d indices of neighbors

    while (!open_list_vt_.empty() && num_iter < astar_params_.max_iterations)
    {
        // if (num_iter%100 == 1){
        //     std::cout << "[a_star] Iteration " << num_iter << std::endl;
        //     publishClosedList(getClosedListVoroT(), closed_list_viz_pub_, "local_map_origin");
        //     // ros::Duration(0.1).sleep();
        // }

        VCell_T cur_node = open_list_vt_.get();

        VCell cur_node_3d = VCell(cur_node.x, cur_node.y, cur_node.z_cm);
        closed_list_vt_.insert(cur_node);

        if (cur_node.isSamePositionAs(goal_node))
        {
            std::cout << astar_params_.drone_id << ":  Found goal after " << num_iter << " iterations" << std::endl;
            // Goal reached, terminate search and obtain path
            tracePathVoroT(cur_node);

            return true;
        }

        IntPoint cur_node_2d(cur_node.x, cur_node.y);

        // Get neighbours that are within the map
        dyn_voro_arr_[cur_node.z_cm]->getVoroNeighbors(
            cur_node_2d, neighbours, marked_bubble_cells_[cur_node.z_cm], true);

        // Explore neighbors of current node. Each neighbor is (grid_x, grid_y, map_z_cm)
        for (const Eigen::Vector3i& nb_node_eig : neighbours) 
        {   
            int nb_t = cur_node.t + astar_params_.st_straight;
            
            VCell_T nb_node(nb_node_eig(0), nb_node_eig(1), nb_node_eig(2), nb_t);
            VCell nb_node_3d(nb_node_eig(0), nb_node_eig(1), nb_node_eig(2));

            // Convert to map coordinates so as to check reservation table
            Eigen::Vector4i nb_grid_4d{ nb_node_eig(0), 
                                        nb_node_eig(1), 
                                        nb_node_eig(2), nb_t};

            if (resrv_tbl_->find(nb_grid_4d) != resrv_tbl_->end() ){
                // Position has been reserved by another agent
                std::cout << "Drone " << astar_params_.drone_id <<  ": Grid Pos (" << nb_grid_4d.transpose() << ") has been reserved" << std::endl;
                continue;
            }

            double tent_g_cost = g_cost_v_[cur_node_3d] + cost_function(cur_node, nb_node);

            // If g_cost is not found or tentative cost is better than previously computed cost, then update costs
            if (g_cost_v_.find(nb_node_3d) == g_cost_v_.end() || tent_g_cost < g_cost_v_[nb_node_3d])
            {
                g_cost_v_[nb_node_3d] = tent_g_cost;
                // The tie_breaker is used to assign a larger weight to the h_cost and favour expanding nodes closer towards the goal
                // f_cost = g_cost (cost to come) + h_cost (cost to go)
                double f_cost = g_cost_v_[nb_node_3d] + astar_params_.tie_breaker * cost_function(nb_node, goal_node);

                // If not in closed list: set parent and add to open list
                if (closed_list_vt_.find(nb_node) == closed_list_vt_.end()) 
                {
                    came_from_vt_[nb_node] = cur_node;
                    open_list_vt_.put(nb_node, f_cost);
                }
                // No need to update parents for nodes already in closed list, paths leading up to current node is alr the most optimal
            }
        }
        num_iter++;
    }

    std::cerr   << "Drone " << astar_params_.drone_id << " :Unable to find goal node ("
                << goal_node.x << ", " << goal_node.y 
                << ") with maximum iteration " << num_iter << std::endl;

    return false;
}

