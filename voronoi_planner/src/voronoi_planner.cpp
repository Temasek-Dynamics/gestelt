#include <voronoi_planner/voronoi_planner.hpp>

void VoronoiPlanner::init(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{ 
  // Params
  astar_params_.max_iterations = 9999;
  astar_params_.debug_viz = true;
  astar_params_.tie_breaker = 1.001;
  astar_params_.cost_function_type  = 1; // 0: getOctileDist, 1: getL1Norm, 2: getL2Norm, 3: getChebyshevDist

  num_agents_ = 5;

  resrv_tbl_ = std::make_shared<std::unordered_set<Eigen::Vector4d>>();

  /* Publishers */

  start_pt_pub_ = nh.advertise<visualization_msgs::Marker>("start_point", 5, true);
  goal_pt_pub_ = nh.advertise<visualization_msgs::Marker>("goal_point", 5, true);

  // Occupancy map publishers
  occ_map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("voronoi_planner/occ_map", 10, true);
  voro_occ_grid_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("voronoi_planner/voro_map", 10, true);

  for (int i = 0; i < num_agents_; i++){
    front_end_planners_.push_back(std::make_unique<AStarPlanner>(
                                    astar_params_, resrv_tbl_));

    closed_list_pubs_.push_back(nh.advertise<visualization_msgs::Marker>("closed_list_"+std::to_string(i), 500 , true));
    front_end_plan_pubs_.push_back(nh.advertise<visualization_msgs::Marker>("fe_plan_"+std::to_string(i), 500 , true));
  }

  voronoi_graph_pub_ = nh.advertise<visualization_msgs::Marker>("voronoi_graph", 500, true);

  /* Subscribers */
  plan_req_dbg_sub_ = nh.subscribe("/plan_request_dbg", 5, &VoronoiPlanner::startDebugCB, this);


  if (use_test_map_){
    generateTestMap2();
  }
  else{
    bool_map_sub_ = nh.subscribe<gestelt_msgs::BoolMapArray>("bool_map_arr", 50, &VoronoiPlanner::boolMapCB, this);
  }

  // Initialize map
  map_.reset(new GridMap);
  map_->initMapROS(nh, pnh);

  initParams(pnh);
}

void VoronoiPlanner::initParams(ros::NodeHandle &pnh)
{
  // pnh.param("map_filename", map_fname_, std::string(""));
  // pnh.param("resolution", res_, 0.1);
  // pnh.param("negate", negate_, false);
  // pnh.param("occ_threshold", occ_th_, 100.0);
  // pnh.param("free_threshold", free_th_, 0.0);
  // pnh.param("yaw", yaw_, 0.0);
  
  pnh.param("verbose_planning", verbose_planning_, false);
  pnh.param("use_test_map", use_test_map_, false);
}


/* SUbscriber callbacks*/

void VoronoiPlanner::boolMapCB(const gestelt_msgs::BoolMapArrayConstPtr& msg)
{ 

  tm_voro_map_init_.start();

  local_origin_x_ = msg->origin.x;
  local_origin_y_ = msg->origin.y;
  res_ = msg->resolution;
  z_separation_cm_ = msg-> z_separation_cm;

  std::vector<Eigen::Vector3d> voro_verts;

  for (int z_cm = 0; z_cm < msg->max_height_cm; z_cm += msg->z_separation_cm){
    
    gestelt_msgs::BoolMap bool_map_msg;
    for (const gestelt_msgs::BoolMap& bool_map_msg_sel : msg->bool_maps){
      if (bool_map_msg_sel.z_cm == z_cm){
        bool_map_msg = bool_map_msg_sel;
        break;
      }
    }

    if (bool_map_msg.map.size() == 0){
      std::cout << "ERROR: MAP LAYER DOES NOT EXIST EVEN THOUGH IT IS EXPECTED AT " << z_cm << " cm!" << std::endl;
    }

    // Initialize bool map 2d vector if it does not exist
    if (bool_map_arr_.find(z_cm) == bool_map_arr_.end()){
      bool_map_arr_[z_cm] = std::make_shared<std::vector<std::vector<bool>>>(msg->height, std::vector<bool>(msg->width, false));
    }

    // set values of boolean map
    for(int j = 0; j < msg->height; j++)
    {
      for (int i = 0; i < msg->width; i++) 
      {
        (*bool_map_arr_[z_cm])[i][j] = bool_map_msg.map[i + j * msg->width];
      }
    }

    // set map boundaries as occupied
    for(int j = 0; j < msg->height; j++)
    {
      (*bool_map_arr_[z_cm])[0][j] = true;
      (*bool_map_arr_[z_cm])[msg->width-1][j] = true;
    }
    for (int i = 0; i < msg->width; i++)
    {
      (*bool_map_arr_[z_cm])[i][0] = true;
      (*bool_map_arr_[z_cm])[i][msg->height-1] = true;
    }


    // Create DynamicVoronoi object if it does not exist
    if (dyn_voro_arr_.find(z_cm) == dyn_voro_arr_.end()){

      DynamicVoronoi::DynamicVoronoiParams dyn_voro_params;
      dyn_voro_params.resolution = msg->resolution;
      dyn_voro_params.origin_x = 0.0;
      dyn_voro_params.origin_y = 0.0;
      dyn_voro_params.origin_z = bool_map_msg.z;

      // Initialize dynamic voronoi 
      dyn_voro_arr_[z_cm] = std::make_shared<DynamicVoronoi>(dyn_voro_params);
      dyn_voro_arr_[z_cm]->initializeMap(msg->width, msg->height, bool_map_arr_[z_cm]);
    }
    
    dyn_voro_arr_[z_cm]->update(); // update distance map and Voronoi diagram
    dyn_voro_arr_[z_cm]->prune();  // prune the Voronoi
    // dyn_voro_arr_[z_cm]->updateAlternativePrunedDiagram();  

    // Get voronoi graph for current layer and append to voro_verts
    std::vector<Eigen::Vector3d> voro_verts_cur_layer = dyn_voro_arr_[z_cm]->getVoronoiVertices();
    voro_verts.insert(voro_verts.end(), voro_verts_cur_layer.begin(), voro_verts_cur_layer.end());

    nav_msgs::OccupancyGrid occ_grid, voro_occ_grid;

    occmapToOccGrid(*dyn_voro_arr_[z_cm], 
                    msg->origin.x, msg->origin.y, 
                    occ_grid); // Occupancy map

    voronoimapToOccGrid(*dyn_voro_arr_[z_cm], 
                        msg->origin.x, msg->origin.y, 
                        voro_occ_grid); // Voronoi map

    voro_occ_grid_pub_.publish(voro_occ_grid);
    occ_map_pub_.publish(occ_grid);
  }

  // Link all voronoi layers together
  for (int z_cm = 0; z_cm < msg->max_height_cm; z_cm += msg->z_separation_cm){
    if (z_cm == 0){
      // Link to top voronoi layer for bottom layer
      dyn_voro_arr_[z_cm]->top_voro_ = dyn_voro_arr_[msg->z_separation_cm];
    }
    else if (z_cm == msg->max_height_cm){
      // Link to bottom voronoi layer for top layer
      dyn_voro_arr_[z_cm]->bottom_voro_ = dyn_voro_arr_[msg->max_height_cm - msg->z_separation_cm];
    }
    else {
      dyn_voro_arr_[z_cm]->bottom_voro_ = dyn_voro_arr_[z_cm - msg->z_separation_cm];
      dyn_voro_arr_[z_cm]->top_voro_ = dyn_voro_arr_[z_cm + msg->z_separation_cm];
    }
  }

  tm_voro_map_init_.stop(false);

  publishVertices(voro_verts, "local_map_origin", voronoi_graph_pub_);

  for (int i = 0; i < num_agents_; i++)
  {
    // Assign voronoi map
    front_end_planners_[i]->assignVoroMap(dyn_voro_arr_, msg->z_separation_cm);
  }

  init_voro_maps_ = true; // Flag to indicate that all voronoi maps have been initialized

  // if (plan_once_){
  //   Eigen::Vector3d plan_start{ 
  //                   -7.0 - local_origin_x_,
  //                   -7.0 - local_origin_y_,
  //                   2.2};

  //   Eigen::Vector3d plan_end{ 
  //                   7.0 - local_origin_x_,
  //                   7.0 - local_origin_y_,
  //                   1.0};

  //   std::cout << "Agent " << 0 << ": plan request from ("<< 
  //                 plan_start.transpose() << ") to (" << plan_end.transpose() << ")" << std::endl;

  //   plan(0, plan_start, plan_end);
  //   plan_once_ = false;
  // }

}

void VoronoiPlanner::startDebugCB(const gestelt_msgs::PlanRequestDebugConstPtr &msg)
{
  if (msg->agent_id > num_agents_){
    std::cout << "Requested plan for agent ID " << msg->agent_id << " exceeds maximum agent num of " << num_agents_ << std::endl;
  }

  Eigen::Vector3d plan_start{ 
                  msg->start.position.x - local_origin_x_,
                  msg->start.position.y - local_origin_y_,
                  msg->start.position.z};

  Eigen::Vector3d plan_end{ 
                  msg->goal.position.x - local_origin_x_,
                  msg->goal.position.y - local_origin_y_,
                  msg->goal.position.z};

  std::cout << "Agent " << msg->agent_id << ": plan request from ("<< 
                plan_start.transpose() << ") to (" << plan_end.transpose() << ")" << std::endl;

  plan(msg->agent_id, plan_start, plan_end);
}

/* Planning functions */

// bool VoronoiPlanner::plan(const Eigen::Vector3d& start, const Eigen::Vector3d& goal){
//   if (!init_voro_maps_){
//     std::cout << "Voronoi maps not initialized! Request a plan after initialization!" << std::endl;
//     return false;
//   }

//   // publishStartAndGoal(start, start_z, goal, goal_z, "local_map_origin", start_pt_pub_, goal_pt_pub_);

//   // if (dyn_voro_arr_.find(goal_z_cm) == dyn_voro_arr_.end()){
//   //   std::cout << "Map slice at height "<<   << " does not exist" << std::endl;
//   //   return false;
//   // }

//   tm_front_end_plan_.start();

//   front_end_planner_ = std::make_unique<AStarPlanner>(
//     dyn_voro_arr_, z_separation_cm_, astar_params_, resrv_tbl_);
//   front_end_planner_->addPublishers(front_end_publisher_map_);

//   if (!front_end_planner_->generatePlanVoronoi(start, goal)){
//     std::cout << "FRONT END FAILED!!!! front_end_planner_->generatePlanVoronoi() from ("<< start.transpose() << ") to (" << goal.transpose() << ")" << std::endl;

//     tm_front_end_plan_.stop(verbose_planning_);

//     front_end_planner_->publishClosedList(front_end_planner_->getClosedListVoronoi(), front_end_publisher_map_["front_end/closed_list"], "local_map_origin");
//     return false;
//   }
//   else{
//     front_end_path_lcl_ = front_end_planner_->getPathPosRaw();
//     publishFrontEndPath(front_end_path_lcl_, "local_map_origin", front_end_plan_viz_pub_) ;
//     front_end_planner_->publishClosedList(front_end_planner_->getClosedListVoronoi(), front_end_publisher_map_["front_end/closed_list"], "local_map_origin");
//   }



//   tm_front_end_plan_.stop(verbose_planning_);

//   return true;
// }


bool VoronoiPlanner::plan(const int& id, const Eigen::Vector3d& start, const Eigen::Vector3d& goal){
  if (!init_voro_maps_){
    std::cout << "Voronoi maps not initialized! Request a plan after initialization!" << std::endl;
    return false;
  }
  publishStartAndGoal(start, goal,"local_map_origin", start_pt_pub_, goal_pt_pub_);

  tm_front_end_plan_.start();

  // Generate plan 
  if (!front_end_planners_[id]->generatePlanVoroT(start, goal)){
    std::cout << "Agent " << id << " FRONT END FAILED!!!! front_end_planner_->generatePlanVoroT() from ("
              << start.transpose() << ") to (" << goal.transpose() << ")" << std::endl;
    tm_front_end_plan_.stop(verbose_planning_);

    publishClosedList(front_end_planners_[id]->getClosedListVoroT(), 
                      closed_list_pubs_[id], "local_map_origin");

    std::cout << "front_end_planners_[id]->getClosedListVoroT() size: " << 
                  front_end_planners_[id]->getClosedListVoroT().size() << std::endl;
    return false;
  }

  tm_front_end_plan_.stop(verbose_planning_);

  // Retrieve space time path and publish it
  space_time_path_[id] = front_end_planners_[id]->getSpaceTimePath();
  publishClosedList(front_end_planners_[id]->getClosedListVoroT(), closed_list_pubs_[id], "local_map_origin");

  // publishSpaceTimePath(space_time_path_[id], "local_map_origin", front_end_plan_pubs_[id]) ;

  // double min_clr = DBL_MAX; // minimum path clearance 
  // double max_clr = 0.0;     // maximum path clearance 

  // for (const Eigen::Vector4d& pos_4d : space_time_path_[id] )
  // {
  //   Eigen::Vector3d pos{pos_4d(0), pos_4d(1), pos_4d(2)}; 
  //   Eigen::Vector3d occ_nearest; 
  //   double dist_to_nearest_nb;
  //   if (map_->getNearestOccupiedCellLocal(pos, occ_nearest, dist_to_nearest_nb)){
  //     min_clr = (min_clr > dist_to_nearest_nb) ? dist_to_nearest_nb : min_clr;
  //     max_clr = (max_clr < dist_to_nearest_nb) ? dist_to_nearest_nb : max_clr;
  //   }
  // }
  // std::cout << "Maximum clearance: " << max_clr << ", Minimum clearance: " << min_clr << std::endl;


  return true;
}


/* Test functions */

// Single layer map test
void VoronoiPlanner::generateTestMap2()
{
  // // Create test boolean map
  // local_origin_x_ = 0.0;
  // local_origin_y_ = 0.0;
  // int num_rows = 10;
  // int num_cols = 13;

  // int z_cm = 0;
  // int z_m = 0;
  // res_ = 0.1;
  // z_separation_cm_ = 50;

  // std::vector<Eigen::Vector3d> voro_verts;
  // std::vector<bool> bool_test_map(num_rows * num_cols, true); // all cells occupied by default

  // /** [0,0] starts at top left
  //  *    0 xxxxxxxxxxxxx
  //  *    1 xxxx00000xxxx
  //  *    2 xxxx00300xxxx
  //  *    3 xxxx00000xxxx
  //  *    4 x00000000000x
  //  *    5 x00000000000x
  //  *    6 x01000000020x
  //  *    7 x00000000000x
  //  *    8 x00000000000x
  //  *    9 xxxxxxxxxxxxx
  //  *      0123456789012
  //  */
  // // 10 rows, 13 columns

  // // coordinates are (row, column)
  // std::vector<std::pair<int, int>> occ_coords = {
  //   std::make_pair(1, 4),
  //   std::make_pair(1, 5),
  //   std::make_pair(1, 6),
  //   std::make_pair(1, 7),
  //   std::make_pair(1, 8),

  //   std::make_pair(2, 4),
  //   std::make_pair(2, 5),
  //   std::make_pair(2, 6),
  //   std::make_pair(2, 7),
  //   std::make_pair(2, 8),

  //   std::make_pair(3, 4),
  //   std::make_pair(3, 5),
  //   std::make_pair(3, 6),
  //   std::make_pair(3, 7),
  //   std::make_pair(3, 8),

  //   std::make_pair(4, 1),
  //   std::make_pair(4, 2),
  //   std::make_pair(4, 3),
  //   std::make_pair(4, 4),
  //   std::make_pair(4, 5),
  //   std::make_pair(4, 6),
  //   std::make_pair(4, 7),
  //   std::make_pair(4, 8),
  //   std::make_pair(4, 9),
  //   std::make_pair(4, 10),
  //   std::make_pair(4, 11),

  //   std::make_pair(5, 1),
  //   std::make_pair(5, 2),
  //   std::make_pair(5, 3),
  //   std::make_pair(5, 4),
  //   std::make_pair(5, 5),
  //   std::make_pair(5, 6),
  //   std::make_pair(5, 7),
  //   std::make_pair(5, 8),
  //   std::make_pair(5, 9),
  //   std::make_pair(5, 10),
  //   std::make_pair(5, 11),

  //   std::make_pair(6, 1),
  //   std::make_pair(6, 2),
  //   std::make_pair(6, 3),
  //   std::make_pair(6, 4),
  //   std::make_pair(6, 5),
  //   std::make_pair(6, 6),
  //   std::make_pair(6, 7),
  //   std::make_pair(6, 8),
  //   std::make_pair(6, 9),
  //   std::make_pair(6, 10),
  //   std::make_pair(6, 11),

  //   std::make_pair(7, 1),
  //   std::make_pair(7, 2),
  //   std::make_pair(7, 3),
  //   std::make_pair(7, 4),
  //   std::make_pair(7, 5),
  //   std::make_pair(7, 6),
  //   std::make_pair(7, 7),
  //   std::make_pair(7, 8),
  //   std::make_pair(7, 9),
  //   std::make_pair(7, 10),
  //   std::make_pair(7, 11),

  //   std::make_pair(8, 1),
  //   std::make_pair(8, 2),
  //   std::make_pair(8, 3),
  //   std::make_pair(8, 4),
  //   std::make_pair(8, 5),
  //   std::make_pair(8, 6),
  //   std::make_pair(8, 7),
  //   std::make_pair(8, 8),
  //   std::make_pair(8, 9),
  //   std::make_pair(8, 10),
  //   std::make_pair(8, 11),
  // };

  // for (const std::pair<int, int>& coord : occ_coords){
  //   // (col + row * num_cols)
  //   bool_test_map[coord.second + coord.first * num_cols] = false;
  // }

  // // Initialize bool map 2d vector if it does not exist
  // if (bool_map_arr_.find(z_cm) == bool_map_arr_.end()){
  //   bool_map_arr_[z_cm] = std::make_shared<std::vector<std::vector<bool>>>(num_rows, std::vector<bool>(num_cols, false));
  // }

  // std::cout << "(*bool_map_arr_[z_cm]) rows = " << (*bool_map_arr_[z_cm]).size() << std::endl;
  // std::cout << "(*bool_map_arr_[z_cm]) cols  = " << (*bool_map_arr_[z_cm])[0].size() << std::endl; 

  // // set values of boolean map
  // for(int i = 0; i < num_rows; i++)
  // {
  //   for (int j = 0; j < num_cols; j++)
  //   {
  //     (*bool_map_arr_[z_cm])[i][j] = bool_test_map[j + i * num_cols];
  //   }
  // }

  // // set map boundaries as occupied
  // // for(int j = 0; j < num_cols; j++)
  // // {
  // //   (*bool_map_arr_[z_cm])[0][j] = true;
  // //   (*bool_map_arr_[z_cm])[num_rows-1][j] = true;
  // // }
  // // for (int i = 0; i < num_rows; i++)
  // // {
  // //   (*bool_map_arr_[z_cm])[i][0] = true;
  // //   (*bool_map_arr_[z_cm])[i][num_cols-1] = true;
  // // }

  // // Create DynamicVoronoi object if it does not exist
  // if (dyn_voro_arr_.find(z_cm) == dyn_voro_arr_.end()){

  //   DynamicVoronoi::DynamicVoronoiParams dyn_voro_params;
  //   dyn_voro_params.resolution = res_;
  //   dyn_voro_params.origin_x = 0.0;
  //   dyn_voro_params.origin_y = 0.0;
  //   dyn_voro_params.origin_z = z_m;

  //   // Initialize dynamic voronoi 
  //   dyn_voro_arr_[z_cm] = std::make_shared<DynamicVoronoi>(dyn_voro_params);
  //   dyn_voro_arr_[z_cm]->initializeMap(num_rows, num_cols, bool_map_arr_[z_cm]);
  // }
  
  // dyn_voro_arr_[z_cm]->update(); // update distance map and Voronoi diagram
  // dyn_voro_arr_[z_cm]->prune();  // prune the Voronoi
  // // dyn_voro_arr_[z_cm]->updateAlternativePrunedDiagram();  

  // // Get voronoi graph for current layer and append to voro_verts
  // std::vector<Eigen::Vector3d> voro_verts_cur_layer = dyn_voro_arr_[z_cm]->getVoronoiVertices();
  // voro_verts.insert(voro_verts.end(), voro_verts_cur_layer.begin(), voro_verts_cur_layer.end());

  // // Publish map
  // nav_msgs::OccupancyGrid occ_grid, voro_occ_grid;

  // occmapToOccGrid(*dyn_voro_arr_[z_cm], 
  //                 -res_/2, -res_/2, 
  //                 occ_grid); // Occupancy map

  // voronoimapToOccGrid(*dyn_voro_arr_[z_cm], 
  //                     -res_/2, -res_/2, 
  //                     voro_occ_grid); // Voronoi map

  // voro_occ_grid_pub_.publish(voro_occ_grid);
  // occ_map_pub_.publish(occ_grid);

  // publishVertices(voro_verts, "world", voronoi_graph_pub_);

  // // Plan a path
  // Eigen::Vector3i start_0{6, 2, 0};
  // Eigen::Vector3i goal_0{6, 10, 0};

  // DblPoint start_2d_map_pos, goal_2d_map_pos;
  // dyn_voro_arr_[z_cm]->idxToPos(IntPoint(start_0(0), start_0(1)), start_2d_map_pos);
  // dyn_voro_arr_[z_cm]->idxToPos(IntPoint(goal_0(0), goal_0(1)), goal_2d_map_pos);

  // Eigen::Vector3d start_3d_map_pos{start_2d_map_pos.x, start_2d_map_pos.y, z_cm};
  // Eigen::Vector3d goal_3d_map_pos{goal_2d_map_pos.x, goal_2d_map_pos.y, z_cm};

  // publishStartAndGoal(start_3d_map_pos, goal_3d_map_pos, 
  //                     "world", start_pt_pub_, goal_pt_pub_);

  // std::cout << "Map Positions: Planning From " << start_3d_map_pos.transpose()
  //           << " to "  << goal_3d_map_pos.transpose() << std::endl;
  // std::cout << "Grid position: Planning from " << start_0.transpose() << " to " <<  goal_0.transpose() << std::endl;


  // // Assign voronoi map
  // front_end_planners_[0]->assignVoroMap(dyn_voro_arr_, z_separation_cm_);
  // front_end_planners_[1]->assignVoroMap(dyn_voro_arr_, z_separation_cm_);

  // Eigen::Vector3i start_1{3, 6, 0};
  // Eigen::Vector3i goal_1{6, 3, 0};

  // tm_front_end_plan_.start();

  // // Generate plans
  // if (!front_end_planners_[0]->generatePlanVoroT(start_0, goal_0)){
  //   std::cout << "Planner 0: FRONT END FAILED!!!! front_end_planner_->generatePlanVoronoi() from ("
  //             << start_0.transpose() << ") to (" << goal_0.transpose() << ")" << std::endl;
  //   tm_front_end_plan_.stop(verbose_planning_);

  //   publishClosedList(front_end_planners_[0]->getClosedListVoroT(), 
  //                     closed_list_pubs_[0], "world");
  //   return ;
  // }


  // if (!front_end_planners_[1]->generatePlanVoroT(start_1, goal_1)){
  //   std::cout << "Planner 0: FRONT END FAILED!!!! front_end_planner_->generatePlanVoronoi() from ("
  //             << start_1.transpose() << ") to (" << goal_1.transpose() << ")" << std::endl;
  //   tm_front_end_plan_.stop(verbose_planning_);

  //   publishClosedList(front_end_planners_[1]->getClosedListVoroT(), 
  //                     closed_list_pubs_[1], "world");
  //   return ;
  // }

  // tm_front_end_plan_.stop(verbose_planning_);

  // space_time_path_[0] = front_end_planners_[0]->getSpaceTimePath();
  // publishClosedList(front_end_planners_[0]->getClosedListVoroT(), closed_list_pubs_[0], "world");

  // space_time_path_[1] = front_end_planners_[1]->getSpaceTimePath();
  // publishClosedList(front_end_planners_[1]->getClosedListVoroT(), closed_list_pubs_[1], "world");

  // publishSpaceTimePath(space_time_path_[0], "world", front_end_plan_pubs_[0]) ;
  // publishSpaceTimePath(space_time_path_[1], "world", front_end_plan_pubs_[1]) ;

  // // Print paths
  // std::cout << "Agent 0 path: " << std::endl;
  // for (size_t i = 0; i < space_time_path_[0].size(); i++){
  //   std::cout << space_time_path_[0][i].transpose() << std::endl;
  // }
  // std::cout << "Agent 1 path: " << std::endl;
  // for (size_t i = 0; i < space_time_path_[1].size(); i++){
  //   std::cout << space_time_path_[1][i].transpose() << std::endl;
  // }

  return ;
}