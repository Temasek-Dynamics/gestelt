#include <voronoi_planner/voronoi_planner.hpp>

void VoronoiPlanner::init(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{ 
  /* Publishers */

  // Occupancy map publishers
  occ_map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("voronoi_planner/occ_map", 10, true);
  voro_occ_grid_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("voronoi_planner/voro_map", 10, true);

  front_end_publisher_map_["front_end/closed_list"] = nh.advertise<visualization_msgs::Marker>("closed_list_viz", 10, true);

  front_end_plan_viz_pub_ = nh.advertise<visualization_msgs::Marker>("plan_viz", 5, true);
  start_pt_pub_ = nh.advertise<visualization_msgs::Marker>("start_point", 5, true);
  goal_pt_pub_ = nh.advertise<visualization_msgs::Marker>("goal_point", 5, true);

  voronoi_graph_pub_ = nh.advertise<visualization_msgs::Marker>("voronoi_graph", 5, true);

  /* Subscribers */
  plan_req_dbg_sub_ = nh.subscribe("/plan_request_dbg", 5, &VoronoiPlanner::startDebugCB, this);

  // start_sub_ = nh.subscribe("/start_dbg", 5, &VoronoiPlanner::startDebugCB, this);
  // goal_sub_ = nh.subscribe("/goal_dbg", 5, &VoronoiPlanner::goalDebugCB, this);
  bool_map_sub_ = nh.subscribe<gestelt_msgs::BoolMapArray>("bool_map_arr", 50, &VoronoiPlanner::boolMapCB, this);

  // Initialize map
  map_.reset(new GridMap);
  map_->initMapROS(nh, pnh);

  initParams(pnh);

  // pgmFileToBoolMap(&bool_map_, msg->width, msg->height, map_fname_);

  // // Set start and goal
  // DblPoint start_pos(0.5, 0.5);
  // DblPoint goal_pos(6.0, 6.0);

  astar_params_.max_iterations = 99999;
  astar_params_.debug_viz = true;
  astar_params_.tie_breaker = 1.01;
  // 0: getOctileDist, 1: getL1Norm, 2: getL2Norm, 3: getChebyshevDist
  astar_params_.cost_function_type  = 1;


  resrv_tbl_ = std::make_shared<std::unordered_set<VCell_T>>();

  if (use_test_map_){
    generateTestMap2();
  }

}

void VoronoiPlanner::initParams(ros::NodeHandle &pnh)
{
  pnh.param("map_filename", map_fname_, std::string(""));
  pnh.param("resolution", res_, 0.1);
  // pnh.param("negate", negate_, false);
  // pnh.param("occ_threshold", occ_th_, 100.0);
  // pnh.param("free_threshold", free_th_, 0.0);
  // pnh.param("yaw", yaw_, 0.0);
  
  pnh.param("verbose_planning", verbose_planning_, false);

  pnh.param("use_test_map", use_test_map_, false);

}

void VoronoiPlanner::realignBoolMap(bool ***map, bool ***map_og, int& size_x, int& size_y)
{
  for (int x=0; x<size_x; x++) {
    (*map)[x] = new bool[size_y];
  }

  for(int j = 0; j < size_y; j++)
  {
    for (int i = 0; i < size_x; i++)
    {
      (*map)[i][j] = (*map_og)[i][size_y-j-1];
    }
  }
}

/* SUbscriber callbacks*/

void VoronoiPlanner::boolMapCB(const gestelt_msgs::BoolMapArrayConstPtr& msg)
{ 
  if (use_test_map_){
    return;
  }

  tm_voro_map_init_.start();

  z_separation_cm_ = msg-> z_separation_cm;

  local_origin_x_ = msg->origin.x;
  local_origin_y_ = msg->origin.y;

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
      dyn_voro_params.resolution = res_;
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

  init_voro_maps_ = true; // Flag to indicate that all voronoi maps have been initialized

  publishVertices(voro_verts, "local_map_origin", voronoi_graph_pub_);
}

void VoronoiPlanner::startDebugCB(const gestelt_msgs::PlanRequestDebugConstPtr &msg)
{
  Eigen::Vector3d plan_start{ 
                  msg->start.position.x - local_origin_x_,
                  msg->start.position.y - local_origin_y_,
                  msg->start.position.z};

  Eigen::Vector3d plan_end{ 
                  msg->goal.position.x - local_origin_x_,
                  msg->goal.position.y - local_origin_y_,
                  msg->goal.position.z};

  plan(plan_start, plan_end);
}

/* Planning functions */

bool VoronoiPlanner::plan(const Eigen::Vector3d& start, const Eigen::Vector3d& goal){
  if (!init_voro_maps_){
    std::cout << "Voronoi maps not initialized! Request a plan after initialization!" << std::endl;
    return false;
  }

  // publishStartAndGoal(start, start_z, goal, goal_z, "local_map_origin", start_pt_pub_, goal_pt_pub_);

  // if (dyn_voro_arr_.find(goal_z_cm) == dyn_voro_arr_.end()){
  //   std::cout << "Map slice at height "<<   << " does not exist" << std::endl;
  //   return false;
  // }

  tm_front_end_plan_.start();

  front_end_planner_ = std::make_unique<AStarPlanner>(
    dyn_voro_arr_, z_separation_cm_, astar_params_, resrv_tbl_);
  front_end_planner_->addPublishers(front_end_publisher_map_);

  if (!front_end_planner_->generatePlanVoronoi(start, goal)){
    std::cout << "FRONT END FAILED!!!! front_end_planner_->generatePlanVoronoi() from ("<< start.transpose() << ") to (" << goal.transpose() << ")" << std::endl;

    tm_front_end_plan_.stop(verbose_planning_);

    front_end_planner_->publishClosedList(front_end_planner_->getClosedListVoronoi(), front_end_publisher_map_["front_end/closed_list"], "local_map_origin");
    return false;
  }
  else{
    front_end_path_lcl_ = front_end_planner_->getPathPosRaw();
    publishFrontEndPath(front_end_path_lcl_, "local_map_origin", front_end_plan_viz_pub_) ;
    front_end_planner_->publishClosedList(front_end_planner_->getClosedListVoronoi(), front_end_publisher_map_["front_end/closed_list"], "local_map_origin");
  }

  double min_clr = DBL_MAX; // minimum path clearance 
  double max_clr = 0.0;     // maximum path clearance 

  for (const Eigen::Vector3d& pos : front_end_path_lcl_ )
  {
    Eigen::Vector3d occ_nearest; 
    double dist_to_nearest_nb;
    if (map_->getNearestOccupiedCellLocal(pos, occ_nearest, dist_to_nearest_nb)){
      min_clr = (min_clr > dist_to_nearest_nb) ? dist_to_nearest_nb : min_clr;
      max_clr = (max_clr < dist_to_nearest_nb) ? dist_to_nearest_nb : max_clr;
    }
  }

  std::cout << "Maximum clearance of path: " << max_clr << std::endl;
  std::cout << "Minimum clearance of path: " << min_clr << std::endl;

  tm_front_end_plan_.stop(verbose_planning_);

  return true;
}

/* Test functions */

// Single layer map test
void VoronoiPlanner::generateTestMap2()
{
  // Create test boolean map
  z_separation_cm_ = 0.5;
  local_origin_x_ = 0.0;
  local_origin_y_ = 0.0;
  int num_rows = 7;
  int num_cols = 9;

  int z_cm = 0;
  int z_m = 0;
  double resolution = 0.1;

  std::vector<Eigen::Vector3d> voro_verts;
  std::vector<bool> bool_test_map(num_rows * num_cols, true); // all cells occupied by default

  /** [0,0] starts at bottom left
   *    0 xxxxxxxxx
   *    1 xxx000xxx
   *    2 xxx000xxx
   *    3 xxx000xxx
   *    4 x0000000x
   *    5 x0000000x
   *    6 x0000000x
   *    7 xxxxxxxxx
   *      012345678
   */

  // coordinates are (row, column)
  std::vector<std::pair<int, int>> occ_coords = {
    std::make_pair(1, 3),
    std::make_pair(1, 4),
    std::make_pair(1, 5),

    std::make_pair(2, 3),
    std::make_pair(2, 4),
    std::make_pair(2, 5),

    std::make_pair(3, 1),
    std::make_pair(3, 2),
    std::make_pair(3, 3),
    std::make_pair(3, 4),
    std::make_pair(3, 5),
    std::make_pair(3, 6),
    std::make_pair(3, 7),

    std::make_pair(4, 1),
    std::make_pair(4, 2),
    std::make_pair(4, 3),
    std::make_pair(4, 4),
    std::make_pair(4, 5),
    std::make_pair(4, 6),
    std::make_pair(4, 7),

    std::make_pair(5, 1),
    std::make_pair(5, 2),
    std::make_pair(5, 3),
    std::make_pair(5, 4),
    std::make_pair(5, 5),
    std::make_pair(5, 6),
    std::make_pair(5, 7)
  };

  for (const std::pair<int, int>& coord : occ_coords){
    // (col + row * num_cols)
    bool_test_map[coord.second + coord.first * num_cols] = false;
  }

  // Initialize bool map 2d vector if it does not exist
  if (bool_map_arr_.find(z_cm) == bool_map_arr_.end()){
    bool_map_arr_[z_cm] = std::make_shared<std::vector<std::vector<bool>>>(num_rows, std::vector<bool>(num_cols, false));
  }

  std::cout << "(*bool_map_arr_[z_cm]) rows = " << (*bool_map_arr_[z_cm]).size() << std::endl;
  std::cout << "(*bool_map_arr_[z_cm]) cols  = " << (*bool_map_arr_[z_cm])[0].size() << std::endl; 

  // set values of boolean map
  for(int i = 0; i < num_rows; i++)
  {
    for (int j = 0; j < num_cols; j++)
    {
      (*bool_map_arr_[z_cm])[i][j] = bool_test_map[j + i * num_cols];
    }
  }

  // set map boundaries as occupied
  // for(int j = 0; j < num_cols; j++)
  // {
  //   (*bool_map_arr_[z_cm])[0][j] = true;
  //   (*bool_map_arr_[z_cm])[num_rows-1][j] = true;
  // }
  // for (int i = 0; i < num_rows; i++)
  // {
  //   (*bool_map_arr_[z_cm])[i][0] = true;
  //   (*bool_map_arr_[z_cm])[i][num_cols-1] = true;
  // }

  // Create DynamicVoronoi object if it does not exist
  if (dyn_voro_arr_.find(z_cm) == dyn_voro_arr_.end()){

    DynamicVoronoi::DynamicVoronoiParams dyn_voro_params;
    dyn_voro_params.resolution = resolution;
    dyn_voro_params.origin_x = 0.0;
    dyn_voro_params.origin_y = 0.0;
    dyn_voro_params.origin_z = z_m;

    // Initialize dynamic voronoi 
    dyn_voro_arr_[z_cm] = std::make_shared<DynamicVoronoi>(dyn_voro_params);
    dyn_voro_arr_[z_cm]->initializeMap(num_rows, num_cols, bool_map_arr_[z_cm]);
  }
  
  dyn_voro_arr_[z_cm]->update(); // update distance map and Voronoi diagram
  dyn_voro_arr_[z_cm]->prune();  // prune the Voronoi
  // dyn_voro_arr_[z_cm]->updateAlternativePrunedDiagram();  

  // Get voronoi graph for current layer and append to voro_verts
  std::vector<Eigen::Vector3d> voro_verts_cur_layer = dyn_voro_arr_[z_cm]->getVoronoiVertices();
  voro_verts.insert(voro_verts.end(), voro_verts_cur_layer.begin(), voro_verts_cur_layer.end());

  // Publish map
  nav_msgs::OccupancyGrid occ_grid, voro_occ_grid;

  occmapToOccGrid(*dyn_voro_arr_[z_cm], 
                  0.0, 0.0, 
                  occ_grid); // Occupancy map

  voronoimapToOccGrid(*dyn_voro_arr_[z_cm], 
                      0.0, 0.0, 
                      voro_occ_grid); // Voronoi map

  voro_occ_grid_pub_.publish(voro_occ_grid);
  occ_map_pub_.publish(occ_grid);


  // Plan a path
  Eigen::Vector3i start{1, 3, 0};
  Eigen::Vector3i goal{7, 3, 0};
  
  front_end_planner_0_ = std::make_unique<AStarPlanner>(
    dyn_voro_arr_, z_separation_cm_, astar_params_, resrv_tbl_);
  front_end_planner_0_->addPublishers(front_end_publisher_map_);

  if (!front_end_planner_0_->generatePlanVoroT(start, goal)){
    std::cout << "FRONT END FAILED!!!! front_end_planner_->generatePlanVoronoi() from ("<< start.transpose() << ") to (" << goal.transpose() << ")" << std::endl;

    tm_front_end_plan_.stop(verbose_planning_);

    front_end_planner_0_->publishClosedList(front_end_planner_0_->getClosedListVoronoi(), front_end_publisher_map_["front_end/closed_list"], "local_map_origin");
    return false;
  }

  front_end_path_lcl_ = front_end_planner_0_->getPathPosRaw();
  publishFrontEndPath(front_end_path_lcl_, "local_map_origin", front_end_plan_viz_pub_) ;
  front_end_planner_0_->publishClosedList(front_end_planner_0_->getClosedListVoronoi(), front_end_publisher_map_["front_end/closed_list"], "local_map_origin");



}