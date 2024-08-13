#include <voronoi_planner/voronoi_planner.hpp>

void VoronoiPlanner::init(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{ 
  // Initialize Params
  initParams(pnh);

  /* Publishers */
  start_pt_pub_ = nh.advertise<visualization_msgs::Marker>("fe_start", 5, true);
  goal_pt_pub_ = nh.advertise<visualization_msgs::Marker>("fe_goal", 5, true);

  // Map publishers
  occ_map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("voro/occ_map", 10, true);
  voro_occ_grid_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("voro/voro_map", 10, true);
  voronoi_graph_pub_ = nh.advertise<visualization_msgs::Marker>("voronoi_graph", 500, true);

  // Planner publishers
  fe_closed_list_pub_ = nh.advertise<visualization_msgs::Marker>("fe_plan/closed_list", 50, true);
  fe_plan_viz_pub_ = nh.advertise<visualization_msgs::Marker>("fe_plan/viz", 50 , true);
  fe_plan_pub_ = nh.advertise<gestelt_msgs::FrontEndPlan>("fe_plan", 5, true);
  fe_plan_broadcast_pub_ = nh.advertise<gestelt_msgs::FrontEndPlan>("/fe_plan/broadcast", 5, true);

  /* Subscribers */
  fe_plan_broadcast_sub_ = nh.subscribe<gestelt_msgs::FrontEndPlan>("/fe_plan/broadcast", 50, &VoronoiPlanner::FEPlanSubCB, this);

  plan_req_dbg_sub_ = nh.subscribe("plan_request_dbg", 50, 
                                  &VoronoiPlanner::planReqDbgCB, this);
  goals_sub_ = nh.subscribe("goals", 50, &VoronoiPlanner::goalsCB, this);
  bool_map_sub_ = nh.subscribe<gestelt_msgs::BoolMapArray>("bool_map_arr", 50, 
                                                            &VoronoiPlanner::boolMapCB, this);

  odom_sub_ = nh.subscribe("odom", 5, &VoronoiPlanner::odometryCB, this);

  /* Timers */
  plan_fe_timer_ = nh.createTimer(ros::Duration(1.0/fe_planner_freq_), 
                                  &VoronoiPlanner::planFETimerCB, this);

  /* Initialization */

  // Initialize planner
  resrv_tbl_ = std::make_shared<std::unordered_set<Eigen::Vector4i>>();

  astar_params_.drone_id = drone_id_;
  astar_params_.max_iterations = 9999;
  astar_params_.debug_viz = true;
  astar_params_.tie_breaker = 1.001;
  astar_params_.cost_function_type  = 1; // 0: getOctileDist, 1: getL1Norm, 2: getL2Norm, 3: getChebyshevDist
  astar_params_.t_unit = t_unit_;
  fe_planner_ = std::make_unique<AStarPlanner>(astar_params_, resrv_tbl_);

  // Initialize map
  map_.reset(new GridMap);
  map_->initMapROS(nh, pnh);
  
}

void VoronoiPlanner::initParams(ros::NodeHandle &pnh)
{
  pnh.param("drone_id", drone_id_, -1);
  pnh.param("num_agents", num_agents_, 1);

  pnh.param("verbose_planning", verbose_planning_, false);
  pnh.param("plan_once", plan_once_, false);
  pnh.param("front_end_planner_frequency", fe_planner_freq_, 10.0);

  pnh.param("local_map_origin", local_map_origin_, std::string("local_map_origin"));
  pnh.param("global_origin", global_origin_, std::string("world"));

  pnh.param("critical_clearance", critical_clr_, 0.25);
  pnh.param("t_unit", t_unit_, 0.1);
}

/* Timer callbacks*/
void VoronoiPlanner::planFETimerCB(const ros::TimerEvent &e)
{
  // Check if waypoint queue is empty
  if (waypoints_.empty()){
    return;
  }

  if (isGoalReached(cur_pos_, waypoints_.nextWP())){
    // If goals is within a given tolerance, then pop this goal and plan next goal (if available)
    waypoints_.popWP();

    return;
  }

  // Plan from current position to next waypoint
  plan(cur_pos_, waypoints_.nextWP());
}

/* Subscriber callbacks*/

void VoronoiPlanner::FEPlanSubCB(const gestelt_msgs::FrontEndPlanConstPtr& msg)
{
  if (!init_voro_maps_){
    return;
  }

  // PRIORITY-BASED PLANNING: Only consider trajectories of drones with higher agent_id
  if (msg->agent_id <= drone_id_){
    return;
  }

  // Add all points on path (with inflation) to reservation table
  double inflation = 0.3; // [m]
  int num_cells_inf = inflation/res_; // Number of cells used for inflation

  double t_now = ros::Time::now().toSec();

  // Round to nearest units of 0.1
  // st_units_elapsed_plan_start: space time units since plan started 
  int st_units_elapsed_plan_start =  tToSpaceTimeUnits(t_now) - tToSpaceTimeUnits(msg->plan_start_time);
  logInfo(str_fmt("st_units_elapsed_plan_start(%d)", st_units_elapsed_plan_start));
  // prev_t: Relative time of last points
  int prev_t = 0;
  for (size_t i = 0; i < msg->plan.size(); i++){
    IntPoint grid_pos;
    // get map position relative to local origin
    DblPoint map_2d_pos(msg->plan[i].position.x - local_origin_x_, 
                        msg->plan[i].position.y - local_origin_y_);
    int map_z_cm =roundToMultInt((int) (msg->plan[i].position.z * 100), z_separation_cm_);
    
    dyn_voro_arr_[map_z_cm]->posToIdx(map_2d_pos, grid_pos);

    // Inflate the cells by the given inflation radius
    for(int x = grid_pos.x - num_cells_inf; x <= grid_pos.x + num_cells_inf; x++)
    {
      for(int y = grid_pos.y - num_cells_inf; y <= grid_pos.y + num_cells_inf; y++)
      {
        // Add position for the entire time interval from previous t to current t
        for (int j = 0; j < msg->plan_time[i] - prev_t; j++) { 
          if (st_units_elapsed_plan_start + prev_t + j < 0){
            continue;
          }
          resrv_tbl_->insert(Eigen::Vector4i{x, y, map_z_cm, st_units_elapsed_plan_start + prev_t + j});
        }
      }
    }

    prev_t = msg->plan_time[i]; 
  } 

  IntPoint grid_pos;
  // get map position relative to local origin
  DblPoint map_2d_pos(msg->plan.back().position.x - local_origin_x_, 
                      msg->plan.back().position.y - local_origin_y_);
  int map_z_cm =roundToMultInt((int) (msg->plan.back().position.z * 100), z_separation_cm_);
  dyn_voro_arr_[map_z_cm]->posToIdx(map_2d_pos, grid_pos);
  resrv_tbl_->insert(Eigen::Vector4i{grid_pos.x, grid_pos.y, map_z_cm, st_units_elapsed_plan_start + msg->plan_time.back()});
}

void VoronoiPlanner::boolMapCB(const gestelt_msgs::BoolMapArrayConstPtr& msg)
{ 
  tm_voro_map_init_.start();

  local_origin_x_ = msg->origin.x;
  local_origin_y_ = msg->origin.y;
  max_height_ = msg->max_height_cm;

  res_ = msg->resolution;
  z_separation_cm_ = msg-> z_separation_cm;

  std::vector<Eigen::Vector3d> voro_verts;

  for (int z_cm = 0; z_cm <= msg->max_height_cm; z_cm += msg->z_separation_cm){
    
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

  viz_helper::publishVertices(voro_verts, local_map_origin_, voronoi_graph_pub_);

  // Assign voronoi map
  fe_planner_->assignVoroMap(dyn_voro_arr_, msg->z_separation_cm, 
                              local_origin_x_, local_origin_y_,
                              msg->max_height_cm);

  init_voro_maps_ = true; // Flag to indicate that all voronoi maps have been initialized
}

void VoronoiPlanner::goalsCB(const gestelt_msgs::GoalsConstPtr &msg)
{
    if (msg->transforms.size() <= 0)
    {
      logError("Received empty waypoints");
      return;
    }
    if (msg->header.frame_id != "world" && msg->header.frame_id != "map" )
    {
      logError("Only waypoint goals in 'world' or 'map' frame are accepted, ignoring waypoints.");
      return;
    }

    std::vector<Eigen::Vector3d> wp_vec;

    for (auto& pos : msg->transforms) {
      // Transform received waypoints from world to UAV origin frame
      wp_vec.push_back(Eigen::Vector3d{
                        pos.translation.x - local_origin_x_, 
                        pos.translation.y - local_origin_y_, 
                        pos.translation.z});
    }

    waypoints_.addMultipleWP(wp_vec);
}

void VoronoiPlanner::planReqDbgCB(const gestelt_msgs::PlanRequestDebugConstPtr &msg)
{
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

  plan(plan_start, plan_end);
}

void VoronoiPlanner::odometryCB(const nav_msgs::OdometryConstPtr &msg)
{
  cur_pos_= Eigen::Vector3d{msg->pose.pose.position.x - local_origin_x_, 
                            msg->pose.pose.position.y - local_origin_y_, 
                            msg->pose.pose.position.z};
  // cur_vel_= Eigen::Vector3d{msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z};
}

/* Planning functions */

bool VoronoiPlanner::plan(const Eigen::Vector3d& start, const Eigen::Vector3d& goal){
  if (!init_voro_maps_){
    std::cout << "Voronoi maps not initialized! Request a plan after initialization!" << std::endl;
    return false;
  }
  
  viz_helper::publishStartAndGoal(start, goal, local_map_origin_, start_pt_pub_, goal_pt_pub_);

  // tm_front_end_plan_.start();

  // Generate plan 
  if (!fe_planner_->generatePlanVoroT(start, goal)){
    logError(str_fmt("Front end failed to generate plan from (%f, %f, %f) to (%f, %f, %f)",
      start(0), start(1), start(2),
      goal(0), goal(1), goal(2)));

    // tm_front_end_plan_.stop(verbose_planning_);

    viz_helper::publishClosedList(fe_planner_->getClosedListVoroT(), 
                      fe_closed_list_pub_, local_map_origin_);

    logError(str_fmt("Closed list size: %d",
                      fe_planner_->getClosedListVoroT().size() ));

    return false;
  }

  // tm_front_end_plan_.stop(verbose_planning_);

  // Retrieve space time path and publish it
  front_end_path_ = fe_planner_->getPath();
  space_time_path_ = fe_planner_->getSpaceTimePath();
  viz_helper::publishClosedList(fe_planner_->getClosedListVoroT(), fe_closed_list_pub_, local_map_origin_);

  // Convert from space time path to gestelt_msgs::FrontEndPlan
  gestelt_msgs::FrontEndPlan fe_plan_msg;

  fe_plan_msg.agent_id = drone_id_;
  fe_plan_msg.header.stamp = ros::Time::now();

  for (size_t i = 0; i < space_time_path_.size(); i++){
    geometry_msgs::Pose pose;
    pose.position.x = space_time_path_[i](0);
    pose.position.y = space_time_path_[i](1);
    pose.position.z = space_time_path_[i](2);
    pose.orientation.w = 1.0; 

    fe_plan_msg.plan.push_back(pose);
    fe_plan_msg.plan_time.push_back(int(space_time_path_[i](3)));
  }

  fe_plan_msg.plan_start_time = ros::Time::now().toSec();

  fe_plan_pub_.publish(fe_plan_msg);
  fe_plan_broadcast_pub_.publish(fe_plan_msg);

  // viz_helper::publishSpaceTimePath(space_time_path_, global_origin_, fe_plan_viz_pub_) ;

  viz_helper::publishFrontEndPath(front_end_path_, global_origin_, fe_plan_viz_pub_) ;

  // double min_clr = DBL_MAX; // minimum path clearance 
  // double max_clr = 0.0;     // maximum path clearance 

  // for (const Eigen::Vector4d& pos_4d : space_time_path_)
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

/**
 * @brief Check if current position is within goal tolerance
 * 
 * @return true 
 * @return false 
 */
bool VoronoiPlanner::isGoalReached(const Eigen::Vector3d& pos, const Eigen::Vector3d& goal)
{
  return (pos - goal).squaredNorm() < squared_goal_tol_;
}