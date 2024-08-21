#include <voronoi_planner/voronoi_planner.hpp>

void VoronoiPlanner::init(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{ 
  // Initialize Params
  initParams(pnh);

  tm_front_end_plan_.updateID(drone_id_);
  tm_voro_map_init_.updateID(drone_id_);

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
  // bool_map_sub_ = nh.subscribe<gestelt_msgs::BoolMapArray>("bool_map_arr", 50, 
  //                                                           &VoronoiPlanner::boolMapCB, this);

  odom_sub_ = nh.subscribe("odom", 5, &VoronoiPlanner::odometryCB, this);

  /* Timers */
  plan_fe_timer_ = nh.createTimer(ros::Duration(1.0/fe_planner_freq_), 
                                  &VoronoiPlanner::planFETimerCB, this);

  gen_voro_map_timer_ = nh.createTimer(ros::Duration(1.0/gen_voro_map_freq_), 
                                  &VoronoiPlanner::genVoroMapTimerCB, this);

  /* Initialization */

  // Initialize planner

  astar_params_.drone_id = drone_id_;
  astar_params_.max_iterations = 9999;
  astar_params_.debug_viz = true;
  astar_params_.tie_breaker = 1.001;
  astar_params_.cost_function_type  = 1; // 0: getOctileDist, 1: getL1Norm, 2: getL2Norm, 3: getChebyshevDist
  astar_params_.t_unit = t_unit_;
  fe_planner_ = std::make_unique<AStarPlanner>(astar_params_);

  // Initialize map
  map_.reset(new GridMap);
  map_->initMapROS(nh, pnh);
}

void VoronoiPlanner::initParams(ros::NodeHandle &pnh)
{
  pnh.param("drone_id", drone_id_, -1);
  pnh.param("num_agents", num_agents_, 1);

  double goal_tol;
  pnh.param("planner/goal_tolerance", goal_tol, 0.1);
  sqr_goal_tol_ = goal_tol * goal_tol;
  pnh.param("planner/verbose_print", verbose_print_, false);
  pnh.param("planner/planner_frequency", fe_planner_freq_, 10.0);
  pnh.param("planner/generate_voronoi_frequency", gen_voro_map_freq_, 10.0);
  pnh.param("planner/plan_once", plan_once_, false);
  pnh.param("planner/t_unit", t_unit_, 0.1);

  pnh.param("reservation_table/inflation", rsvn_tbl_inflation_, 0.3);
  pnh.param("reservation_table/time_buffer", rsvn_tbl_t_buffer_, 0.5);
  pnh.param("reservation_table/window_size", rsvn_tbl_window_size_, -1);
  t_buffer_ = (int) std::lround(rsvn_tbl_t_buffer_/t_unit_);  // [space-time units] for time buffer

  pnh.param("local_map_origin", local_map_origin_, std::string("local_map_origin"));
  pnh.param("global_origin", global_origin_, std::string("world"));
}

/* Timer callbacks*/
void VoronoiPlanner::planFETimerCB(const ros::TimerEvent &e)
{
  if (!init_voro_maps_){
    return;
  }

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

void VoronoiPlanner::genVoroMapTimerCB(const ros::TimerEvent &e)
{
  if (!map_->getBoolMap3D(bool_map_3d_)){
    return;
  }

  tm_voro_map_init_.start();

  std::lock_guard<std::mutex> voro_map_guard(voro_map_mtx_);

  std::vector<Eigen::Vector3d> voro_verts;

  for (auto const& bool_map : bool_map_3d_.bool_maps)
  {
    int z_cm = bool_map.first;
    double z_m = cmToM(bool_map.first);

    // Create DynamicVoronoi object if it does not exist
    DynamicVoronoi::DynamicVoronoiParams dyn_voro_params;
    dyn_voro_params.res = bool_map_3d_.resolution;
    dyn_voro_params.origin_x = 0.0;
    dyn_voro_params.origin_y = 0.0;
    dyn_voro_params.origin_z = z_m;
    dyn_voro_params.origin_z_cm = z_cm;

    // Initialize dynamic voronoi 
    dyn_voro_arr_[z_cm] = std::make_shared<DynamicVoronoi>(dyn_voro_params);
    dyn_voro_arr_[z_cm]->initializeMap(bool_map_3d_.width, 
                                      bool_map_3d_.height, 
                                      bool_map.second);
    
    dyn_voro_arr_[z_cm]->update(); // update distance map and Voronoi diagram
    dyn_voro_arr_[z_cm]->prune();  // prune the Voronoi
    dyn_voro_arr_[z_cm]->updateAlternativePrunedDiagram();  

    // Get voronoi graph for current layer and append to voro_verts
    std::vector<Eigen::Vector3d> voro_verts_cur_layer = dyn_voro_arr_[z_cm]->getVoronoiVertices();
    voro_verts.insert(voro_verts.end(), voro_verts_cur_layer.begin(), voro_verts_cur_layer.end());

    nav_msgs::OccupancyGrid occ_grid, voro_occ_grid;

    occmapToOccGrid(*dyn_voro_arr_[z_cm], 
                    bool_map_3d_.origin(0), bool_map_3d_.origin(1), 
                    occ_grid); // Occupancy map

    voronoimapToOccGrid(*dyn_voro_arr_[z_cm], 
                        bool_map_3d_.origin(0), bool_map_3d_.origin(1), 
                        voro_occ_grid); // Voronoi map

    voro_occ_grid_pub_.publish(voro_occ_grid);
    occ_map_pub_.publish(occ_grid);

  }

  // Link to the layer on top for bottommost layer
  dyn_voro_arr_[bool_map_3d_.min_height_cm]->top_voro_ = dyn_voro_arr_[bool_map_3d_.min_height_cm + bool_map_3d_.z_separation_cm];
  // Link to layer below for topmost layer
  dyn_voro_arr_[bool_map_3d_.max_height_cm]->bottom_voro_ = dyn_voro_arr_[bool_map_3d_.max_height_cm - bool_map_3d_.z_separation_cm];

  // Link all voronoi layers together
  for (int z_cm = bool_map_3d_.min_height_cm + bool_map_3d_.z_separation_cm ; 
          z_cm < bool_map_3d_.max_height_cm - bool_map_3d_.z_separation_cm; 
          z_cm += bool_map_3d_.z_separation_cm){
    // link to both layers on top and below
    dyn_voro_arr_[z_cm]->bottom_voro_ = dyn_voro_arr_[z_cm - bool_map_3d_.z_separation_cm];
    dyn_voro_arr_[z_cm]->top_voro_ = dyn_voro_arr_[z_cm + bool_map_3d_.z_separation_cm];
  }

  tm_voro_map_init_.stop(false);

  viz_helper::publishVertices(voro_verts, local_map_origin_, voronoi_graph_pub_);

  init_voro_maps_ = true; // Flag to indicate that all voronoi maps have been initialized
}

/* Subscriber callbacks*/

void VoronoiPlanner::FEPlanSubCB(const gestelt_msgs::FrontEndPlanConstPtr& msg)
{
  if (!init_voro_maps_){
    return;
  }

  // PRIORITY-BASED PLANNING: Only consider trajectories of drones with lower id
  if (msg->agent_id >= drone_id_ ){
    return;
  }

  std::lock_guard<std::mutex> rsvn_tbl_guard(rsvn_tbl_mtx_);

  // Clear reservation table
  rsvn_tbl_[msg->agent_id] = RsvnTable(msg->t_plan_start);
  
  // Add all points on path (with inflation) to reservation table
  cells_inf_ = (int) std::lround(rsvn_tbl_inflation_/bool_map_3d_.resolution); // Number of cells used for inflation

  int window_size = (rsvn_tbl_window_size_ > 0) ? std::min(rsvn_tbl_window_size_, (int) msg->plan.size()) : msg->plan.size() ; 

  int prev_t = 0; // prev_t: Relative time of last points
  for (size_t i = 0; i < (size_t) window_size; i++){ // For points on plan up to window size
    IntPoint grid_pos;
    // get map position relative to local origin
    DblPoint map_2d_pos(msg->plan[i].position.x - bool_map_3d_.origin(0), 
                        msg->plan[i].position.y - bool_map_3d_.origin(1));
    int map_z_cm =roundToMultInt(mToCm(msg->plan[i].position.z), bool_map_3d_.z_separation_cm);
    {
      std::lock_guard<std::mutex> voro_map_guard(voro_map_mtx_);
      dyn_voro_arr_[map_z_cm]->posToIdx(map_2d_pos, grid_pos);
    }

    // Inflate the cells by the given inflation radius
    for(int x = grid_pos.x - cells_inf_; x <= grid_pos.x + cells_inf_; x++)
    {
      for(int y = grid_pos.y - cells_inf_; y <= grid_pos.y + cells_inf_; y++)
      {
        // Reserve for time interval from previous t to current t, including time buffer
        for (int j = - t_buffer_; j < msg->plan_time[i] - prev_t + t_buffer_; j++) { 
          rsvn_tbl_[msg->agent_id].table.insert(Eigen::Vector4i{ x, y, 
                                                                map_z_cm, 
                                                                prev_t + j});
        }
      }
    }

    prev_t = msg->plan_time[i]; 
  } 

}

void VoronoiPlanner::goalsCB(const gestelt_msgs::GoalsConstPtr &msg)
{
    if (msg->transforms.size() <= 0)
    {
      ROS_ERROR("Received empty waypoints. Ignoring waypoints.");
      return;
    }
    if (msg->header.frame_id != "world" && msg->header.frame_id != "map" )
    {
      ROS_ERROR("Only waypoint goals in 'world' or 'map' frame are accepted, ignoring waypoints.");
      return;
    }

    std::vector<Eigen::Vector3d> wp_vec;

    for (auto& pos : msg->transforms) {
      // Transform received waypoints from world to UAV origin frame
      wp_vec.push_back(Eigen::Vector3d(
                        pos.translation.x - bool_map_3d_.origin(0), 
                        pos.translation.y - bool_map_3d_.origin(1), 
                        pos.translation.z));
    }

    waypoints_.addMultipleWP(wp_vec);
}

void VoronoiPlanner::planReqDbgCB(const gestelt_msgs::PlanRequestDebugConstPtr &msg)
{
  Eigen::Vector3d plan_start( 
                  msg->start.position.x - bool_map_3d_.origin(0),
                  msg->start.position.y - bool_map_3d_.origin(1),
                  msg->start.position.z);

  Eigen::Vector3d plan_end( 
                  msg->goal.position.x - bool_map_3d_.origin(0),
                  msg->goal.position.y - bool_map_3d_.origin(1),
                  msg->goal.position.z);

  std::cout << "Agent " << msg->agent_id << ": plan request from ("<< 
                plan_start.transpose() << ") to (" << plan_end.transpose() << ")" << std::endl;

  plan(plan_start, plan_end);
}

void VoronoiPlanner::odometryCB(const nav_msgs::OdometryConstPtr &msg)
{
  cur_pos_= Eigen::Vector3d{msg->pose.pose.position.x - bool_map_3d_.origin(0), 
                            msg->pose.pose.position.y - bool_map_3d_.origin(1), 
                            msg->pose.pose.position.z};
  // cur_vel_= Eigen::Vector3d{msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z};
}

/* Planning functions */

bool VoronoiPlanner::plan(const Eigen::Vector3d& start, const Eigen::Vector3d& goal){
  if (!init_voro_maps_){
    std::cout << "Voronoi maps not initialized! Request a plan after initialization!" << std::endl;
    return false;
  }

  if (plan_once_ && plan_complete_){
    return true;
  }

  auto isAllPrioPlansRcv = [&] () {
    for (int i = 0; i < drone_id_; i++){
      if (rsvn_tbl_.find(i) == rsvn_tbl_.end()){
        return false;
      }
    }
    return true;
  };

  while (!isAllPrioPlansRcv()){
    ros::Duration(0.001).sleep();  // sleep for 1 ms
  }

  // Assign voronoi map
  {
    std::lock_guard<std::mutex> voro_map_guard(voro_map_mtx_);
    fe_planner_->assignVoroMap(dyn_voro_arr_, 
                                bool_map_3d_.z_separation_cm, 
                                bool_map_3d_.origin(0), bool_map_3d_.origin(1),
                                bool_map_3d_.max_height_cm,
                                bool_map_3d_.min_height_cm,
                                bool_map_3d_.resolution);
  }

  // Update reservation table on planner
  {
    std::lock_guard<std::mutex> rsvn_tbl_guard(rsvn_tbl_mtx_);
    fe_planner_->updateReservationTable(rsvn_tbl_);
    rsvn_tbl_.clear();
  }


  viz_helper::publishStartAndGoal(start, goal, local_map_origin_, start_pt_pub_, goal_pt_pub_);

  tm_front_end_plan_.start();

  // Generate plan 
  if (!fe_planner_->generatePlanVoroT(start, goal))
  {
    ROS_ERROR("Drone %d: Failed to generate plan from (%f, %f, %f) to (%f, %f, %f)", drone_id_, start(0), start(1), start(2),
                                                                            goal(0), goal(1), goal(2));

    tm_front_end_plan_.stop(verbose_print_);

    // viz_helper::publishClosedList(fe_planner_->getClosedListVoroT(), 
    //                   fe_closed_list_pub_, local_map_origin_);

    // ROS_ERROR("Closed list size: %ld", fe_planner_->getClosedListVoroT().size() );

    return false;
  }


  tm_front_end_plan_.stop(verbose_print_);

  // Retrieve space time path and publish it
  front_end_path_ = fe_planner_->getPath(cur_pos_);
  space_time_path_ = fe_planner_->getPathWithTime(cur_pos_);
  // smoothed_path_ = fe_planner_->getSmoothedPath();
  // smoothed_path_t_ = fe_planner_->getSmoothedPathWithTime();
  // viz_helper::publishClosedList(fe_planner_->getClosedListVoroT(), fe_closed_list_pub_, local_map_origin_);

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

  fe_plan_msg.t_plan_start = ros::Time::now().toSec();

  fe_plan_pub_.publish(fe_plan_msg);
  fe_plan_broadcast_pub_.publish(fe_plan_msg);

  // viz_helper::publishSpaceTimePath(space_time_path_, global_origin_, fe_plan_viz_pub_) ;

  viz_helper::publishFrontEndPath(front_end_path_, global_origin_, fe_plan_viz_pub_);

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

  plan_complete_ = true;

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
  return (pos - goal).squaredNorm() < sqr_goal_tol_;
}