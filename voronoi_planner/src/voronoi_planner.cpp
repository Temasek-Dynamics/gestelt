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
  bool_map_sub_ = nh.subscribe<gestelt_msgs::BoolMap>("bool_map", 50, &VoronoiPlanner::boolMapCB, this);

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
  astar_params_.tie_breaker = 1.00001;
  astar_params_.cost_function_type  = 2;

}

void VoronoiPlanner::initParams(ros::NodeHandle &pnh)
{
  pnh.param("map_filename", map_fname_, std::string(""));
  pnh.param("resolution", res_, 0.1);
  // pnh.param("negate", negate_, false);
  // pnh.param("occ_threshold", occ_th_, 100.0);
  // pnh.param("free_threshold", free_th_, 0.0);
  // pnh.param("yaw", yaw_, 0.0);

  pnh.param("z_multiple", z_multiple_, -1);
  
  pnh.param("verbose_planning", verbose_planning_, false);

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

void VoronoiPlanner::boolMapCB(const gestelt_msgs::BoolMapConstPtr& msg)
{ 
  int z_origin_cm = (int) (msg->origin.z * 100); // Convert z origin to cm
  local_origin_x_ = msg->origin.x;
  local_origin_y_ = msg->origin.y;

  if (bool_map_arr_.find(z_origin_cm) == bool_map_arr_.end()){
    // Initialize bool map 2d vector
    bool_map_arr_[z_origin_cm] = std::make_shared<std::vector<std::vector<bool>>>(msg->height, std::vector<bool>(msg->width, false));
  }

  // set values of boolean map
  for(int j = 0; j < msg->height; j++)
  {
    for (int i = 0; i < msg->width; i++)
    {
      (*bool_map_arr_[z_origin_cm])[i][j] = msg->map[i + j * msg->width];
    }
  }

  // set map boundaries as occupied
  for(int j = 0; j < msg->height; j++)
  {
    (*bool_map_arr_[z_origin_cm])[0][j] = true;
    (*bool_map_arr_[z_origin_cm])[msg->width-1][j] = true;
  }
  for (int i = 0; i < msg->width; i++)
  {
    (*bool_map_arr_[z_origin_cm])[i][0] = true;
    (*bool_map_arr_[z_origin_cm])[i][msg->height-1] = true;
  }

  // tm_voronoi_map_init_.start();

  if (dyn_voro_arr_.find(z_origin_cm) == dyn_voro_arr_.end()){

    DynamicVoronoi::DynamicVoronoiParams dyn_voro_params;
    dyn_voro_params.resolution = res_;
    dyn_voro_params.origin_x = 0.0;
    dyn_voro_params.origin_y = 0.0;
    dyn_voro_params.origin_z = msg->origin.z;

    // Initialize dynamic voronoi 
    dyn_voro_arr_[z_origin_cm] = std::make_shared<DynamicVoronoi>(dyn_voro_params);
    dyn_voro_arr_[z_origin_cm]->initializeMap(msg->width, msg->height, bool_map_arr_[z_origin_cm]);
  }
  dyn_voro_arr_[z_origin_cm]->update(); // update distance map and Voronoi diagram


  // if (doPrune){
  dyn_voro_arr_[z_origin_cm]->prune();  // prune the Voronoi
  // }
  // else if (doPruneAlternative) { // prune the Voronoi
    // dyn_voro_arr_[z_origin_cm]->updateAlternativePrunedDiagram();  
  // }

  // Publish voronoi graph
  std::vector<Eigen::Vector3d> voronoi_vertices;
  dyn_voro_arr_[z_origin_cm]->getVoronoiVertices(voronoi_vertices);

  // tm_voronoi_map_init_.stop(verbose_planning_);

  publishVertices(voronoi_vertices, z_origin_cm, "local_map_origin", voronoi_graph_pub_);

  nav_msgs::OccupancyGrid occ_grid, voro_occ_grid;

  occmapToOccGrid(*dyn_voro_arr_[z_origin_cm], 
                  msg->origin.x, msg->origin.y, 
                  occ_grid); // Occupancy map

  voronoimapToOccGrid(*dyn_voro_arr_[z_origin_cm], 
                      msg->origin.x, msg->origin.y, 
                      voro_occ_grid); // Voronoi map

  voro_occ_grid_pub_.publish(voro_occ_grid);
  occ_map_pub_.publish(occ_grid);
}

// void VoronoiPlanner::pgmFileToBoolMap(bool ***map,
//                                       int& size_x, int& size_y,
//                                       const std::string& fname)
// {
//   SDL_Surface* img;

//   unsigned char* pixels;
//   unsigned char* p;
//   int rowstride, n_channels;
//   unsigned int i,j;
//   int k;
//   double occ;
//   int color_sum;
//   double color_avg;

//   // Load the image using SDL.  If we get NULL back, the image load failed.
//   if(!(img = IMG_Load(fname.c_str())))
//   {
//     std::string errmsg = std::string("failed to open image file \"") + 
//             fname + std::string("\"");
//     throw std::runtime_error(errmsg);
//   }

//   // Copy the image data into the map structure
//   size_x = img->w;
//   size_y = img->h;

//   *map = new bool*[size_x];
//   for (int x=0; x<size_x; x++) {
//     (*map)[x] = new bool[size_y];
//   }

//   // Allocate space to hold the data
//   occ_idx_.clear();
//   free_idx_.clear();

//   // Get values that we'll need to iterate through the pixels
//   rowstride = img->pitch;
//   n_channels = img->format->BytesPerPixel;

//   // Copy pixel data into the map structure
//   pixels = (unsigned char*)(img->pixels);
//   for(int j = 0; j < size_y; j++)
//   {
//     for (int i = 0; i < size_x; i++)
//     {
//       // Compute mean of RGB for this pixel
//       p = pixels + j*rowstride + i*n_channels;
//       color_sum = 0;
//       for(k=0;k<n_channels;k++)
//         color_sum += *(p + (k));
//       color_avg = color_sum / (double)n_channels;

//       // If negate is true, we consider blacker pixels free, and whiter
//       // pixels free.  Otherwise, it's vice versa.
//       if(negate_)
//         occ = color_avg / 255.0;
//       else
//         occ = (255 - color_avg) / 255.0;
      
//       // Apply thresholds to RGB means to determine occupancy values for
//       // map.  Note that we invert the graphics-ordering of the pixels to
//       // produce a map with cell (0,0) in the lower-left corner.

//       if(occ > occ_th_){  // Occupied
//         (*map)[i][j] = true;
//       }
//       else if(occ < free_th_){ // Free
//         (*map)[i][j] = false;
//       }
//       else{ // Unknown
//         (*map)[i][j] = true;
//       }


//     }
//   }

//   SDL_FreeSurface(img);
// }


// void VoronoiPlanner::loadMapFromFile(nav_msgs::OccupancyGrid& map,
//                                       const std::string& fname)
// {
//   SDL_Surface* img;
  
//   unsigned char* pixels;
//   unsigned char* p;
//   int rowstride, n_channels;
//   unsigned int i,j;
//   int k;
//   double occ;
//   int color_sum;
//   double color_avg;

//   // Load the image using SDL.  If we get NULL back, the image load failed.
//   if(!(img = IMG_Load(fname.c_str())))
//   {
//     std::string errmsg = std::string("failed to open image file \"") + 
//             fname + std::string("\"");
//     throw std::runtime_error(errmsg);
//   }

//   // Copy the image data into the map structure
//   map.header.stamp = ros::Time::now();
//   map.header.frame_id = "map";
//   map.info.width = img->w;
//   map.info.height = img->h;
//   map.info.resolution = res_;
//   map.info.origin.position.x = origin_x_;
//   map.info.origin.position.y = origin_y_;
//   map.info.origin.position.z = 0.0;
//   tf2::Quaternion q;
//   q.setRPY(0, 0, 0.0);
//   map.info.origin.orientation.x = q.x();
//   map.info.origin.orientation.y = q.y();
//   map.info.origin.orientation.z = q.z();
//   map.info.origin.orientation.w = q.w();

//   // Allocate space to hold the data
//   map.data.resize(map.info.width * map.info.height);
//   occ_idx_.clear();
//   free_idx_.clear();

//   // Get values that we'll need to iterate through the pixels
//   rowstride = img->pitch;
//   n_channels = img->format->BytesPerPixel;

//   // Copy pixel data into the map structure
//   pixels = (unsigned char*)(img->pixels);
//   for(int j = 0; j < map.info.height; j++)
//   {
//     for (int i = 0; i < map.info.width; i++)
//     {
//       // Compute mean of RGB for this pixel
//       p = pixels + j*rowstride + i*n_channels;
//       color_sum = 0;
//       for(k=0;k<n_channels;k++)
//         color_sum += *(p + (k));
//       color_avg = color_sum / (double)n_channels;

//       // If negate is true, we consider blacker pixels free, and whiter
//       // pixels free.  Otherwise, it's vice versa.
//       if(negate_)
//         occ = color_avg / 255.0;
//       else
//         occ = (255 - color_avg) / 255.0;
      
//       // Apply thresholds to RGB means to determine occupancy values for
//       // map.  Note that we invert the graphics-ordering of the pixels to
//       // produce a map with cell (0,0) in the lower-left corner.
//       size_t idx = map2Dto1DIdx(map.info.width, i, map.info.height - j - 1);

//       if(occ > occ_th_){  // Occupied
//         map.data[idx] = cost_val::OCC;
//         occ_idx_.push_back(idx);
//       }
//       else if(occ < free_th_){ // Free
//         map.data[idx] = cost_val::FREE;
//         free_idx_.push_back(idx);
//       }
//       else{ // Unknown
//         map.data[idx] = cost_val::UNKNOWN;
//         unknown_idx_.push_back(idx);
//       }
//     }
//   }

//   SDL_FreeSurface(img);
// }
