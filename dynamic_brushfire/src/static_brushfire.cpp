#include <static_brushfire/static_brushfire.hpp>

void StaticBrushfire::init(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{ 
  occ_map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("dyn_brushfire/occ_map", 10, true);
  dist_occ_map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("dyn_brushfire/dist_map", 10, true);
  voro_occ_grid_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("dyn_brushfire/voronoi_map", 10, true);

  initParams(pnh);

  pgmFileToBoolMap(&bool_map_, size_x_, size_y_, map_fname_);

  DynamicVoronoi dyn_voro;
  dyn_voro.initializeMap(size_x_, size_y_, bool_map_);
  dyn_voro.update(); // update distance map and Voronoi diagram
  dyn_voro.visualize("/home/john/gestelt_ws/src/gestelt/dynamic_brushfire/maps/final.ppm");

  // if (doPrune){
  //   dyn_voro.prune();  // prune the Voronoi
  // }
  // else if (doPruneAlternative) { // prune the Voronoi
  //   dyn_voro.updateAlternativePrunedDiagram();  
  // }

  // mapToOccGrid(dist_map_, occ_grid_.info.width, occ_grid_.info.height, dist_occ_grid_);
  voronoimapToOccGrid(dyn_voro, size_x_, size_y_,  voro_occ_grid_);

  // occ_map_pub_.publish(occ_grid_);
  // dist_occ_map_pub_.publish(dist_occ_grid_);
  voro_occ_grid_pub_.publish(voro_occ_grid_);

  // loadMapFromFile(occ_grid_, map_fname_);
  // computeDistanceMap();

}

void StaticBrushfire::initParams(ros::NodeHandle &pnh)
{
  pnh.param("map_filename", map_fname_, std::string(""));
  pnh.param("resolution", res_, 0.1);
  pnh.param("negate", negate_, false);
  pnh.param("occ_threshold", occ_th_, 100.0);
  pnh.param("free_threshold", free_th_, 0.0);
  pnh.param("origin_x", origin_x_, 0.0);
  pnh.param("origin_y", origin_y_, 0.0);
  pnh.param("yaw", yaw_, 0.0);
}

void StaticBrushfire::pgmFileToBoolMap(bool ***map,
                                      int& size_x, int& size_y,
                                      const std::string& fname)
{
  SDL_Surface* img;

  unsigned char* pixels;
  unsigned char* p;
  int rowstride, n_channels;
  unsigned int i,j;
  int k;
  double occ;
  int color_sum;
  double color_avg;

  // Load the image using SDL.  If we get NULL back, the image load failed.
  if(!(img = IMG_Load(fname.c_str())))
  {
    std::string errmsg = std::string("failed to open image file \"") + 
            fname + std::string("\"");
    throw std::runtime_error(errmsg);
  }

  // Copy the image data into the map structure
  size_x = img->w;
  size_y = img->h;

  *map = new bool*[size_x];
  for (int x=0; x<size_x; x++) {
    (*map)[x] = new bool[size_y];
  }

  // Allocate space to hold the data
  occ_idx_.clear();
  free_idx_.clear();

  // Get values that we'll need to iterate through the pixels
  rowstride = img->pitch;
  n_channels = img->format->BytesPerPixel;

  // Copy pixel data into the map structure
  pixels = (unsigned char*)(img->pixels);
  for(int j = 0; j < size_y; j++)
  {
    for (int i = 0; i < size_x; i++)
    {
      // Compute mean of RGB for this pixel
      p = pixels + j*rowstride + i*n_channels;
      color_sum = 0;
      for(k=0;k<n_channels;k++)
        color_sum += *(p + (k));
      color_avg = color_sum / (double)n_channels;

      // If negate is true, we consider blacker pixels free, and whiter
      // pixels free.  Otherwise, it's vice versa.
      if(negate_)
        occ = color_avg / 255.0;
      else
        occ = (255 - color_avg) / 255.0;
      
      // Apply thresholds to RGB means to determine occupancy values for
      // map.  Note that we invert the graphics-ordering of the pixels to
      // produce a map with cell (0,0) in the lower-left corner.

      if(occ > occ_th_){  // Occupied
        (*map)[i][j] = true;
      }
      else if(occ < free_th_){ // Free
        (*map)[i][j] = false;
      }
      else{ // Unknown
        (*map)[i][j] = true;
      }


    }
  }

  SDL_FreeSurface(img);
}


void StaticBrushfire::loadMapFromFile(nav_msgs::OccupancyGrid& map,
                                      const std::string& fname)
{
  SDL_Surface* img;

  unsigned char* pixels;
  unsigned char* p;
  int rowstride, n_channels;
  unsigned int i,j;
  int k;
  double occ;
  int color_sum;
  double color_avg;

  // Load the image using SDL.  If we get NULL back, the image load failed.
  if(!(img = IMG_Load(fname.c_str())))
  {
    std::string errmsg = std::string("failed to open image file \"") + 
            fname + std::string("\"");
    throw std::runtime_error(errmsg);
  }

  // Copy the image data into the map structure
  map.header.stamp = ros::Time::now();
  map.header.frame_id = "map";
  map.info.width = img->w;
  map.info.height = img->h;
  map.info.resolution = res_;
  map.info.origin.position.x = origin_x_;
  map.info.origin.position.y = origin_y_;
  map.info.origin.position.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw_);
  map.info.origin.orientation.x = q.x();
  map.info.origin.orientation.y = q.y();
  map.info.origin.orientation.z = q.z();
  map.info.origin.orientation.w = q.w();

  // Allocate space to hold the data
  map.data.resize(map.info.width * map.info.height);
  occ_idx_.clear();
  free_idx_.clear();

  // Get values that we'll need to iterate through the pixels
  rowstride = img->pitch;
  n_channels = img->format->BytesPerPixel;

  // Copy pixel data into the map structure
  pixels = (unsigned char*)(img->pixels);
  for(int j = 0; j < map.info.height; j++)
  {
    for (int i = 0; i < map.info.width; i++)
    {
      // Compute mean of RGB for this pixel
      p = pixels + j*rowstride + i*n_channels;
      color_sum = 0;
      for(k=0;k<n_channels;k++)
        color_sum += *(p + (k));
      color_avg = color_sum / (double)n_channels;

      // If negate is true, we consider blacker pixels free, and whiter
      // pixels free.  Otherwise, it's vice versa.
      if(negate_)
        occ = color_avg / 255.0;
      else
        occ = (255 - color_avg) / 255.0;
      
      // Apply thresholds to RGB means to determine occupancy values for
      // map.  Note that we invert the graphics-ordering of the pixels to
      // produce a map with cell (0,0) in the lower-left corner.
      size_t idx = map2Dto1DIdx(map.info.width, i, map.info.height - j - 1);

      if(occ > occ_th_){  // Occupied
        map.data[idx] = cost_val::OCC;
        occ_idx_.push_back(idx);
      }
      else if(occ < free_th_){ // Free
        map.data[idx] = cost_val::FREE;
        free_idx_.push_back(idx);
      }
      else{ // Unknown
        map.data[idx] = cost_val::UNKNOWN;
        unknown_idx_.push_back(idx);
      }
    }
  }

  SDL_FreeSurface(img);
}

// void StaticBrushfire::computeDistanceMap()
// {
//   dist_map_.resize(occ_grid_.info.width * occ_grid_.info.height, -1);
//   obst_map_.resize(occ_grid_.info.width * occ_grid_.info.height, -1); 

//   open_queue_.clear();

//   for(int j = 0; j < occ_grid_.info.height; j++)
//   {
//     for (int i = 0; i < occ_grid_.info.width; i++)
//     {
//       const size_t idx = map2Dto1DIdx(occ_grid_.info.width, i, occ_grid_.info.height - j - 1);
//       const int8_t s = occ_grid_.data[idx];

//       if (s == cost_val::OCC){ // Occupied
//         obst_map_[idx] = idx; 
//         dist_map_[idx] = 0; 

//         open_queue_.put(idx, 0); // add to open queue
//       }
//       else if (s == cost_val::UNKNOWN){ // Unknown
//          dist_map_[idx] = 0; 
//       }
//       else if (s == cost_val::FREE){ // Free
//          dist_map_[idx] = INF; 
//       }
//       else{
//          dist_map_[idx] = INF; 
//       }
//     }
//   }

//   while (!open_queue_.empty()){
//     const size_t idx = open_queue_.get();
//     lowerStatic(idx);
//   }
// }


void StaticBrushfire::lowerStatic(const size_t& idx)
{
  for (const size_t& nb_idx: get8ConNeighbours(idx))
  { 
    const double d = getL2Norm(obst_map_[idx], nb_idx); //d: distance from nearest obstacle on idx to neighbor
    if (d < dist_map_[nb_idx]) {
      dist_map_[nb_idx] = d;
      obst_map_[nb_idx] = obst_map_[idx];
      open_queue_.put(nb_idx, d); // add to open queue
    }
  }
}

void StaticBrushfire::computeDistanceMap()
{
  dist_map_.resize(occ_grid_.info.width * occ_grid_.info.height, -1);
  obst_map_.resize(occ_grid_.info.width * occ_grid_.info.height, -1); 
  voro_map_.resize(occ_grid_.info.width * occ_grid_.info.height, false); 
  to_raise_.resize(occ_grid_.info.width * occ_grid_.info.height, false); 

  open_queue_.clear();

  for(int j = 0; j < occ_grid_.info.height; j++)
  {
    for (int i = 0; i < occ_grid_.info.width; i++)
    {
      const size_t idx = map2Dto1DIdx(occ_grid_.info.width, i, occ_grid_.info.height - j - 1);
      const int8_t val = occ_grid_.data[idx];

      if (val == cost_val::OCC){ // Occupied
        obst_map_[idx] = idx; 
        dist_map_[idx] = 0; 

        open_queue_.put(idx, 0); // add to open queue
      }
      else if (val == cost_val::UNKNOWN){ // Unknown
         dist_map_[idx] = 0; 
      }
      else if (val == cost_val::FREE){ // Free
         dist_map_[idx] = INF; 
      }
      else{
         dist_map_[idx] = INF; 
      }
    }
  }

  while (!open_queue_.empty()){
    const size_t idx = open_queue_.get();
    if (to_raise_[idx]){
      raise(idx);
    }
    else if (isOcc(obst_map_[idx])){
      voro_map_[idx] = false;
      lower(idx);
    }
  }
  
}
