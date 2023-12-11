#ifndef _A_STAR_PLANNER_H_
#define _A_STAR_PLANNER_H_

#include <global_planner/planner_base.h>

class AStarPlanner : public PlannerBase
{
public:
  AStarPlanner(ros::NodeHandle& nh){
    nh.param("enable_debug", debug_, true);
    nh.param("uav_origin_frame", uav_origin_frame_, std::string("drone0_origin"));

    set_start_sub_ = nh.subscribe("/plan_set_start", 10, &AStarPlanner::setStartCb, this);
    set_goal_sub_ = nh.subscribe("/plan_set_goal", 10, &AStarPlanner::setGoalCb, this);

    trigger_plan_sub_ = nh.subscribe("/trigger_plan", 10, &AStarPlanner::triggerPlanCb, this);

    gridmap_sub_ = nh.subscribe("/gridmap", 10, &AStarPlanner::gridmapSubCb, this);

    plan_viz_pub_ = nh.advertise<visualization_msgs::Marker>("/plan", 10);
    closed_list_viz_pub_ = nh.advertise<visualization_msgs::Marker>("/closed_list", 10);

    grid_map_.reset(new pcl::PointCloud<pcl::PointXYZ>);

    start_pose_(0) = 0.0;
    start_pose_(1) = 0.0;
    start_pose_(2) = 1.0;

    goal_pose_(0) = 4.0;
    goal_pose_(1) = 0.0;
    goal_pose_(2) = 1.0;

  }

  void setStartCb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    ROS_INFO("Start has been set!");
    start_pose_(0) = msg->pose.position.x;
    start_pose_(1) = msg->pose.position.y;
    start_pose_(2) = msg->pose.position.z;
  }

  void setGoalCb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    ROS_INFO("Goal has been set!");
    goal_pose_(0) = msg->pose.position.x;
    goal_pose_(1) = msg->pose.position.y;
    goal_pose_(2) = msg->pose.position.z;
  }

  void triggerPlanCb(const std_msgs::Empty::ConstPtr &msg) {
    ROS_INFO("Planning triggered!");
    if (generate_plan(start_pose_, goal_pose_)){
      common_->publishPath(this->getPath(), plan_viz_pub_);
    }
  }

  void gridmapSubCb(const sensor_msgs::PointCloud2::ConstPtr &msg){
    pcl::fromROSMsg(*msg, *grid_map_);

    if (grid_map_->isOrganized()){
      ROS_ERROR("[global_planner] Organized point clouds are not supported!");
      return;
    }

    if (!grid_map_init_){
      ROS_INFO("[global_planner] Grid map initialized");
      // TODO figure out a way to pass this from the mapper
      Eigen::Vector3d map_size = Eigen::Vector3d(80.0, 80.0, 3.0);
      double ground_height = -0.25;
      Eigen::Vector3d map_origin = Eigen::Vector3d( -map_size(0)/2, -map_size(1)/2, ground_height);
      double map_res = 0.1;

      common_.reset(new PlannerCommon(grid_map_, map_origin, map_size, map_res, uav_origin_frame_));
      grid_map_init_ = true;
    }
    else {
      common_->updateMap(grid_map_);
    }
  }

  // Reset previously saved path and openlist
  void reset(){
    planning_successful_ = false;
    gridnode_path_.clear();
    closed_list_.clear();
    open_list_ = std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, GridNode::CompareCostPtr>();
  }

  // void addGridMap(GridMap::Ptr occ_map){
  //   occ_map_ = occ_map;

  //   common_.reset(new PlannerCommon(occ_map));
  //   grid_map_init_ = true;
  // }

  bool generate_plan(Vector3d start_pos, Vector3d goal_pos){
    if (!grid_map_init_){
      ROS_ERROR("[global_planner] Grid map is not initialized! Unable to start generating plan.");
      return false;
    }
    
    ROS_INFO("[global_planner] generate_plan starting");
    reset();
    // TODO: Convert start and goal pos (in world frame) to the UAV's frame

    Vector3i start_idx, goal_idx;

    if (!common_->posToIdx(start_pos, start_idx) || !common_->posToIdx(goal_pos, goal_idx)){
      return false;
    }

    GridNodePtr start_node = std::make_shared<GridNode>(start_idx);
    GridNodePtr goal_node = std::make_shared<GridNode>(goal_idx);

    start_node->g_cost = 0.0;
    start_node->f_cost = tie_breaker_ * common_->get_euclidean_cost(start_node, goal_node);
    start_node->parent = nullptr;
    
    addToOpenlist(start_node);

    int num_iter = 0;

    while (!open_list_.empty()) 
    {
      GridNodePtr cur_node = popOpenlist();
      addToClosedlist(cur_node); // Mark as visited

      if (*cur_node == *goal_node)
      {
        ROS_INFO("[global_planner] Goal found!");
        // Goal reached, terminate search and obtain path
        planning_successful_ = true;
        tracePath(cur_node);
        return true;
      }

      // Explore neighbors
      for (GridNodePtr nb_node : common_->getNeighbors(cur_node))
      {
        double tent_g_cost = cur_node->g_cost + common_->get_euclidean_cost(cur_node, nb_node);

        if (isInClosedList(nb_node)) {
          // Update explored cost
          if (tent_g_cost < nb_node->g_cost){
            nb_node->g_cost = tent_g_cost;
            nb_node->f_cost = tent_g_cost + tie_breaker_ * common_->get_euclidean_cost(nb_node, goal_node);
          }
          continue;
        }
        
        if (tent_g_cost < nb_node->g_cost) {
          nb_node->g_cost = tent_g_cost;
          nb_node->f_cost = tent_g_cost + tie_breaker_ * common_->get_euclidean_cost(nb_node, goal_node);
          
          nb_node->parent = cur_node;
          addToOpenlist(nb_node);
        }
      }
      num_iter++;

      if (debug_){
        if (num_iter % 50 == 0){
          common_->publishClosedList(closed_list_, closed_list_viz_pub_);
        }
      }
    }

    return false;
  }

  std::vector<Eigen::Vector3d> getPath() {
    return pos_path_;
  }

private:

  void addToOpenlist(GridNodePtr node){
    node->state = CellState::OPEN;
    open_list_.push(node);
  }

  GridNodePtr popOpenlist(){
    GridNodePtr node = open_list_.top();
    open_list_.pop();
    return node;
  }

  void addToClosedlist(GridNodePtr node){
    node->state = CellState::CLOSED; //move current node to closed set.
    closed_list_.insert(node);
  }

  bool isInClosedList(GridNodePtr node){
    if (closed_list_.find(node) != closed_list_.end()){
      return true;
    }
    return false;
  }

  void tracePath(GridNodePtr node) {
    if (!planning_successful_){
      ROS_ERROR("[global_planner] The tracePath() method should only be called when planning is successful. Returning empty path");
      return;
    }

    gridnode_path_.clear();
    idx_path_.clear();
    pos_path_.clear();

    GridNodePtr cur_node = node; 
    while (cur_node->parent != nullptr){
      gridnode_path_.push_back(cur_node);
      cur_node = cur_node->parent;
    }

    // Push back the start node
    gridnode_path_.push_back(cur_node);

    // Reverse the order of the path so that it goes from start to goal
    std::reverse(gridnode_path_.begin(), gridnode_path_.end());

    // Get vector of path node indices and positions
    for (auto gridnode_ptr : gridnode_path_) {
      Eigen::Vector3d gridnode_pos;
      common_->idxToPos(gridnode_ptr->idx, gridnode_pos);

      pos_path_.push_back(gridnode_pos); 
      idx_path_.push_back(gridnode_ptr->idx); 
    }
  }

private: 
  std::vector<GridNodePtr> gridnode_path_;
  std::vector<Eigen::Vector3i> idx_path_; // Path with indices of nodes
  std::vector<Eigen::Vector3d> pos_path_; // Path with position of nodes

  // ROS Subscribers
  ros::Subscriber set_start_sub_, set_goal_sub_;
  ros::Subscriber trigger_plan_sub_;
  ros::Subscriber gridmap_sub_;

  // ROS Publishers
  ros::Publisher plan_viz_pub_, closed_list_viz_pub_;

  /* Params */
  bool debug_{false};
  std::string uav_origin_frame_;
  const double tie_breaker_ = 1.0 + 1.0 / 10000;

  /* Flags */
  bool planning_successful_{false};
  bool grid_map_init_{false};

  /* Temporarily stored data */
  Eigen::Vector3d start_pose_, goal_pose_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr grid_map_;

  /* Path planner data structures */

  // class for commonly used helper functions 
  std::unique_ptr<PlannerCommon> common_; 

  // open list stores nodes yet to be visited
  std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, GridNode::CompareCostPtr> open_list_; 
  // closed list stores nodes that have been visited
  std::unordered_set<GridNodePtr, GridNode::PointedObjHash, GridNode::PointedObjEq> closed_list_;

};

#endif