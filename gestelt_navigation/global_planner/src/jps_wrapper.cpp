#include <global_planner/jps_wrapper.h>

JPSWrapper::JPSWrapper(std::shared_ptr<GridMap> map, const JPSParams& params):
    params_(params)
{
    map_ = map;
    jps_planner_ = std::make_shared<JPSPlanner3D>(params_.planner_verbose);
}

void JPSWrapper::reset()
{
    path_jps_.clear();
    path_jps_raw_.clear();
    path_dmp_.clear();
    path_dmp_raw_.clear();
    dmp_search_region_.clear();
}

void JPSWrapper::addPublishers(
    std::unordered_map<std::string, ros::Publisher> &publisher_map)
{
    // closed_list_viz_pub_ = publisher_map["front_end/closed_list"];
}

bool JPSWrapper::generatePlan(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &goal_pos)
{
    reset();
    std::shared_ptr<JPS::VoxelMapUtil> map_util = ::std::make_shared<JPS::VoxelMapUtil>();

    tm_jps_map_.start();
        map_->updateLocalMap();
        // Origin of local map should be set relative to UAV current position (i.e. doesn't change unless we change the local map size)
        map_util->setMap(map_->getLocalOrigin(), 
                        map_->getLocalMapNumVoxels(), 
                        map_->getData(),
                        map_->getRes());

        jps_planner_->setMapUtil(map_util);
        jps_planner_->updateMap();
    tm_jps_map_.stop(params_.print_timers);

    tm_jps_plan_.start();
        if( !jps_planner_->plan(start_pos, goal_pos, 1, true)){
            std::cout << "[Front-End] JPS Planner failed!" << std::endl;
            return false;
        }
    tm_jps_plan_.stop(params_.print_timers);

    for (auto& pt : jps_planner_->getRawPath()){
        path_jps_raw_.push_back(pt);
    }

    if (params_.interpolate)
    {
        path_jps_intp_ = interpolatePath(path_jps_raw_);
    }

    for (auto& pt : jps_planner_->getPath()){
        path_jps_.push_back(pt);
    }

    if (params_.use_dmp){
        tm_dmp_plan_.start();

            // Set up DMP planner
            IterativeDMPlanner3D dmp(params_.planner_verbose);
            dmp.setPotentialRadius(Vec3f(params_.dmp_pot_rad, params_.dmp_pot_rad, params_.dmp_pot_rad));        // Set 3D potential field radius
            dmp.setSearchRadius(Vec3f(params_.dmp_search_rad, params_.dmp_search_rad, params_.dmp_search_rad)); // Set the valid search region around given path
            dmp.setEps(params_.dmp_heuristic_weight); // Set weight on heuristic
            dmp.setCweight(params_.dmp_col_weight); // Set collision cost weight
            dmp.setPow(params_.dmp_pow); // Set collision cost weight

            // Set map util for collision checking, must be called before planning
            dmp.setMap(map_util, start_pos); 

            // Plan DMP path
            if (!dmp.iterativeComputePath( start_pos, goal_pos, jps_planner_->getRawPath(), 1)){ // Compute the path given the jps path
                std::cout << "[Front-End] DMP Planner failed!" << std::endl;
                return false;
            }
        tm_dmp_plan_.stop(params_.print_timers);

        // TODO: Figure out way to assign vectors without iteration
        for (auto& pt : dmp.getPath()){
            path_dmp_.push_back(pt);
        }
        for (auto& pt : dmp.getRawPath()){
            path_dmp_raw_.push_back(pt);
        }
        // Get search region
        for (auto& pt : dmp.getSearchRegion()) {
            dmp_search_region_.push_back(pt);
        }
    }

    return true;
}


std::vector<Eigen::Vector3d> JPSWrapper::interpolatePath(const std::vector<Eigen::Vector3d>& path)
{
    std::vector<Eigen::Vector3d> path_intp; // Interpolated path

    double unit_diag_dist = sqrt(2 * map_->getRes()* map_->getRes()) + 0.00001;

    for (size_t i = 0; i < path.size()-1; i++)
    {
        double dist_btw_pts = (path[i+1] - path[i]).norm();

        // Compare current point [i] and next point [i+1] to see if their distance 
        //      exceeds a threshold. If so, interpolate between them
        if (dist_btw_pts >= unit_diag_dist)
        {
            Eigen::Vector3d fwd_vec = (path[i+1] - path[i]).normalized();
            
            Eigen::Vector3d pt_intp = path[i]; // Interpolated point

            int step = 0;
            while ((pt_intp - path[i]).norm() < dist_btw_pts){
                pt_intp = path[i] + step * map_->getRes() * fwd_vec;

                path_intp.push_back(pt_intp);
                step++;
            }
        }
        else {
            path_intp.push_back(path[i]);
        }
    }
    // Push the goal
    path_intp.push_back(path.back());

    return path_intp;
}   
 