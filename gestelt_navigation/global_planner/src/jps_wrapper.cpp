#include <global_planner/jps_wrapper.h>

JPSWrapper::JPSWrapper(std::shared_ptr<GridMap> map, const JPSParams& params):
    params_(params)
{
    map_ = map;
    // jps_planner_ = std::make_shared<path_finding_util::GraphSearch>(map_->getLocalMapNumVoxels(), 1, true);
    jps_planner_ = std::make_shared<JPSPlanner3D>(params_.planner_verbose);
}

void JPSWrapper::reset()
{
    path_jps_.clear();
    path_dmp_.clear();
    dmp_search_region_.clear();
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
        bool valid_jps_plan = jps_planner_->plan(start_pos, goal_pos, 1, true);
    tm_jps_plan_.stop(params_.print_timers);

    if (!valid_jps_plan){
        return false;
    }
    auto path_jps = jps_planner_->getRawPath();
    for (auto& pt : path_jps){
        path_jps_.push_back(pt);
    }

    // Set up DMP planner
    IterativeDMPlanner3D dmp(params_.planner_verbose);
    dmp.setPotentialRadius(Vec3f(params_.dmp_pot_rad, params_.dmp_pot_rad, params_.dmp_pot_rad));        // Set 3D potential field radius
    dmp.setSearchRadius(Vec3f(params_.dmp_search_rad, params_.dmp_search_rad, params_.dmp_search_rad)); // Set the valid search region around given path
    dmp.setEps(params_.dmp_heuristic_weight); // Set weight on heuristic
    dmp.setCweight(params_.dmp_col_weight); // Set collision cost weight
    dmp.setPow(params_.dmp_pow); // Set collision cost weight

    tm_c_.start();
        // Set map util for collision checking, must be called before planning
        dmp.setMap(map_util, start_pos); 
    tm_c_.stop(true);

    // Plan DMP path
    tm_dmp_plan_.start();
        bool valid_dmp_plan = dmp.iterativeComputePath( start_pos, goal_pos, path_jps, 1); // Compute the path given the jps path
    tm_dmp_plan_.stop(params_.print_timers);

    if (!valid_dmp_plan){
        return false;
    }
    for (auto& pt : dmp.getRawPath()){
        path_dmp_.push_back(pt);
    }
    // Get search region
    for (auto& pt : dmp.getSearchRegion()) {
        dmp_search_region_.push_back(pt);
    }

    return true;
}