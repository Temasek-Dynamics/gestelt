#include <global_planner/jps_wrapper.h>

JPSWrapper::JPSWrapper(std::shared_ptr<GridMap> map, const JPSParams& jps_params):
    jps_params_(jps_params)
{
    map_ = map;
    // jps_planner_ = std::make_shared<path_finding_util::GraphSearch>(map_->getLocalMapNumVoxels(), 1, true);
    jps_planner_ = std::make_shared<JPSPlanner3D>( planner_verbose_);
}

void JPSWrapper::reset()
{
    path_pos_.clear();
}

bool JPSWrapper::generatePlan(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &goal_pos)
{
    reset();
    
    std::cout << "JPS generate plan " << std::endl;
    std::shared_ptr<JPS::VoxelMapUtil> map_util = ::std::make_shared<JPS::VoxelMapUtil>();

    map_->updateLocalMapData();
    // map_util->setMap(map_->getOrigin(), map_->getLocalMapNumVoxels(), map_->getData(),
    //                  map_->getRes());
    map_util->setMap(Eigen::Vector3d(0.0, 0.0,0.0), map_->getLocalMapNumVoxels(), map_->getData(),
                     map_->getRes());

    jps_planner_->setMapUtil(map_util);
    jps_planner_->updateMap();

    // define start and goal
    // Eigen::Vector3d start_pos_offset = start_pos;
    Eigen::Vector3d start_pos_offset = Eigen::Vector3d(2.0, 2.0, 2.0);
    Eigen::Vector3d goal_pos_offset = Eigen::Vector3d(5.0, 5.0, 5.0);

    if (!jps_planner_->plan(start_pos_offset, goal_pos_offset, 1, true)){
        return false;
    }

    auto path_jps = jps_planner_->getRawPath();

    // Convert from float to double 
    for (auto point : path_jps){
        path_pos_.push_back(point.cast<double>());
    }

    // Eigen::Vector3d start_local = map_->GetCoordLocal(start_pos);
    // Eigen::Vector3d goal_local = map_->GetCoordLocal(goal_pos);
    // Eigen::Vector3i start_i = start_local.cast<int>();
    // Eigen::Vector3i goal_i = goal_local.cast<int>();

    // jps_planner_->Plan(map_, start_i, goal_i, true);
    // std::vector<path_finding_util::StatePtr> path = jps_planner_->GetPath();
    // std::vector<Eigen::Vector3d> path_out = path_finding_util::ConvertPathToVector(path);

    // path_pos_ = path_finding_util::PathLocalToGlobal(path_out, map_);

    // // reverse the path
    // std::reverse(path_pos_.begin(), path_pos_.end());

    return true;
}

/**
 * @brief Get successful plan in terms of path positions
 *
 * @return std::vector<Eigen::Vector3d>
 */
std::vector<Eigen::Vector3d> JPSWrapper::getPathPos()
{
    return path_pos_;
}
