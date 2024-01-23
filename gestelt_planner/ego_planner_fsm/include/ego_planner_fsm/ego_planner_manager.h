#ifndef _EGO_PLANNER_MANAGER_H_
#define _EGO_PLANNER_MANAGER_H_

#include <stdlib.h>

#include <Eigen/Eigen>

#include <ros/ros.h>

#include <traj_utils/plan_container.hpp>
#include <traj_utils/planning_visualization.h>

#include <optimizer/poly_traj_optimizer.h>

#include <grid_map/grid_map.h>

namespace ego_planner
{
  // Key algorithms of mapping and planning are called
  class EGOPlannerManager
  {
  public:
    EGOPlannerManager();
    ~EGOPlannerManager();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /* main planning interface */
    void initPlanModules(ros::NodeHandle &nh, ros::NodeHandle &pnh, PlanningVisualization::Ptr vis = NULL);

    bool computeInitState(
        const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
        const Eigen::Vector3d &start_acc, const Eigen::Vector3d &local_target_pt,
        const Eigen::Vector3d &local_target_vel, const bool flag_polyInit,
        const bool flag_randomPolyTraj, const double &ts, poly_traj::MinJerkOpt &initMJO);

    /**
     * @brief 
     * 
     * @param start_pt 
     * @param start_vel 
     * @param start_acc 
     * @param end_pt 
     * @param end_vel 
     * @param formation_start_pt 
     * @param formation_end_pt 
     * @param flag_polyInit Intialize new polynomial if true
     * @param flag_randomPolyTraj Randomize inner points if true
     * @param touch_goal 
     * @return true 
     * @return false 
     */
    bool reboundReplan(
        const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
        const Eigen::Vector3d &start_acc, 
        const Eigen::Vector3d &end_pt, const Eigen::Vector3d &end_vel, 
        const bool flag_polyInit, 
        const bool flag_randomPolyTraj, const bool touch_goal);

    /**
     * @brief Get a global trajectory given boundary conditions and waypoints
     * 
     * @param start_pos 
     * @param start_vel 
     * @param start_acc 
     * @param waypoints 
     * @param end_vel 
     * @param end_acc 
     * @return true 
     * @return false 
     */
    bool planGlobalTrajWaypoints(
        const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel,
        const Eigen::Vector3d &start_acc, const std::vector<Eigen::Vector3d> &waypoints,
        const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);
    
    void getLocalTarget(
        const double planning_horizon,
        const Eigen::Vector3d &start_pt, const Eigen::Vector3d &global_end_pt,
        Eigen::Vector3d &local_target_pos, Eigen::Vector3d &local_target_vel,
        bool &touch_goal);

    bool EmergencyStop(Eigen::Vector3d stop_pos);

    bool checkCollision(int drone_id);

    bool setLocalTrajFromOpt(const poly_traj::MinJerkOpt &opt, const bool touch_goal);
    
    double getSwarmClearance(void) { 
      return ploy_traj_opt_->get_swarm_clearance_(); 
    }

    /**
     * @brief Return Number of distinctive constraint points per piece
     * 
     * @return int Number of distinctive constraint points per piece
     */
    int getCpsNumPrePiece(void) { 
      return ploy_traj_opt_->get_cps_num_perPiece_(); 
    }
    // PtsChk_t getPtsCheck(void) { return ploy_traj_opt_->get_pts_check_(); }

    PlanParameters pp_;
    GridMap::Ptr grid_map_;
    TrajContainer traj_;

  private:
    PlanningVisualization::Ptr visualization_;

    PolyTrajOptimizer::Ptr ploy_traj_opt_;

    int continous_failures_count_{0};

  public:
    typedef std::unique_ptr<EGOPlannerManager> Ptr;

    // !SECTION
  };
} // namespace ego_planner

#endif // _EGO_PLANNER_MANAGER_H_