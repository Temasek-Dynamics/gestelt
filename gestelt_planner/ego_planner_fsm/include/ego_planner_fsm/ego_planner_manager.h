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

    /**
     * @brief Generate a minimum jerk trajectory given boundary conditions and inner waypoints while ignoring obstacles
     * 
     * @param start_pt start position vector [x, y, z]
     * @param start_vel start velocity vector [dx, dy, dz]
     * @param start_acc start acceleration vector [ddx, ddy, ddz]
     * @param inner_wps Vector of inner waypoints [ [x_1, y_1, z_1], [x_2, y_2, z_2], ..., [x_M-1, y_M-1, z_M-1]]
     * @param local_target_pos Local target position vector [x, y, z]
     * @param local_target_vel Local target velocity vector [dx, dy, dz]
     * @param segs_t_dur Vector of time durations (s) of each segment (t_1, t_2, ..., t_M)
     * @param mj_opt Minimum jerk trajectory optimizer 
     * @return true 
     * @return false 
     */
    bool generateMinJerkTraj(
      const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
      const std::vector<Eigen::Vector3d>& inner_wps,
      const Eigen::Vector3d &local_target_pos, const Eigen::Vector3d &local_target_vel,
      const Eigen::VectorXd &segs_t_dur,
      poly_traj::MinJerkOpt &mj_opt);

    /**
     * @brief Compute an initial minimum jerk trajectory given boundary conditions while ignoring obstacles
     * 
     * @param start_pt 
     * @param start_vel 
     * @param start_acc 
     * @param local_target_pos 
     * @param local_target_vel 
     * @param flag_polyInit         IF TRUE, Initialize new polynomial trajectory, ELSE initialize from previous trajectory
     * @param flag_randomPolyTraj   Randomize the inner points of the polynomial trajectory
     * @param t_seg_dur             Single value representing duration of each segment, assuming they are all equal
     * @param initMJO 
     * @return true 
     * @return false 
     */
    bool computeInitState(
        const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc, 
        const Eigen::Vector3d &local_target_pos, const Eigen::Vector3d &local_target_vel, 
        const bool flag_polyInit, const bool flag_randomPolyTraj, 
        const double &t_seg_dur, poly_traj::MinJerkOpt &initMJO);

    /**
     * @brief Given boundary conditions, plan a path and optimize it given user-defined objectives
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
     * @brief Ignoring obstacles, generate a minimum jerk trajectory given boundary conditions and waypoints
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
        poly_traj::MinJerkOpt& globalMJO,
        const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc, 
        const std::vector<Eigen::Vector3d> &waypoints,
        const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);

    /**
     * @brief Get local goal (position and velocity) based on planning horizon 
     * 
     * @param planning_horizon 
     * @param start_pt 
     * @param global_end_pt 
     * @param local_target_pos 
     * @param local_target_vel 
     * @param touch_goal 
     */
    void getLocalTarget(
        const double planning_horizon,
        const Eigen::Vector3d &start_pt, const Eigen::Vector3d &global_end_pt,
        Eigen::Vector3d &local_target_pos, Eigen::Vector3d &local_target_vel,
        bool &touch_goal);

    bool EmergencyStop(Eigen::Vector3d stop_pos);

    bool checkCollision(int drone_id);

    /**
     * @brief Set the local trajectory from a given minimum jerk trajectory 
     * 
     * @param opt Minimum Jerk trajectory optimizer object
     * @param touch_goal If goal is reached
     * @return true 
     * @return false 
     */
    bool setLocalTrajFromOpt(const poly_traj::MinJerkOpt &opt, const bool touch_goal);
    
    double getSwarmClearance(void) { 
      return ploy_traj_opt_->get_swarm_clearance_(); 
    }

    /**
     * @brief Return Number of constraint points per piece
     * 
     * @return int Number of constraint points per piece
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