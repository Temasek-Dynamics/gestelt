#ifndef _PLAN_CONTAINER_H_
#define _PLAN_CONTAINER_H_

#include <Eigen/Eigen>
#include <vector>
#include <ros/ros.h>

#include <optimizer/poly_traj_utils.hpp>

using std::vector;

namespace ego_planner
{
  // Outer idx is id of constraint piece, inner idx is constraint points with individual pairs (time, position)
  typedef std::vector<std::vector<std::pair<double, Eigen::Vector3d>>> PtsChk_t;

  struct GlobalTrajData
  {
    poly_traj::Trajectory traj;
    double duration; // Total duration of trajectory
    double global_start_time; // Actual time at which global trajectory is set
    double glb_t_of_lc_tgt; // Actual time at which local target has been set
    double last_glb_t_of_lc_tgt; // Actual time at which previous local target is set 
  };

  struct LocalTrajData
  {
    poly_traj::Trajectory traj;
    PtsChk_t pts_chk;
    int drone_id; // A negative value indicates no received trajectories.
    int traj_id; // Trajectory id
    double duration; // Total duration of trajectory
    double start_time; // world time
    double end_time;   // world time
    Eigen::Vector3d start_pos; // starting position
  };

  typedef std::vector<LocalTrajData> SwarmTrajData;

  class TrajContainer
  {
  public:
    GlobalTrajData global_traj; // Global trajectory from start to actual goal
    LocalTrajData local_traj; // Local trajectory from start to local target/goal
    SwarmTrajData swarm_traj;

    TrajContainer()
    {
      local_traj.traj_id = 0;
    }
    ~TrajContainer() {}

    void setGlobalTraj(const poly_traj::Trajectory &trajectory, const double &world_time)
    {
      global_traj.traj = trajectory;
      global_traj.duration = trajectory.getTotalDuration();
      global_traj.global_start_time = world_time;
      global_traj.glb_t_of_lc_tgt = world_time;
      global_traj.last_glb_t_of_lc_tgt = -1.0;

      local_traj.drone_id = -1; //set to negative to indicate no received trajectories from other drones yet
      local_traj.duration = 0.0;
      local_traj.traj_id = 0;
    }

    void setLocalTraj(const poly_traj::Trajectory &trajectory, const PtsChk_t &pts_to_chk, const double &world_time, const int drone_id = -1)
    {
      local_traj.drone_id = drone_id;
      local_traj.traj_id++;
      local_traj.duration = trajectory.getTotalDuration();
      local_traj.start_pos = trajectory.getJuncPos(0);
      local_traj.start_time = world_time;
      local_traj.traj = trajectory;
      local_traj.pts_chk = pts_to_chk;
    }

    /**
     * @brief Get a sampled global trajectory for visualization
     * 
     * @param t_step time step (s) to sample trajectory
     * @return std::vector<Eigen::Vector3d> 
     */
    std::vector<Eigen::Vector3d> getGlobalTrajViz(const double t_step)
    {
      int num_sampled_pts = floor(global_traj.duration / t_step);
      std::vector<Eigen::Vector3d> global_traj_viz(num_sampled_pts);

      for (int i = 0; i < num_sampled_pts; i++)
      {
        global_traj_viz[i] = global_traj.traj.getPos(i * t_step);
      }

      return global_traj_viz;
    }

  };

  struct PlanParameters
  {
    /* planning algorithm parameters */
    double max_vel_, max_acc_;     // physical limits
    double polyTraj_piece_length;  // Length of each segment
    double feasibility_tolerance_; // permitted ratio of vel/acc exceeding limits
    double planning_horizon_; // Distance to plan ahead
    bool use_distinctive_trajs; // If true, enables planning of multiple distinctive trajectories
    bool touch_goal; // Local target is global target
    // Denotes if it is a single drone or a swarm
    int drone_id; // single drone: drone_id <= 0, swarm: drone_id >= 1

    /* processing time */
    double time_search_ = 0.0;
    double time_optimize_ = 0.0;
    double time_adjust_ = 0.0;
  };

} // namespace ego_planner

#endif // _PLAN_CONTAINER_H_