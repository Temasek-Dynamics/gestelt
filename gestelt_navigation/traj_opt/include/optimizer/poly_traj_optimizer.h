#ifndef _POLY_TRAJ_OPTIMIZER_H_
#define _POLY_TRAJ_OPTIMIZER_H_

#include <Eigen/Eigen>
#include <ros/ros.h>
#include <grid_map/grid_map.h>
#include <path_searching/dyn_a_star.h>
#include <traj_utils/plan_container.hpp>
#include <traj_utils/planning_visualization.h>
#include "optimizer/lbfgs.hpp"
#include "poly_traj_utils.hpp"

namespace ego_planner
{

  class ConstraintPoints
  {
  public:
    int cp_size; // deformation points
    Eigen::MatrixXd points;
    std::vector<std::vector<Eigen::Vector3d>> base_point; // p of the {p,v} pair. The point at the start of the direction vector (collision point)
    std::vector<std::vector<Eigen::Vector3d>> direction;  // v of the {p,v} pair. Direction vector, must be normalized.
    std::vector<bool> flag_temp;                          // A flag that used in many places. Initialize it everytime before using it.

    void resize_cp(const int size_set)
    {
      cp_size = size_set;

      base_point.clear();
      direction.clear();
      flag_temp.clear();

      points.resize(3, size_set);
      base_point.resize(cp_size);
      direction.resize(cp_size);
      flag_temp.resize(cp_size);
    }

    void segment(ConstraintPoints &buf, const int start, const int end)
    {
      if (start < 0 || end >= cp_size || points.rows() != 3)
      {
        ROS_ERROR("Wrong segment index! start=%d, end=%d", start, end);
        return;
      }

      buf.resize_cp(end - start + 1);
      buf.points = points.block(0, start, 3, end - start + 1);
      buf.cp_size = end - start + 1;
      for (int i = start; i <= end; i++)
      {
        buf.base_point[i - start] = base_point[i];
        buf.direction[i - start] = direction[i];
      }
    }

    /**
     * @brief Return idx of the 2/3 waypoint
     * 
     * @param points 
     * @param touch_goal 
     * @return int 
     */
    static int two_thirds_id(Eigen::MatrixXd &points, const bool touch_goal)
    {
      return touch_goal ? points.cols() - 1 : points.cols() - 1 - (points.cols() - 2) / 3;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  }; // class ConstraintPoints

  class PolyTrajOptimizer
  {
  public:
    typedef std::unique_ptr<PolyTrajOptimizer> Ptr;

  private:
    std::shared_ptr<GridMap> grid_map_;
    AStar::Ptr a_star_;
    PlanningVisualization::Ptr visualization_;

    poly_traj::MinJerkOpt jerkOpt_;

    SwarmTrajData *swarm_trajs_{NULL}; // Can not use shared_ptr and no need to free
    std::shared_ptr<std::unordered_map<int, ego_planner::LocalTrajData>> swarm_minco_trajs_; // Swarm MINCO trajectories

    ConstraintPoints cps_;
    // PtsChk_t pts_check_;

    int drone_id_;            // ID of drone
    int cps_num_perPiece_;   // number of distinctive constraint points per piece
    int variable_num_;       // optimization variables
    int piece_num_;          // poly traj piece numbers
    int iter_num_;           // iteration of the solver
    double min_ellip_dist2_; // min trajectory distance in swarm
    bool touch_goal_;

    enum FORCE_STOP_OPTIMIZE_TYPE
    {
      DONT_STOP,
      STOP_FOR_REBOUND,
      STOP_FOR_ERROR
    } force_stop_type_;

    /* optimization parameters */
    double wei_obs_, wei_obs_soft_;                               // obstacle weight
    double wei_swarm_;                                            // swarm weight
    double wei_feas_;                                             // feasibility weight
    double wei_sqrvar_;                                           // squared variance weight
    double wei_time_;                                             // time weight
    double obs_clearance_, obs_clearance_soft_, swarm_clearance_; // safe distance
    double max_vel_, max_acc_;                                    // dynamic limits

    double t_now_;

    std::vector<double> spheres_radius_;                // Vector of sphere radius, size is no. of segments/pieces
    std::vector<Eigen::Vector3d> spheres_center_; // Vector of sphere centers, size is no. of segments/pieces
    Eigen::MatrixXd inner_cstr_pts_; // inner constraint points of trajectory (excludes boundary points), this is finer than the inner CONTROL points

  public:
    PolyTrajOptimizer(){}
    ~PolyTrajOptimizer() {}

    enum CHK_RET
    {
      OBS_FREE,
      ERR,
      FINISH
    };

    /* set variables */
    void setParam(ros::NodeHandle &pnh);
    void setEnvironment(const std::shared_ptr<GridMap> &map);
    void setVisualizer(PlanningVisualization::Ptr vis);
    void setSwarmTrajs(SwarmTrajData *swarm_trajs_ptr);
    void setDroneId(const int drone_id);
    void setIfTouchGoal(const bool touch_goal);

    void assignSwarmTrajs(std::shared_ptr<std::unordered_map<int, ego_planner::LocalTrajData>>& swarm_minco_trajs);

    /**
     * Returns the minimum jerk optimizer object
    */
    const poly_traj::MinJerkOpt &getMinJerkOpt(void) { return jerkOpt_; }

    /**
     * @brief Get the parameter value for number of constraint points per piece.
     * This is user-defined
     * 
     * @return int 
     */
    int get_cps_num_perPiece_(void) { return cps_num_perPiece_; }

    /**
     * @brief Get the user-defined swarm clearance parameter
     * 
     * @return double 
     */
    double get_swarm_clearance_(void) { return swarm_clearance_; }

    /* main planning API */

    /**
     * @brief Optimize a trajectory given boundary conditions, inner points and segment durations.
     * 
     * @param iniState 
     * @param finState 
     * @param inner_ctrl_pts 
     * @param initT 
     * @param spheres_radius 
     * @param spheres_centers 
     * @param final_cost 
     * @return true 
     * @return false 
     */
    bool optimizeTrajectorySFC(const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
                            const Eigen::MatrixXd &inner_ctrl_pts, const Eigen::VectorXd &initT,
                            const std::vector<double>& spheres_radius,
                            const std::vector<Eigen::Vector3d>& spheres_centers,
                            double &final_cost);

    /**
     * @brief Optimize a trajectory given boundary conditions, inner points and segment durations.
     * 
     * 
     * @param iniState Initial state
     * @param finState Final state 
     * @param inner_ctrl_pts Inner control points (less fine than constraint points)
     * @param initT Time duration at each point
     * @param initial_cstr_pts Initial constraint points (more fine than control points)
     * @param final_cost 
     * @return true 
     * @return false 
     */
    bool optimizeTrajectory(const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
                            const Eigen::MatrixXd &inner_ctrl_pts, const Eigen::VectorXd &initT,
                            Eigen::MatrixXd &initial_cstr_pts, double &final_cost)
    {
      // IF size of inner points and segment durations are not the same, there is a bug
      if (inner_ctrl_pts.cols() != (initT.size() - 1))
      {
        ROS_ERROR("[PolyTrajOptimizer::optimizeTrajectory] inner_cstr_pts.cols() != (initT.size()-1)");
        return false;
      }

      piece_num_ = initT.size();

      jerkOpt_.reset(iniState, finState, piece_num_);

      // Number of coefficients
      variable_num_ = 4 * (piece_num_ - 1) + 1;

      ros::Time t0 = ros::Time::now(), t1, t2;
      int restart_nums = 0, rebound_times = 0;
      bool flag_force_return, flag_still_occ, flag_success;

      double x_init[variable_num_];
      // copy the inner points to x_init
      memcpy(x_init, inner_ctrl_pts.data(), inner_ctrl_pts.size() * sizeof(x_init[0]));
      Eigen::Map<Eigen::VectorXd> Vt(x_init + inner_ctrl_pts.size(), initT.size()); // Virtual Time

      // Convert from real time to virtual time
      RealT2VirtualT(initT, Vt);

      lbfgs::lbfgs_parameter_t lbfgs_params;
      lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
      lbfgs_params.mem_size = 16;
      lbfgs_params.max_iterations = 200;
      // lbfgs_params.g_epsilon = 0.1;
      lbfgs_params.min_step = 1e-32;
      // lbfgs_params.abs_curv_cond = 0;
      lbfgs_params.past = 3;
      lbfgs_params.delta = 1.0e-3;
      do
      {
        /* ---------- prepare ---------- */
        iter_num_ = 0;
        flag_force_return = false;
        force_stop_type_ = DONT_STOP;
        flag_still_occ = false;
        flag_success = false;
        t_now_ = ros::Time::now().toSec();

        /* ---------- optimize ---------- */
        t1 = ros::Time::now();
        int result = lbfgs::lbfgs_optimize(
            variable_num_,                  // The number of variables
            x_init,                         // The array of variables.
            &final_cost,                    // The pointer to the variable that receives the final value of the objective function for the variables
            PolyTrajOptimizer::costFunctionCallback, // The callback function to provide function and gradient evaluations given a current values of variables
            NULL,                           //  The callback function to provide values of the upperbound of the stepsize to search in, provided with the beginning values of variables before the linear search, and the current step vector (can be negative gradient).
            PolyTrajOptimizer::earlyExitCallback,    // The callback function to receive the progress (the number of iterations, the current value of the objective function) of the minimization process.
            this,                           // A user data for the client program. The callback functions will receive the value of this argument.
            &lbfgs_params);                 // The pointer to a structure representing parameters for L-BFGS optimization. A client program can set this parameter to NULL to use the default parameters

        t2 = ros::Time::now();
        double time_ms = (t2 - t1).toSec() * 1000;
        // double total_time_ms = (t2 - t0).toSec() * 1000;

        /* ---------- get result and check collision ---------- */

        // IF converged, maxmimum_iteration, already minimized or stop
        if (result == lbfgs::LBFGS_CONVERGENCE ||
            result == lbfgs::LBFGSERR_MAXIMUMITERATION ||
            result == lbfgs::LBFGS_ALREADY_MINIMIZED ||
            result == lbfgs::LBFGS_STOP)
        {
          flag_force_return = false;

          /* Check for collisions in the path */
          std::vector<std::pair<int, int>> segments_nouse;
          if (min_ellip_dist2_ > pow((swarm_clearance_ * 1.25), 2) &&
              finelyCheckAndSetConstraintPoints(segments_nouse, jerkOpt_, false) == CHK_RET::OBS_FREE)
          {
            flag_success = true;
            // printf("\033[32miter=%d,time(ms)=%5.3f,total_t(ms)=%5.3f,cost=%5.3f\n\033[0m", iter_num_, time_ms, total_time_ms, final_cost);
          }
          else
          {
            // Path in collision with obstacle, restart optimization
            flag_still_occ = true;
            restart_nums++;
            // printf("\033[32miter=%d,time(ms)=%5.3f, fine check collided, keep optimizing\n\033[0m", iter_num_, time_ms);
          }
        }
        // ELSE IF Cancelled
        else if (result == lbfgs::LBFGSERR_CANCELED)
        {
          flag_force_return = true;
          rebound_times++; 
          // std::cout << "iter=" << iter_num_ << ",time(ms)=" << time_ms << ",rebound." <<std::endl;
        }
        // ELSE ERROR
        else
        {
          std::cout << "iter=" << iter_num_ << ",time(ms)=" << time_ms << ",error." <<std::endl;
          ROS_WARN("[PolyTrajOptimizer::optimizeTrajectory] Solver error. Return = %d, %s. Skip this planning.", result, lbfgs::lbfgs_strerror(result));
        }

      } while (
          (flag_still_occ && restart_nums < 3) ||
          (flag_force_return && force_stop_type_ == STOP_FOR_REBOUND && rebound_times <= 20));

      initial_cstr_pts = cps_.points; 

      return flag_success;
    }

    /**
     * @brief Get points along trajectory to check
     * 
     * @param traj Trajectory to check
     * @param num_pieces number of pieces to check
     * @param pts_check Vector of points to check
     * @return true 
     * @return false 
     */
    bool computePointsToCheck(poly_traj::Trajectory &traj, int num_pieces, PtsChk_t &pts_check)
    {
      pts_check.clear();
      pts_check.resize(num_pieces);
      const double RES = grid_map_->getResolution();
      const double RES_2 = RES / 2;

      Eigen::VectorXd durations = traj.getDurations();
      Eigen::VectorXd t_seg_start(durations.size() + 1); // Store starting time of each segment
      t_seg_start(0) = 0;
      for (int i = 0; i < durations.size(); ++i){
        t_seg_start(i + 1) = t_seg_start(i) + durations(i);
      }
      const double TOTAL_TRAJ_DURATION = durations.sum(); // total duration of trajectory
      double t = 0.0, t_step = RES / max_vel_;
      Eigen::Vector3d pt_last = traj.getPos(0.0);
      // pts_check[0].push_back(pt_last);
      int id_cps_curr = 0, id_piece_curr = 0;

      while (true)
      {
        if (t > TOTAL_TRAJ_DURATION) // Total trajectory duration is negative
        {
          if (touch_goal_ && pts_check.size() > 0)
          {
            return true;
          }
          else
          {
            ROS_ERROR("drone %d: [PolyTrajOptimizer::computePointsToCheck] Failed to get points list to check. touch_goal_=%d, pts_check.size()=%d", drone_id_, touch_goal_, (int)pts_check.size());
            pts_check.clear();
            return false;
          }
        }

        const double next_t_stp = t_seg_start(id_piece_curr) + durations(id_piece_curr) / cps_num_perPiece_ * ((id_cps_curr + 1) - cps_num_perPiece_ * id_piece_curr);
        if (t >= next_t_stp)
        {
          if (id_cps_curr + 1 >= cps_num_perPiece_ * (id_piece_curr + 1))
          {
            ++id_piece_curr;
            // std::cout << "id_piece_curr=" << id_piece_curr <<std::endl;
            // std::cout << "traj.getPieceSize()=" << traj.getPieceSize() <<std::endl;
          }
          // if end of path, break
          if (++id_cps_curr >= num_pieces)
          {
            break;
          }
        }

        // std::cout << "pts_check.size()" << pts_check.size() << " id_cps_curr=" << id_cps_curr <<std::endl;
        Eigen::Vector3d pt = traj.getPos(t);

        // IF time is close to 0, 
        //    OR size of current point check array is 0
        //    OR distance between last and current point is more than map resolution/2
        if (t < 1e-5 || pts_check[id_cps_curr].size() == 0 || (pt - pt_last).cwiseAbs().maxCoeff() > RES_2)
        {
          pts_check[id_cps_curr].emplace_back(std::pair<double, Eigen::Vector3d>(t, pt));
          pt_last = pt;
        }

        t += t_step;
      }

      return true;
    }

    /**
     * @brief Check for collision along path and set {p,v} pairs to constraint points
     * 
     */
    CHK_RET finelyCheckAndSetConstraintPoints(std::vector<std::pair<int, int>> &segments,
                                              const poly_traj::MinJerkOpt &mjo,
                                              const bool flag_first_init /*= true*/)
  {

    Eigen::MatrixXd init_points = mjo.getInitConstraintPoints(cps_num_perPiece_);
    poly_traj::Trajectory traj = mjo.getTraj();

    if (flag_first_init)
    {
      cps_.resize_cp(init_points.cols());
      cps_.points = init_points;
    }

    /*** Segment the initial trajectory according to obstacles ***/

    vector<std::pair<int, int>> segment_ids; // Vector of (seg_start_idx, seg_end_idx)
    constexpr int ENOUGH_INTERVAL = 2; // Threshold for repeated occupancy values
    // double step_size = grid_map_->getResolution() / ((init_points.col(0) - init_points.rightCols(1)).norm() / (init_points.cols() - 1)) / 1.5;
    int seg_start_idx = -1, seg_end_idx = -1;
    int same_occ_state_times = ENOUGH_INTERVAL + 1;
    bool occ, last_occ = false;
    bool flag_start_of_seg = false, flag_end_of_seg = false;
    // only check closed 2/3 points.
    int i_end = ConstraintPoints::two_thirds_id(init_points, touch_goal_); // number of pieces to check

    // Get points along trajectory to check for collision
    PtsChk_t pts_check; // Vector of constraint pieces. Within each piece is a vector of (timestamp, point position).
    if (!computePointsToCheck(traj, i_end, pts_check))
    {
      return CHK_RET::ERR;
    }

    // Split trajectory into segments based on whether they are in collision or not
    for (int i = 0; i < i_end; ++i) // for each piece 
    {
      for (size_t j = 0; j < pts_check[i].size(); ++j) // for each point to check
      {
        // get occupancy value at position
        occ = grid_map_->getInflateOccupancy(pts_check[i][j].second); 

        // Case 1: START OF NEW SEGMENT. IF occupied and was not previously occupied
        if (occ && !last_occ)
        {
          if (same_occ_state_times > ENOUGH_INTERVAL || i == 0)
          {
            seg_start_idx = i;
            flag_start_of_seg = true;
          }
          same_occ_state_times = 0; 
        }
        // Case 2: END OF CURRENT SEGMENT. IF not occupied and was previously occupied
        else if (!occ && last_occ)
        {
          seg_end_idx = i + 1;

          // If end of current segment AND (segment is large enough OR end of entire path is reached)
          if (same_occ_state_times > ENOUGH_INTERVAL || (i == i_end - 1))
          {
            flag_end_of_seg = true;
          }

          same_occ_state_times = 0;
        }
        // Case 3: IF NO CHANGE in occupancy: Continue current segment
        else
        {
          ++same_occ_state_times;
        }

        last_occ = occ;

        if (flag_start_of_seg && flag_end_of_seg)
        {
          // Reset flags
          flag_start_of_seg = false;
          flag_end_of_seg = false;
          if (seg_start_idx < 0 || seg_end_idx < 0)
          {
            ROS_ERROR("UAV_%d: [PolyTrajOptimizer::finelyCheckAndSetConstraintPoints] Should not happen! seg_start_idx=%d, seg_end_idx=%d", drone_id_, seg_start_idx, seg_end_idx);
            return CHK_RET::ERR;
          }
          segment_ids.push_back(std::pair<int, int>(seg_start_idx, seg_end_idx));
        }
      }
    }

    /* ALL SEGMENTS ARE COLLISION FREE, return in advance */
    if (segment_ids.size() == 0)
    {
      return CHK_RET::OBS_FREE;
    }

    vector<vector<Eigen::Vector3d>> a_star_paths;
    // For each segement, generate a collision free a_star path
    for (size_t i = 0; i < segment_ids.size(); ++i)
    {
      // Search from back to head
      Eigen::Vector3d pt_a(init_points.col(segment_ids[i].second)), pt_b(init_points.col(segment_ids[i].first));
      ASTAR_RET ret = a_star_->AstarSearch(/*(pt_a-pt_b).norm()/10+0.05*/ grid_map_->getResolution(), pt_a, pt_b);
      if (ret == ASTAR_RET::SUCCESS)
      {
        a_star_paths.push_back(a_star_->getPath());
      }
      else if (ret == ASTAR_RET::SEARCH_ERR && i + 1 < segment_ids.size()) // connect the next segment
      {
        // Concatenate the current segment to the next segment (Longer path)
        // Perform Astar search on the concatenated path  
        segment_ids[i].second = segment_ids[i + 1].second;
        segment_ids.erase(segment_ids.begin() + i + 1);
        --i;
        ROS_WARN("Drone%d: [PolyTrajOptimizer::finelyCheckAndSetConstraintPoints] Concatenated 2 segments and performing A* search again.", drone_id_);
      }
      else
      {
        ROS_ERROR("Drone%d: [PolyTrajOptimizer::finelyCheckAndSetConstraintPoints] A-star error, force return!", drone_id_);
        return CHK_RET::ERR;
      }
    }

    /*** calculate bounds ***/
    int id_low_bound, id_up_bound;
    vector<std::pair<int, int>> bounds(segment_ids.size());
    for (size_t i = 0; i < segment_ids.size(); i++) // For each segment
    {

      if (i == 0) // first segment
      {
        id_low_bound = 1;
        if (segment_ids.size() > 1)
        {
          id_up_bound = (int)(((segment_ids[0].second + segment_ids[1].first) - 1.0f) / 2); // id_up_bound : -1.0f fix()
        }
        else
        {
          id_up_bound = init_points.cols() - 2;
        }
      }
      else if (i == segment_ids.size() - 1) // last segment, i != 0 here
      {
        id_low_bound = (int)(((segment_ids[i].first + segment_ids[i - 1].second) + 1.0f) / 2); // id_low_bound : +1.0f ceil()
        id_up_bound = init_points.cols() - 2;
      }
      else
      {
        id_low_bound = (int)(((segment_ids[i].first + segment_ids[i - 1].second) + 1.0f) / 2); // id_low_bound : +1.0f ceil()
        id_up_bound = (int)(((segment_ids[i].second + segment_ids[i + 1].first) - 1.0f) / 2);  // id_up_bound : -1.0f fix()
      }

      bounds[i] = std::pair<int, int>(id_low_bound, id_up_bound);
    }

    /*** Adjust segment length ***/
    vector<std::pair<int, int>> adjusted_segment_ids(segment_ids.size());
    constexpr double MINIMUM_PERCENT = 0.0; // Each segment is guaranteed to have sufficient points to generate sufficient force
    int minimum_points = round(init_points.cols() * MINIMUM_PERCENT);
    for (size_t i = 0; i < segment_ids.size(); i++)
    {
      // /*** Adjust segment length ***/
      // int num_pts_in_seg = segment_ids[i].second - segment_ids[i].first + 1;
      // //cout << "i = " << i << " first = " << segment_ids[i].first << " second = " << segment_ids[i].second <<std::endl;

      // // If not enough points in segment
      // if (num_pts_in_seg < minimum_points)
      // {
      //   double add_points_each_side = (int)(((minimum_points - num_pts_in_seg) + 1.0f) / 2);

      //   adjusted_segment_ids[i].first = segment_ids[i].first - add_points_each_side >= bounds[i].first
      //                                       ? segment_ids[i].first - add_points_each_side
      //                                       : bounds[i].first;

      //   adjusted_segment_ids[i].second = segment_ids[i].second + add_points_each_side <= bounds[i].second
      //                                        ? segment_ids[i].second + add_points_each_side
      //                                        : bounds[i].second;
      // }
      // else // As per default
      // {
      //   adjusted_segment_ids[i] = segment_ids[i];
      // }

      adjusted_segment_ids[i] = segment_ids[i];
    }

    // For each segment, check for overlaps
    for (size_t i = 1; i < adjusted_segment_ids.size(); i++) // Avoid overlap
    {
      if (adjusted_segment_ids[i - 1].second >= adjusted_segment_ids[i].first)
      {
        double middle = (double)(adjusted_segment_ids[i - 1].second + adjusted_segment_ids[i].first) / 2.0;
        adjusted_segment_ids[i - 1].second = static_cast<int>(middle - 0.1);
        adjusted_segment_ids[i].first = static_cast<int>(middle + 1.1);
      }
    }

    // Used for return
    vector<std::pair<int, int>> final_segment_ids;

    /*** Assign data to each segment ***/
    for (size_t i = 0; i < segment_ids.size(); i++) // For each segment
    {
      // step 1: set cps_.flag_temp[j], this indicates if the segment has computed a {p,v} value
      for (int j = adjusted_segment_ids[i].first; j <= adjusted_segment_ids[i].second; ++j){
        cps_.flag_temp[j] = false;
      }

      // step 2: Obtain the {p,v} for each constraint point
      int got_intersection_id = -1;
      for (int j = segment_ids[i].first + 1; j < segment_ids[i].second; ++j) // For each point in the segment
      {
        Eigen::Vector3d ctrl_pts_law(init_points.col(j + 1) - init_points.col(j - 1));
        Eigen::Vector3d intersection_point; // The point from the current constraint point that is normal to the AStar path
        int Astar_id = a_star_paths[i].size() / 2; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
        int last_Astar_id; 
        // a_star_paths[i][Astar_id] is midway point in the A* path for the i-th segment.
        // val: value of cos(theta), where theta is the angle between ctrl_pts_law and (astar path midpoint - point in segment)
        double val = (a_star_paths[i][Astar_id] - init_points.col(j)).dot(ctrl_pts_law);
        double init_val = val;

        // Execute while loop until intersection point is found
        while (true)
        {
          last_Astar_id = Astar_id;

          if (val >= 0) // If theta <= 90 degrees 
          {
            ++Astar_id; // Go towards end of A* path
            if (Astar_id >= (int)a_star_paths[i].size()) // IF end of astar path
            {
              break;
            }
          }
          else // If theta > 90 degrees 
          {
            --Astar_id; // Go towards start of A* path
            if (Astar_id < 0) // IF start of A* path
            {
              break;
            }
          }

          val = (a_star_paths[i][Astar_id] - init_points.col(j)).dot(ctrl_pts_law);
          
          if (val * init_val <= 0 && (abs(val) > 0 || abs(init_val) > 0)) // val = init_val = 0.0 is not allowed
          {
            intersection_point =
                a_star_paths[i][Astar_id] +
                ((a_star_paths[i][Astar_id] - a_star_paths[i][last_Astar_id]) *
                 (ctrl_pts_law.dot(init_points.col(j) - a_star_paths[i][Astar_id]) / ctrl_pts_law.dot(a_star_paths[i][Astar_id] - a_star_paths[i][last_Astar_id])) // = t
                );

            got_intersection_id = j;
            break;
          }
        }
        
        // If intersection point exists
        if (got_intersection_id >= 0)
        {
          double length = (intersection_point - init_points.col(j)).norm();
          // Check that length is non-zero value
          if (length > 1e-5) 
          {
            cps_.flag_temp[j] = true;
            // Iterate from intersection point to current point
            // Obtain the {p,v} values for the constraint point
            for (double a = length; a >= 0.0; a -= grid_map_->getResolution())
            {
              bool occ = grid_map_->getInflateOccupancy((a / length) * intersection_point + (1 - a / length) * init_points.col(j));

              if (occ || a < grid_map_->getResolution())
              {
                if (occ){
                  a += grid_map_->getResolution();
                }
                cps_.base_point[j].push_back((a / length) * intersection_point + (1 - a / length) * init_points.col(j));
                cps_.direction[j].push_back((intersection_point - init_points.col(j)).normalized());
                break;
              }
            }
          }
          else
          {
            got_intersection_id = -1;
          }
        }
      }

      /* Edge case: the segment length is too short. 
      Here the control points may lie outside the A* path, leading to opposite gradient direction. So I have to take special care of it */
      if (segment_ids[i].second - segment_ids[i].first == 1)
      {
        Eigen::Vector3d ctrl_pts_law(init_points.col(segment_ids[i].second) - init_points.col(segment_ids[i].first)), intersection_point;
        Eigen::Vector3d middle_point = (init_points.col(segment_ids[i].second) + init_points.col(segment_ids[i].first)) / 2;
        int Astar_id = a_star_paths[i].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
        double val = (a_star_paths[i][Astar_id] - middle_point).dot(ctrl_pts_law), init_val = val;
        while (true)
        {

          last_Astar_id = Astar_id;

          if (val >= 0)
          {
            ++Astar_id; // Previous Astar search from back to head
            if (Astar_id >= (int)a_star_paths[i].size())
            {
              break;
            }
          }
          else
          {
            --Astar_id;
            if (Astar_id < 0)
            {
              break;
            }
          }

          val = (a_star_paths[i][Astar_id] - middle_point).dot(ctrl_pts_law);

          if (val * init_val <= 0 && (abs(val) > 0 || abs(init_val) > 0)) // val = init_val = 0.0 is not allowed
          {
            intersection_point =
                a_star_paths[i][Astar_id] +
                ((a_star_paths[i][Astar_id] - a_star_paths[i][last_Astar_id]) *
                 (ctrl_pts_law.dot(middle_point - a_star_paths[i][Astar_id]) / ctrl_pts_law.dot(a_star_paths[i][Astar_id] - a_star_paths[i][last_Astar_id])) // = t
                );

            if ((intersection_point - middle_point).norm() > 0.01) // 1cm.
            {
              cps_.flag_temp[segment_ids[i].first] = true;
              cps_.base_point[segment_ids[i].first].push_back(init_points.col(segment_ids[i].first));
              cps_.direction[segment_ids[i].first].push_back((intersection_point - middle_point).normalized());

              got_intersection_id = segment_ids[i].first;
            }
            break;
          }
        }
      }

      // step 3: For remaining constraint points with no {p,v}, compute the {p,v} values 
      if (got_intersection_id >= 0)
      {
        // From intersection point idx to end of adjusted segment, compute {p,v}
        for (int j = got_intersection_id + 1; j <= adjusted_segment_ids[i].second; ++j){
         if (!cps_.flag_temp[j])
          {
            cps_.base_point[j].push_back(cps_.base_point[j - 1].back());
            cps_.direction[j].push_back(cps_.direction[j - 1].back());
          }
        }
 
        // From intersection point idx to start of adjusted segment, compute {p,v}
        for (int j = got_intersection_id - 1; j >= adjusted_segment_ids[i].first; --j){
          if (!cps_.flag_temp[j])
          {
            cps_.base_point[j].push_back(cps_.base_point[j + 1].back());
            cps_.direction[j].push_back(cps_.direction[j + 1].back());
          }
        }

        final_segment_ids.push_back(adjusted_segment_ids[i]);
      }
      else
      {
        // Just ignore, it does not matter ^_^.
        // ROS_ERROR("Failed to generate direction! segment_id=%d", i);
      }
    }

    visualization_->displayAStarList(a_star_paths, 0);

    segments = final_segment_ids;
    return CHK_RET::FINISH;
  }

    bool roughlyCheckConstraintPoints(void)  
    {
      // int end_idx = cps_.cp_size - 1;

      /*** Check and segment the initial trajectory according to obstacles ***/
      int seg_start_idx, seg_end_idx;
      vector<std::pair<int, int>> segment_ids;
      bool flag_new_obs_valid = false;
      int i_end = ConstraintPoints::two_thirds_id(cps_.points, touch_goal_); // only check closed 2/3 points.
      for (int i = 1; i <= i_end; ++i)
      {

        bool occ = grid_map_->getInflateOccupancy(cps_.points.col(i));

        /*** check if the new collision will be valid ***/
        if (occ)
        {
          for (size_t k = 0; k < cps_.direction[i].size(); ++k)
          {
            if ((cps_.points.col(i) - cps_.base_point[i][k]).dot(cps_.direction[i][k]) < 1 * grid_map_->getResolution()) // current point is outside all the collision_points.
            {
              occ = false;
              break;
            }
          }
        }

        if (occ)
        {
          flag_new_obs_valid = true;

          int j;
          for (j = i - 1; j >= 0; --j)
          {
            occ = grid_map_->getInflateOccupancy(cps_.points.col(j));
            if (!occ)
            {
              seg_start_idx = j;
              break;
            }
          }
          if (j < 0) // fail to get the obs free point
          {
            ROS_ERROR("UAV_%d: ERROR! the drone is in obstacle. It means a crash in real-world.", drone_id_);
            seg_start_idx = 0;
          }

          for (j = i + 1; j < cps_.cp_size; ++j)
          {
            occ = grid_map_->getInflateOccupancy(cps_.points.col(j));

            if (!occ)
            {
              seg_end_idx = j;
              break;
            }
          }
          if (j >= cps_.cp_size) // fail to get the obs free point
          {
            ROS_WARN("UAV_%d: WARN! terminal point of the current trajectory is in obstacle, skip this planning.", drone_id_);

            force_stop_type_ = STOP_FOR_ERROR;
            return false;
          }

          i = j + 1;

          segment_ids.push_back(std::pair<int, int>(seg_start_idx, seg_end_idx));
        }
      }

      if (flag_new_obs_valid)
      {
        vector<vector<Eigen::Vector3d>> a_star_paths;
        for (size_t i = 0; i < segment_ids.size(); ++i)
        {
          /*** a star search ***/
          Eigen::Vector3d pt_a(cps_.points.col(segment_ids[i].second)), pt_b(cps_.points.col(segment_ids[i].first));
          ASTAR_RET ret = a_star_->AstarSearch(/*(pt_a-pt_b).norm()/10+0.05*/ grid_map_->getResolution(), pt_a, pt_b);
          if (ret == ASTAR_RET::SUCCESS)
          {
            a_star_paths.push_back(a_star_->getPath());
          }
          else if (ret == ASTAR_RET::SEARCH_ERR && i + 1 < segment_ids.size()) // connect the next segment
          {
            segment_ids[i].second = segment_ids[i + 1].second;
            segment_ids.erase(segment_ids.begin() + i + 1);
            --i;
            ROS_WARN("A conor case 2, I have never exeam it.");
          }
          else
          {
            ROS_ERROR("A-star error");
            segment_ids.erase(segment_ids.begin() + i);
            --i;
          }
        }

        for (size_t i = 1; i < segment_ids.size(); i++) // Avoid overlap
        {
          if (segment_ids[i - 1].second >= segment_ids[i].first)
          {
            double middle = (double)(segment_ids[i - 1].second + segment_ids[i].first) / 2.0;
            segment_ids[i - 1].second = static_cast<int>(middle - 0.1);
            segment_ids[i].first = static_cast<int>(middle + 1.1);
          }
        }

        /*** Assign parameters to each segment ***/
        for (size_t i = 0; i < segment_ids.size(); ++i)
        {
          // step 1
          for (int j = segment_ids[i].first; j <= segment_ids[i].second; ++j)
            cps_.flag_temp[j] = false;

          // step 2
          int got_intersection_id = -1;
          for (int j = segment_ids[i].first + 1; j < segment_ids[i].second; ++j)
          {
            Eigen::Vector3d ctrl_pts_law(cps_.points.col(j + 1) - cps_.points.col(j - 1)), intersection_point;
            int Astar_id = a_star_paths[i].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
            double val = (a_star_paths[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law), init_val = val;
            while (true)
            {

              last_Astar_id = Astar_id;

              if (val >= 0)
              {
                ++Astar_id; // Previous Astar search from back to head
                if (Astar_id >= (int)a_star_paths[i].size())
                {
                  break;
                }
              }
              else
              {
                --Astar_id;
                if (Astar_id < 0)
                {
                  break;
                }
              }

              val = (a_star_paths[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law);

              if (val * init_val <= 0 && (abs(val) > 0 || abs(init_val) > 0)) // val = init_val = 0.0 is not allowed
              {
                intersection_point =
                    a_star_paths[i][Astar_id] +
                    ((a_star_paths[i][Astar_id] - a_star_paths[i][last_Astar_id]) *
                    (ctrl_pts_law.dot(cps_.points.col(j) - a_star_paths[i][Astar_id]) / ctrl_pts_law.dot(a_star_paths[i][Astar_id] - a_star_paths[i][last_Astar_id])) // = t
                    );

                got_intersection_id = j;
                break;
              }
            }

            if (got_intersection_id >= 0)
            {
              double length = (intersection_point - cps_.points.col(j)).norm();
              if (length > 1e-5)
              {
                cps_.flag_temp[j] = true;
                for (double a = length; a >= 0.0; a -= grid_map_->getResolution())
                {
                  bool occ = grid_map_->getInflateOccupancy((a / length) * intersection_point + (1 - a / length) * cps_.points.col(j));

                  if (occ || a < grid_map_->getResolution())
                  {
                    if (occ)
                      a += grid_map_->getResolution();
                    cps_.base_point[j].push_back((a / length) * intersection_point + (1 - a / length) * cps_.points.col(j));
                    cps_.direction[j].push_back((intersection_point - cps_.points.col(j)).normalized());
                    break;
                  }
                }
              }
              else
              {
                got_intersection_id = -1;
              }
            }
          }

          //step 3
          if (got_intersection_id >= 0)
          {
            for (int j = got_intersection_id + 1; j <= segment_ids[i].second; ++j)
              if (!cps_.flag_temp[j])
              {
                cps_.base_point[j].push_back(cps_.base_point[j - 1].back());
                cps_.direction[j].push_back(cps_.direction[j - 1].back());
              }

            for (int j = got_intersection_id - 1; j >= segment_ids[i].first; --j)
              if (!cps_.flag_temp[j])
              {
                cps_.base_point[j].push_back(cps_.base_point[j + 1].back());
                cps_.direction[j].push_back(cps_.direction[j + 1].back());
              }
          }
          else{
            ROS_WARN("[Poly Traj Optimizer] Failed to generate direction. It doesn't matter.");
          }
        }

        force_stop_type_ = STOP_FOR_REBOUND;

        // visualization_->displayAStarList(a_star_paths, 0);
        
        return true;
      }

      return false;
    }

  private:

    /* Optimizer callbacks */

    /**
     * @brief The LBFGS callback function to provide function and gradient evaluations given a current values of variables
     * 
     * @param func_data The user data sent for lbfgs_optimize() function by the client.
     * @param x         The current values of variables.
     * @param grad      The gradient vector. The callback function must compute the gradient values for the current variables.
     * @param n         The number of variables.
     * @return double   The value of the objective function for the current
     *                          variables.
     */
    static double costFunctionCallbackSFC(void *func_data, const double *x, double *grad, const int n);

    /**
     * @brief The LBFGS callback function to receive the progress (the number of iterations, the current value of the objective function) of the minimization process.
     * 
     * @param func_data 
     * @param x 
     * @param g 
     * @param fx 
     * @param xnorm 
     * @param gnorm 
     * @param step 
     * @param n 
     * @param k 
     * @param ls 
     * @return int 
     */
    static int earlyExitCallback(void *func_data, const double *x, const double *g,
                                 const double fx, const double xnorm, const double gnorm,
                                 const double step, int n, int k, int ls);

    /* mappings between real world time and unconstrained virtual time */
    template <typename EIGENVEC>
    void RealT2VirtualT(const Eigen::VectorXd &RT, EIGENVEC &VT);

    template <typename EIGENVEC>
    void VirtualT2RealT(const EIGENVEC &VT, Eigen::VectorXd &RT);

    template <typename EIGENVEC, typename EIGENVECGD>
    void VirtualTGradCost(const Eigen::VectorXd &RT, const EIGENVEC &VT,
                          const Eigen::VectorXd &gdRT, EIGENVECGD &gdVT,
                          double &costT);

    /* gradient and cost evaluation functions */
    template <typename EIGENVEC>
    void initAndGetSmoothnessGradCost2PT(EIGENVEC &gdT, double &cost);

    /**
     * @brief Get cost for constraints on PVA
     * 
     * @tparam EIGENVEC 
     * @param gdT Gradient of size of number of pieces
     * @param costs a vector of costs
     * @param K Constraint points per piece, or total sample number
     */
    template <typename EIGENVEC>
    void addPVAGradCost2CT_SFC(EIGENVEC &gdT, Eigen::VectorXd &costs, const int &K);

    /**
     * @brief Cost of swarm 
     * 
     * @param idx_cp 
     * @param t 
     * @param p 
     * @param v 
     * @param gradp 
     * @param gradt 
     * @param grad_prev_t 
     * @param costp 
     * @return true 
     * @return false 
     */
    bool swarmGradCostP(const int idx_cp,
                        const double t,
                        const Eigen::Vector3d &p,
                        const Eigen::Vector3d &v,
                        Eigen::Vector3d &gradp,
                        double &gradt,
                        double &grad_prev_t,
                        double &costp);

    /**
     * @brief Penalty on variance of distance between each point i.e. penalize the non-uniformity of distance between points
     * 
     * @param ps 
     * @param gdp 
     * @param var 
     */
    void distanceSqrVarianceWithGradCost2p(const Eigen::MatrixXd &ps,
                                           Eigen::MatrixXd &gdp,
                                           double &var);


    /**
     * @brief 
     * 
     * @tparam EIGENVEC 
     * @param gdT Gradient of size of number of pieces
     * @param costs a vector of costs
     * @param K Constraint points per piece, or total sample number
     */
    template <typename EIGENVEC>
    void addPVAGradCost2CT(EIGENVEC &gdT, Eigen::VectorXd &costs, const int &K)
    {
      int N = gdT.size();
      Eigen::Vector3d pos, vel, acc, jer;
      Eigen::Vector3d gradp, gradv, grada; // Each Gradient is a vector with (x,y,z) components
      double costp, costv, costa;
      Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3; // Time basis
      double s1, s2, s3, s4, s5;
      double step, alpha;
      Eigen::Matrix<double, 6, 3> gradViolaPc, gradViolaVc, gradViolaAc;
      double gradViolaPt, gradViolaVt, gradViolaAt;
      double omega;
      int idx_cp = 0; // Index of constraint point
      costs.setZero();

      std::vector<Eigen::Vector3d> all_s_obs;
      std::vector<Eigen::Vector3d> all_v_obs;

      double t = 0;
      for (int i = 0; i < N; ++i) // for each piece/segment number
      {
        const Eigen::Matrix<double, 6, 3> &c = jerkOpt_.get_b().block<6, 3>(i * 6, 0); // Polynomial coefficients 
        double T_i = jerkOpt_.get_T1()(i);
        step = T_i / K; // Duration of each piece / sample number.
        // step = jerkOpt_.get_T1()(i) / K;
        s1 = 0.0; // Time t, it will increase with each step of f the constraint point

        for (int j = 0; j <= K; ++j) // For each constraint point (or sample) in the segment. This is also known as the sample index
        {
          s2 = s1 * s1;   // t^2
          s3 = s2 * s1;   // t^3
          s4 = s2 * s2;   // t^4
          s5 = s4 * s1;   // t^5
          beta0 << 1.0, s1, s2, s3, s4, s5;                           // (1, t, t^2,  t^3,  t^4,    t^5)
          beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;  // (0, 1, 2t,   3t^2, 4t^3,   5t^4)
          beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;     // (0, 0, 1,    6t,   12t^2,  20t^3)
          beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2;          // (0, 0, 0,    1,    24t,    60t^2)
          alpha = j / K ; // progress along segment/piece, value is from 0 to 1

          // p_i(t) = c_i.T * beta(t) // (m, 1) = (m, 2s) * (2s, 1)
          pos = c.transpose() * beta0; // position at current constraint point
          vel = c.transpose() * beta1; // velocity at current constraint point
          acc = c.transpose() * beta2; // acceleration at current constraint point
          jer = c.transpose() * beta3; // jerk at current constraint point

          // omega: quadrature coefficients using trapezoid rule
          omega = (j == 0 || j == K) ? 0.5 : 1.0;

          cps_.points.col(idx_cp) = pos;

          /**
           * Penalty on clearance to static obstacle
           */
          if (!(idx_cp == 0 || idx_cp > ConstraintPoints::two_thirds_id(cps_.points, touch_goal_))){ // only apply to first 2/3

            Eigen::Matrix<double, 6, 3> pd_cost_c_i_static; // Accumulation of sum across each {p,v}
            pd_cost_c_i_static.setZero();
            double pd_cost_t_static{0};
            double cost_static{0};

            for (size_t j_cp = 0; j_cp < cps_.direction[idx_cp].size(); ++j_cp) // For each constraint direction
            {
              Eigen::Vector3d s_obs = cps_.base_point[idx_cp][j_cp];
              Eigen::Vector3d v_obs = cps_.direction[idx_cp][j_cp];

              all_s_obs.push_back(s_obs);
              all_v_obs.push_back(v_obs);

              double dist = (pos - s_obs).dot(v_obs);
              double obs_penalty = obs_clearance_ - dist;
              // double dist_err_soft = obs_clearance_soft_ - dist;

              if (obs_penalty > 0) // If clearance requirements from obstacle is not met
              {
                // Partial derivatives of Constraint
                Eigen::Matrix<double, 6, 3> pd_constr_c_i = - beta0 * v_obs.transpose(); // Partial derivative of constraint w.r.t c_i // (2s, m) = (2s, 1) * (1, m)
                double pd_constr_t =  - v_obs.transpose() * vel;// P.D. of constraint w.r.t t // (1,1) = (1, m) * (m, 1)

                // Intermediate calculations for chain rule to get partial derivatives of cost J
                double cost = (T_i / K) * omega * wei_obs_ * pow(obs_penalty, 3);
                double pd_cost_constr = 3 * (T_i / K) * omega * wei_obs_ * pow(obs_penalty, 2) ; // P.D. of cost w.r.t constraint
                double pd_t_T_i = (j / K); // P.D. of time t w.r.t T_i

                // Partial derivatives of Cost J
                Eigen::Matrix<double, 6, 3> pd_cost_c_i = pd_cost_constr * pd_constr_c_i; // P.D. of cost w.r.t c_i. Uses chain rule // (m,2s)
                double pd_cost_t = cost / T_i  + pd_cost_constr * pd_constr_t * pd_t_T_i;// P.D. of cost w.r.t t // (1,1)

                pd_cost_c_i_static += pd_cost_c_i;
                pd_cost_t_static += pd_cost_t;
                cost_static += cost;
              }
            }
            
            // Sum up sampled costs
            jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += pd_cost_c_i_static;
            gdT(i) += pd_cost_t_static; 
            costs(0) += cost_static; 
          }

          /**
           * Penalty on clearance to swarm/dynamic obstacles
           */
          double gradt, grad_prev_t;
          if (swarmGradCostP(idx_cp, t + step * j, pos, vel, gradp, gradt, grad_prev_t, costp))
          {
            gradViolaPc = beta0 * gradp.transpose();
            gradViolaPt = alpha * gradt;
            jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omega * step * gradViolaPc;
            gdT(i) += omega * (costp / K + step * gradViolaPt);
            if (i > 0)
            {
              gdT.head(i).array() += omega * step * grad_prev_t;
            }
            costs(1) += omega * step * costp;
          }

          /**
           * Penalty on velocity constraint, vector of (x,y,z)
           */
          double vpen = vel.squaredNorm() - max_vel_ * max_vel_; 
          if (vpen > 0){
            
            // Partial derivatives of Constraint
            Eigen::Matrix<double, 6, 3> pd_constr_c_i = 2 * beta1 * vel.transpose(); // Partial derivative of constraint w.r.t c_i // (2s, m) = (2s, 1) * (1, m)
            double pd_constr_t =  2 * beta2.transpose() * c * vel;// P.D. of constraint w.r.t t // (1,1) = (1, 2s) * (2s, m) * (m, 1)

            // Intermediate calculations for chain rule to get partial derivatives of cost J
            double pd_cost_constr = 3 * (T_i / K) * omega * wei_feas_ * pow(vpen,2) ; // P.D. of cost w.r.t constraint
            double cost = (T_i / K) * omega * wei_feas_ * pow(vpen,3);
            double pd_t_T_i = (j / K); // P.D. of time t w.r.t T_i

            // Partial derivatives of Cost J
            Eigen::Matrix<double, 6, 3> pd_cost_c_i = pd_cost_constr * pd_constr_c_i; // P.D. of cost w.r.t c_i. Uses chain rule // (m,2s)
            double pd_cost_t = cost / T_i  + pd_cost_constr * pd_constr_t * pd_t_T_i;// P.D. of cost w.r.t t // (1,1)

            // Sum up sampled costs
            jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += pd_cost_c_i;
            gdT(i) += pd_cost_t; 
            costs(2) += cost; 

          }

          /**
           * Penalty on acceleration constraint, vector of (x,y,z)
           */
          double apen = acc.squaredNorm() - max_acc_ * max_acc_; 
          if (apen > 0){
            
            // Partial derivatives of Constraint
            Eigen::Matrix<double, 6, 3> pd_constr_c_i = 2 * beta2 * acc.transpose(); // Partial derivative of constraint w.r.t c_i // (2s, m) = (2s, 1) * (1, m)
            double pd_constr_t =  2 * beta3.transpose() * c * acc;// P.D. of constraint w.r.t t // (1,1) = (1, 2s) * (2s, m) * (m, 1)

            // Intermediate calculations for chain rule to get partial derivatives of cost J
            double pd_cost_constr = 3 * (T_i / K) * omega * wei_feas_ * pow(apen,2) ; // P.D. of cost w.r.t constraint
            double cost = (T_i / K) * omega * wei_feas_ * pow(apen,3);
            double pd_t_T_i = (j / K); // P.D. of time t w.r.t T_i

            // Partial derivatives of Cost J
            Eigen::Matrix<double, 6, 3> pd_cost_c_i = pd_cost_constr * pd_constr_c_i; // P.D. of cost w.r.t c_i. Uses chain rule // (m,2s)
            double pd_cost_t = cost / T_i  + pd_cost_constr * pd_constr_t * pd_t_T_i;// P.D. of cost w.r.t t // (1,1)

            // Sum up sampled costs
            jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += pd_cost_c_i;
            gdT(i) += pd_cost_t; 
            costs(2) += cost; 
          }

          // printf("L\n");
          s1 += step;
          if (j != K || (j == K && i == N - 1))
          {
            ++idx_cp;
          }
        }

        t += jerkOpt_.get_T1()(i);
      }

      /**
       * Penalty on quadratic variance
       */
      Eigen::MatrixXd gdp;
      double var;
      // lengthVarianceWithGradCost2p(cps_.points, K, gdp, var);
      distanceSqrVarianceWithGradCost2p(cps_.points, gdp, var);
      // std::cout << "var=" << var <<std::endl;

      idx_cp = 0;
      for (int i = 0; i < N; ++i) // for each piece/segment number
      {
        step = jerkOpt_.get_T1()(i) / K;
        s1 = 0.0;

        for (int j = 0; j <= K; ++j) // For each constraint point
        {
          s2 = s1 * s1;
          s3 = s2 * s1;
          s4 = s2 * s2;
          s5 = s4 * s1;
          beta0 << 1.0, s1, s2, s3, s4, s5;
          beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
          alpha = 1.0 / K * j;
          vel = jerkOpt_.get_b().block<6, 3>(i * 6, 0).transpose() * beta1;

          omega = (j == 0 || j == K) ? 0.5 : 1.0;

          gradViolaPc = beta0 * gdp.col(idx_cp).transpose();
          gradViolaPt = alpha * gdp.col(idx_cp).transpose() * vel;
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omega * gradViolaPc;
          gdT(i) += omega * (gradViolaPt);

          s1 += step;
          if (j != K || (j == K && i == N - 1))
          {
            ++idx_cp;
          }
        }
      }
      costs(3) += var;


      // Publish for debugging
      visualization_->pubSVPairs(all_s_obs, all_v_obs, 0, Eigen::Vector4d(1, 0.5, 0, 1));
    }

    /**
     * @brief The LBFGS callback function to provide function and gradient evaluations given a current values of variables
     * 
     * @param func_data The user data sent for lbfgs_optimize() function by the client.
     * @param x         The current values of variables.
     * @param grad      The gradient vector. The callback function must compute
     *                      the gradient values for the current variables.
     * @param n         The number of variables.
     * @return double   The value of the objective function for the current
     *                          variables.
     */
    static double costFunctionCallback(void *func_data, const double *x, double *grad, const int n);  

    // void lengthVarianceWithGradCost2p(const Eigen::MatrixXd &ps,
    //                                   const int n,
    //                                   Eigen::MatrixXd &gdp,
    //                                   double &var);


  }; // class PolyTrajOptimizer

} // namespace ego_planner

#endif //_POLY_TRAJ_OPTIMIZER_H_