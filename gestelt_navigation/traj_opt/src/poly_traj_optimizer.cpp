#include "optimizer/poly_traj_optimizer.h"

namespace ego_planner
{

  bool PolyTrajOptimizer::optimizeTrajectory(const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
                          const Eigen::MatrixXd &inner_ctrl_pts, const Eigen::VectorXd &initT,
                          Eigen::MatrixXd &initial_cstr_pts, double &final_cost)
  {
    // IF size of inner points and segment durations are not the same, there is a bug
    if (inner_ctrl_pts.cols() != (initT.size() - 1))
    {
      ROS_ERROR("[PolyTrajOptimizer::optimizeTrajectory] inner_ctrl_pts.cols() != (initT.size()-1)");
      return false;
    }

    num_segs_ = initT.size();

    mjo_xi_.reset(iniState, finState, num_segs_);

    // Number of coefficients
    variable_num_ = 4 * (num_segs_ - 1) + 1;

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
      auto opt_start_time = ros::Time::now();
      int result = lbfgs::lbfgs_optimize(
          variable_num_,                  // The number of variables
          x_init,                         // The array of variables.
          &final_cost,                    // The pointer to the variable that receives the final value of the objective function for the variables
          PolyTrajOptimizer::costFunctionCallback, // The callback function to provide function and gradient evaluations given a current values of variables
          NULL,                           //  The callback function to provide values of the upperbound of the stepsize to search in, provided with the beginning values of variables before the linear search, and the current step vector (can be negative gradient).
          PolyTrajOptimizer::earlyExitCallback,    // The callback function to receive the progress (the number of iterations, the current value of the objective function) of the minimization process.
          this,                           // A user data for the client program. The callback functions will receive the value of this argument.
          &lbfgs_params);                 // The pointer to a structure representing parameters for L-BFGS optimization. A client program can set this parameter to NULL to use the default parameters

      auto opt_end_time = ros::Time::now();
      double time_ms = (opt_end_time - opt_start_time).toSec() * 1000;
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
            finelyCheckAndSetConstraintPoints(segments_nouse, mjo_xi_, false) == CHK_RET::OBS_FREE)
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
        ROS_WARN("[PolyTrajOptimizer]: UAV %d: Solver error. Return = %d, %s. Skip this planning.", drone_id_, result, lbfgs::lbfgs_strerror(result));
      }

    } while (
        (flag_still_occ && restart_nums < 3) ||
        (flag_force_return && force_stop_type_ == STOP_FOR_REBOUND && rebound_times <= 20));

    initial_cstr_pts = cps_.points; 

    return flag_success;
  }

  /* EGO Planner algorithm: callbacks by the L-BFGS optimizer */
  double PolyTrajOptimizer::costFunctionCallback(void *func_data, const double *x, double *grad, const int n)
  {
    // Pointer to current instance of PolyTrajOptimizer
    PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(func_data);

    opt->min_ellip_dist2_ = std::numeric_limits<double>::max();

    // x is pointer to start of the array/matrix
    Eigen::Map<const Eigen::MatrixXd> P(x, 3, opt->num_segs_ - 1); // 3 x (M-1)
    // Eigen::VectorXd T(Eigen::VectorXd::Constant(piece_nums, opt->t2T(x[n - 1]))); // same t
    // we specify "x + (3 * (opt->num_segs_ - 1))" because that is the address, its not the value
    Eigen::Map<const Eigen::VectorXd> t(x + (3 * (opt->num_segs_ - 1)), opt->num_segs_);

    // Gradient values of P and t
    Eigen::Map<Eigen::MatrixXd> gradP(grad, 3, opt->num_segs_ - 1);
    Eigen::Map<Eigen::VectorXd> gradt(grad + (3 * (opt->num_segs_ - 1)), opt->num_segs_);

    // from virtual time t to real time T
    Eigen::VectorXd T(opt->num_segs_);
    opt->VirtualT2RealT(t, T);

    Eigen::VectorXd gradT(opt->num_segs_); // P.D. of costs w.r.t time t, A vector of size (M, 1)
    double jerk_cost = 0, time_cost = 0;
    Eigen::VectorXd obs_swarm_feas_qvar_costs(4); // Vector of costs containing (Static obstacles, swarm, dynamic, feasibility, qvar)

    opt->mjo_xi_.generate(P, T);

    /* 1. Smoothness cost */
    // jerk_cost is the cost of the jerk minimization trajectory
    opt->mjo_xi_.initGradCost(gradT, jerk_cost); // does addGradJbyT(gdT) and addGradJbyC(gdC);

    /** 2. Time integral cost 
      *   2a. Static obstacle cost
      *   2b. Swarm cost
      *   2c. Dynamical feasibility
      *   2d. Uniformity of points
    */
    opt->addPVAGradCost2CT(gradT, obs_swarm_feas_qvar_costs, opt->cps_num_perPiece_); 

    if (opt->iter_num_ > 3 && jerk_cost / opt->num_segs_ < 10.0) // 10.0 is an experimental value that indicates the trajectory is smooth enough.
    {
      opt->roughlyCheckConstraintPoints();
    }

    opt->mjo_xi_.getGrad2TP(gradT, gradP);
    // time_cost += opt->rho_ * T(0) * piece_nums;  // same t
    // grad[n - 1] = (gradT.sum() + opt->rho_ * piece_nums) * gdT2t(x[n - 1]);  // same t

    opt->VirtualTGradCost(T, t, gradT, gradt, time_cost);

    opt->iter_num_ += 1;
    return jerk_cost + obs_swarm_feas_qvar_costs.sum() + time_cost;

    return 0;
  }

  bool PolyTrajOptimizer::swarmGradCostP(const int idx_cp,
                                         const double t,
                                         const Eigen::Vector3d &p,
                                         const Eigen::Vector3d &v,
                                         Eigen::Vector3d &gradp,
                                         double &gradt,
                                         double &grad_prev_t,
                                         double &costp)
  {
    // if (idx_cp <= 0 || idx_cp > ConstraintPoints::two_thirds_id(cps_.points, touch_goal_)) // only apply to first 2/3
    //   return false;

    bool ret = false;

    gradp.setZero();
    gradt = 0;
    grad_prev_t = 0;
    costp = 0;

    const double CLEARANCE2 = (swarm_clearance_ * 1.5) * (swarm_clearance_ * 1.5);
    constexpr double a = 2.0, b = 1.0, inv_a2 = 1 / a / a, inv_b2 = 1 / b / b;

    double pt_time = t_now_ + t;

    swarm_traj_mutex_.lock();  

    for (const auto& id_traj_pair : swarm_local_trajs_){ // Iterate through trajectories

      auto agent_traj = id_traj_pair.second;
      
      if ((agent_traj.drone_id < 0) || agent_traj.drone_id == drone_id_)
      {
        // Ignore 
        continue;
      }

      double traj_i_start_time = agent_traj.start_time;

      Eigen::Vector3d swarm_p, swarm_v;
      if (pt_time < traj_i_start_time + agent_traj.duration)
      {
        swarm_p = agent_traj.traj.getPos(pt_time - traj_i_start_time);
        swarm_v = agent_traj.traj.getVel(pt_time - traj_i_start_time);
      }
      else
      {
        double exceed_time = pt_time - (traj_i_start_time + agent_traj.duration);
        swarm_v = agent_traj.traj.getVel(agent_traj.duration);
        swarm_p = agent_traj.traj.getPos(agent_traj.duration) +
                  exceed_time * swarm_v;
      }
      Eigen::Vector3d dist_vec = p - swarm_p;
      double ellip_dist2 = dist_vec(2) * dist_vec(2) * inv_a2 + (dist_vec(0) * dist_vec(0) + dist_vec(1) * dist_vec(1)) * inv_b2;
      double dist2_err = CLEARANCE2 - ellip_dist2;
      double dist2_err2 = dist2_err * dist2_err;
      double dist2_err3 = dist2_err2 * dist2_err;

      if (dist2_err3 > 0)
      {
        ret = true;

        costp += wei_swarm_ * dist2_err3;

        Eigen::Vector3d dJ_dP = wei_swarm_ * 3 * dist2_err2 * (-2) * Eigen::Vector3d(inv_b2 * dist_vec(0), inv_b2 * dist_vec(1), inv_a2 * dist_vec(2));
        gradp += dJ_dP;
        gradt += dJ_dP.dot(v - swarm_v);
        grad_prev_t += dJ_dP.dot(-swarm_v);
      }

      if (min_ellip_dist2_ > ellip_dist2)
      {
        min_ellip_dist2_ = ellip_dist2;
      }
    }
    
    swarm_traj_mutex_.unlock();  

    return ret;
  }

  void PolyTrajOptimizer::distanceSqrVarianceWithGradCost2p(const Eigen::MatrixXd &ps,
                                                            Eigen::MatrixXd &gdp,
                                                            double &var)
  {
    size_t num_pts = ps.cols();
    // dist_btw_all_pts: distance between consecutive points
    //    ps.rightCols(num_pts-1): block containing all columns except first one
    //    ps.rightCols(num_pts-1): block containing all columns except last one
    Eigen::MatrixXd dist_btw_all_pts = ps.rightCols(num_pts - 1 ) - ps.leftCols(num_pts - 1);
    // dsqrs: Get squared norm of dist_btw_all_pts
    Eigen::VectorXd dsqrs = dist_btw_all_pts.colwise().squaredNorm().transpose();
    // dquarsum: Get squared norm of of all dsqrs
    double dquarsum = dsqrs.squaredNorm(); 

    // dquarmean: Mean of all squared distance
    double dquarmean = dquarsum / (num_pts-1);

    var = wei_sqrvar_ * (dquarmean);
    gdp.resize(3, num_pts);
    gdp.setZero();

    for (size_t i = 0; i < num_pts; i++) // iterate through all points 
    {
      if (i != 0) // If not first point
      {
        gdp.col(i) += wei_sqrvar_ * (4.0 * (dsqrs(i - 1)) / (num_pts-1) * dist_btw_all_pts.col(i - 1));
      }
      if (i != (num_pts-1)) // If not last point
      {
        gdp.col(i) += wei_sqrvar_ * (-4.0 * (dsqrs(i)) / (num_pts-1) * dist_btw_all_pts.col(i));
      }
    }

    return;
  }

  /* helper functions */
  void PolyTrajOptimizer::setParam(ros::NodeHandle &pnh)
  {
    pnh.param("drone_id", drone_id_, -1);

    pnh.param("optimization/num_cstr_pts_per_seg", cps_num_perPiece_, -1);
    pnh.param("optimization/weight_sph_bounds", wei_sph_bounds_, -1.0);
    pnh.param("optimization/weight_swarm", wei_swarm_, -1.0);
    pnh.param("optimization/weight_feasibility", wei_feas_, -1.0);
    pnh.param("optimization/weight_sqrvariance", wei_sqrvar_, -1.0);
    pnh.param("optimization/weight_time", wei_time_, -1.0);
    pnh.param("optimization/swarm_clearance", swarm_clearance_, -1.0);
    pnh.param("optimization/max_vel", max_vel_, -1.0);
    pnh.param("optimization/max_acc", max_acc_, -1.0);

    // EGO-Planner only params
    pnh.param("optimization/obstacle_clearance", obs_clearance_, -1.0);
    pnh.param("optimization/weight_obstacle", wei_obs_, -1.0);

    // Barrier function
    pnh.param("optimization/barrier/alpha", bar_alp_, -1.0);
    pnh.param("optimization/barrier/buffer", bar_buf_, -1.0);
    pnh.param("optimization/barrier/opt_weight", wei_barrier_, -1.0);

  }

  int PolyTrajOptimizer::earlyExitCallback(void *func_data, const double *x, const double *g, const double fx, const double xnorm, const double gnorm, const double step, int n, int k, int ls)
  {
    PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(func_data);

    return (opt->force_stop_type_ == STOP_FOR_ERROR || opt->force_stop_type_ == STOP_FOR_REBOUND);
  }


  template <typename EIGENVEC>
  void PolyTrajOptimizer::RealT2VirtualT(const Eigen::VectorXd &RT, EIGENVEC &VT)
  {
    for (int i = 0; i < RT.size(); ++i)
    {
      VT(i) = RT(i) > 1.0 ? (sqrt(2.0 * RT(i) - 1.0) - 1.0)
                          : (1.0 - sqrt(2.0 / RT(i) - 1.0));
    }
  }

  template <typename EIGENVEC>
  void PolyTrajOptimizer::VirtualT2RealT(const EIGENVEC &VT, Eigen::VectorXd &RT)
  {
    for (int i = 0; i < VT.size(); ++i)
    {
      RT(i) = VT(i) > 0.0 ? ((0.5 * VT(i) + 1.0) * VT(i) + 1.0)
                          : 1.0 / ((0.5 * VT(i) - 1.0) * VT(i) + 1.0);
    }
  }

  template <typename EIGENVEC, typename EIGENVECGD>
  void PolyTrajOptimizer::VirtualTGradCost(
      const Eigen::VectorXd &RT, const EIGENVEC &VT,
      const Eigen::VectorXd &gdRT, EIGENVECGD &gdVT,
      double &costT)
  {
    // VirtualTGradCost(T, t, gradT, gradt, time_cost)
    // gdRT: gradT // Grad real time
    // gdVT: gradt // Grad virtual time

    for (int i = 0; i < VT.size(); ++i)
    {
      double gdVT2Rt;
      if (VT(i) > 0)
      {
        gdVT2Rt = VT(i) + 1.0;
      }
      else
      {
        double denSqrt = (0.5 * VT(i) - 1.0) * VT(i) + 1.0;
        gdVT2Rt = (1.0 - VT(i)) / (denSqrt * denSqrt);
      }

      gdVT(i) = (gdRT(i) + wei_time_) * gdVT2Rt;
    }

    costT = RT.sum() * wei_time_;
  }


} // namespace ego_planner