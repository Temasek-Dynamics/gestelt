#include "optimizer/poly_traj_optimizer.h"

namespace ego_planner
{
  bool PolyTrajOptimizer::optimizeTrajectorySFC(
      const Eigen::Matrix3d &startPVA, const Eigen::Matrix3d &endPVA,
      const Eigen::MatrixXd &inner_ctrl_pts, const Eigen::VectorXd &initT,
      const std::vector<Eigen::Vector3d>& spheres_center, const std::vector<double>& spheres_radius, 
      const std::vector<Eigen::Vector3d>& spheres_intxn_plane_vec, const std::vector<double>& spheres_intxn_plane_dist,
      const std::vector<Eigen::Vector3d>& sfc_traj_waypoints, const std::vector<double>& intxn_circle_radius,
      double &final_cost)
  {
    // IF size of inner points and segment durations are not the same, there is a bug
    if (inner_ctrl_pts.cols() != (initT.size() - 1))
    {
      ROS_ERROR("[PolyTrajOptimizer::optimizeTrajectory] inner_ctrl_pts.cols() != (initT.size()-1)");
      return false;
    }

    // assign all necessary variables
    spheres_radius_ = spheres_radius;
    spheres_center_ = spheres_center;
    intxn_plane_vec_ = spheres_intxn_plane_vec;
    intxn_plane_dist_ = spheres_intxn_plane_dist;
    intxn_center_ = sfc_traj_waypoints;
    intxn_circle_radius_ = intxn_circle_radius;

    // reset all existing variables
    opt_costs_.reset();
    intermediate_cstr_pts_xi_.clear();
    intermediate_cstr_pts_q_.clear();

    // Initialize minimum jerk trajectory
    num_segs_ = initT.size(); // Number of segments
    variable_num_ = 4 * (num_segs_ - 1) + 1; //  Number of decision variables. Position (size 3 x (M-1)) + Time ( size 1 x (M-1) + 1) 
    mjo_xi_.reset(startPVA, endPVA, num_segs_);
    mjo_q_.reset(startPVA, endPVA, num_segs_);

    ros::Time t0 = ros::Time::now(), t1, t2;
    int restart_nums = 0, rebound_times = 0;
    bool flag_force_return, flag_still_occ, flag_success;

    double x_init[variable_num_]; // Initial decision variables: Array of [ inner_ctrl_pts_xi, initT ]

    // Convert initial SFC points from q into xi variables
    //      (inner_ctrl_pts is of size (3, M-1))
    // Eigen::MatrixXd inner_ctrl_pts_xi = f_BInv_ctrl_pts(inner_ctrl_pts, spheres_center_, spheres_radius_); // (3, num_segs_ - 1);
    Eigen::MatrixXd inner_ctrl_pts_xi = inner_ctrl_pts;
    // XJP: Copy transformed inner_ctrl_pts_xi
    memcpy(x_init, inner_ctrl_pts_xi.data(), inner_ctrl_pts_xi.size() * sizeof(x_init[0]));
    // Virtual Time Vt
    Eigen::Map<Eigen::VectorXd> Vt(x_init + inner_ctrl_pts_xi.size(), initT.size()); 

    // Convert from real time initT to virtual time Vt
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
          PolyTrajOptimizer::costFunctionCallbackSFC, // The callback function to provide function and gradient evaluations given a current values of variables
          NULL,                           //  The callback function to provide values of the upperbound of the stepsize to search in, provided with the beginning values of variables before the linear search, and the current step vector (can be negative gradient).
          PolyTrajOptimizer::earlyExitCallback,    // The callback function to receive the progress (the number of iterations, the current value of the objective function) of the minimization process.
          this,                           // A user data for the client program. The callback functions will receive the value of this argument.
          &lbfgs_params);                 // The pointer to a structure representing parameters for L-BFGS optimization. A client program can set this parameter to NULL to use the default parameters

      t2 = ros::Time::now();
      double time_ms = (t2 - t1).toSec() * 1000;
      double total_time_ms = (t2 - t0).toSec() * 1000;

      /* ---------- get result and check collision ---------- */

      // IF converged, maxmimum_iteration, already minimized or stop
      if (result == lbfgs::LBFGS_CONVERGENCE ||
          result == lbfgs::LBFGSERR_MAXIMUMITERATION ||
          result == lbfgs::LBFGS_ALREADY_MINIMIZED ||
          result == lbfgs::LBFGS_STOP)
      {
        flag_force_return = false;
        // TODO: Add collision-checking in the path
        printf("\033[32m [PolyTrajOptimizer] Optimization Succeeded! iter=%d, time(ms)=%5.3f, total_t(ms)=%5.3f, cost=%5.3f\n \033[0m", iter_num_, time_ms, total_time_ms, final_cost);
        flag_success = true;
      }
      // ELSE IF Cancelled
      else if (result == lbfgs::LBFGSERR_CANCELED)
      {
        flag_force_return = true;
        rebound_times++; 
        std::cout << "[PolyTrajOptimizer] Optimization Cancelled!" << std::endl;
        // std::cout << "iter=" << iter_num_ << ",time(ms)=" << time_ms << ",rebound." <<std::endl;
      }
      // ELSE ERROR
      else
      {
        std::cout << "iter=" << iter_num_ << ",time(ms)=" << time_ms << ",error." <<std::endl;
        ROS_WARN("[PolyTrajOptimizer] Solver error. Return = %d, %s. Skip this planning.", result, lbfgs::lbfgs_strerror(result));
      }

    } while ((flag_force_return && force_stop_type_ == STOP_FOR_REBOUND && rebound_times <= 20));

    return flag_success;
  }

  /* callbacks by the L-BFGS optimizer */
  double PolyTrajOptimizer::costFunctionCallbackSFC(void *func_data, const double *x, double *grad, const int n)
  {
    // Pointer to current instance of PolyTrajOptimizer
    PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(func_data);

    // func_data = instance of class                                  // The user data sent for lbfgs_optimize() function by the client.
    // x         = x_init = [inner_ctrl_pts, virtual_time_durations]  // x is pointer to start of the array/matrix. The current values of variables.
    // grad      = (gradP, gradt)                                     // The gradient vector. The callback function must compute the gradient values for the current variables.
    // n         = variable_num_ = 4 * (num_segs_ - 1) + 1           // The number of variables.

    // P_xi IS CONTROL POINTS IN XI UNCONSTRAINED SPACE
    //      Taken from decision variables at start of iteration. inner 3d position of trajectory
    Eigen::Map<const Eigen::MatrixXd> P_xi(x, 3, opt->num_segs_ - 1); // 3 x (M-1)
    //      we specify "x + (3 * (opt->num_segs_ - 1))" because that is the address, its not the value
    Eigen::Map<const Eigen::VectorXd> t(x + (3 * (opt->num_segs_ - 1)), opt->num_segs_);
    // convert from virtual time t to real time T
    Eigen::VectorXd T(opt->num_segs_);
    opt->VirtualT2RealT(t, T); 

    // gradP: start from grad[0]
    Eigen::Map<Eigen::MatrixXd> gradP(grad, 3, opt->num_segs_ - 1); // p.d.(H / xi)
    // gradt: start from grad[3 * (M-1)]
    Eigen::Map<Eigen::VectorXd> gradt(grad + (3 * (opt->num_segs_ - 1)), opt->num_segs_); // p.d.(H / t_i)
    // gradT: P.D. of objective function H w.r.t time T, A vector of size (M, 1)
    //        Each element is a value of the gradient
    Eigen::VectorXd gradT(opt->num_segs_); 

    // Initialize values
    double jerk_cost = 0, time_cost = 0;
    opt->min_ellip_dist2_ = std::numeric_limits<double>::max();

    // For forward cost evaluation: Get minimum jerk trajectory coordinates in q space
    Eigen::MatrixXd P_q;
    if (opt->iter_num_ == 0){
      P_q = P_xi;
    }
    else {
      P_q = opt->f_B_ctrl_pts(P_xi,
                              opt->spheres_center_, opt->spheres_radius_,
                              opt->intxn_plane_vec_, opt->intxn_plane_dist_,
                              opt->intxn_center_, opt->intxn_circle_radius_);
    }
    opt->mjo_q_.generate(P_q, T); // Generate minimum jerk trajectory
    opt->cstr_pts_q_ = opt->mjo_q_.getInitConstraintPoints(opt->cps_num_perPiece_); // Discretize trajectory into inner constraint points

    opt->ctrl_pts_q_optimal_ = P_q;

    // For visualization: Get minimum jerk trajectory coordinates in xi space
    opt->mjo_xi_.generate(P_xi, T); // Generate minimum jerk trajectory
    opt->cstr_pts_xi_ = opt->mjo_xi_.getInitConstraintPoints(opt->cps_num_perPiece_);

    /** 1. Jerk cost 
     * jerk_cost is trajectory jerk cost
     */
    opt->mjo_q_.initGradCost(gradT, jerk_cost); // Among other things, do addGradJbyT(gdT) and addGradJbyC(gdC);

    /** 2. Time integral cost 
      *   2a. Static obstacle cost
      *   2b. Swarm cost
      *   2c. Dynamical feasibility
      *   2d. Uniformity of points
    */
    Eigen::VectorXd obs_swarm_feas_qvar_costs(4); // Vector of costs containing (Static obstacles, swarm, dynamic, feasibility, qvar)
    opt->addPVAGradCost2CT_SFC(gradT, obs_swarm_feas_qvar_costs, opt->cps_num_perPiece_, opt->mjo_q_); 

    // Update the gradient costs p.d.(H / T_i) 
    //                       and p.d.(H / xi)
    opt->mjo_q_.getGrad2TP(gradT, gradP, P_xi, 
                            opt->spheres_center_, opt->spheres_radius_,
                            opt->intxn_plane_vec_, opt->intxn_plane_dist_,
                            opt->intxn_center_, opt->intxn_circle_radius_);
    // time_cost += opt->rho_ * T(0) * piece_nums;  // same t
    // grad[n - 1] = (gradT.sum() + opt->rho_ * piece_nums) * gdT2t(x[n - 1]);  // same t

    /** 3. Time cost 
    */
    opt->VirtualTGradCost(T, t, gradT, gradt, time_cost);

    // Collect intermediate MJO trajectories for publishing later
    opt->intermediate_cstr_pts_xi_.push_back(P_xi);
    opt->intermediate_cstr_pts_q_.push_back(P_q);

    opt->visualization_->displayIntermediateGrad("aggregate_position", P_xi, gradP);

    opt->opt_costs_.addCosts(
      jerk_cost,
      obs_swarm_feas_qvar_costs(0),
      obs_swarm_feas_qvar_costs(1),
      obs_swarm_feas_qvar_costs(2),
      obs_swarm_feas_qvar_costs(3),
      time_cost);

    opt->iter_num_++;

    return jerk_cost + obs_swarm_feas_qvar_costs.sum() + time_cost;
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

  template <typename EIGENVEC>
  void PolyTrajOptimizer::addPVAGradCost2CT_SFC(
      EIGENVEC &gdT, Eigen::VectorXd &costs, const int &K, poly_traj::MinJerkOpt& mjo)
  {
    int N = num_segs_; // N: Number of segments
    Eigen::Vector3d pos, vel, acc, jer;
    Eigen::Vector3d gradp, gradv, grada; // Each Gradient is a vector with (x,y,z) components
    double costp, costv, costa; 
    Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3; // Time basis
    double s1, s2, s3, s4, s5;    // time (s1 = t, s2 = t^2, ..., s5 = t^5)
    double step, alpha;           // step: time step, alpha: 
    Eigen::Matrix<double, 6, 3> gradViolaPc, gradViolaVc, gradViolaAc;
    double gradViolaPt, gradViolaVt, gradViolaAt;
    double omega;    // Quadrature coefficient
    int idx_cp = 0; // Index of constraint point
    costs.setZero();

    double t = 0;
    for (int i = 0; i < N; ++i) // for each piece/segment number
    {
      // c: Polynomial coefficients 
      const Eigen::Matrix<double, 6, 3> &c = mjo.get_b().block<6, 3>(i * 6, 0); 
      double T_i = mjo.get_T1()(i);
      step = T_i / K; // Duration of each piece / sample number.
      s1 = 0.0; // Time t, it will increase with each step of the constraint point

      for (int j = 0; j <= K; ++j) // For each constraint point (or sample) in the segment. This is also known as the sample index
      {
        s2 = s1 * s1;   // t^2
        s3 = s2 * s1;   // t^3
        s4 = s2 * s2;   // t^4
        s5 = s4 * s1;   // t^5
        // betax: Time bases 
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

        /**
         * Penalty on static obstacle, vector of (x,y,z)
         */
        
        // obs_static_pen: Distance from current point to center of sphere at segment i 
        Eigen::Vector3d sph_ctr_to_pos_vec = pos - spheres_center_[i]; // Sphere center to pos vector
        double sph_ctr_to_pos_dist = sph_ctr_to_pos_vec.norm();

        double obs_static_pen = sph_ctr_to_pos_dist * sph_ctr_to_pos_dist - spheres_radius_[i] * spheres_radius_[i] ; 

        if (obs_static_pen > 0){  // If current point is outside the sphere/on boundary 
        
          // std::cout << "Segment " << i << ": ," << "Penalty " << obs_static_pen << ", POINT (" << pos.transpose() 
          //   << ") is outside sphere with center ("<< spheres_center_[i].transpose() << ") with radius " << spheres_radius_[i] << std::endl;

          // Partial derivatives of Constraint
          Eigen::Matrix<double, 6, 3> pd_constr_c_i = 2 * beta0 * sph_ctr_to_pos_vec.transpose(); // Partial derivative of constraint w.r.t c_i // (2s, m) = (2s, 1) * (1, m)
          double pd_constr_t =  2 * beta1.transpose() * c * sph_ctr_to_pos_vec;// P.D. of constraint w.r.t t // (1,1) = (1, 2s) * (2s, m) * (m, 1)

          // Intermediate calculations for chain rule to get partial derivatives of cost J
          double pd_cost_constr = 3 * (T_i / K) * omega * wei_sph_bounds_ * pow(obs_static_pen,2) ; // P.D. of cost w.r.t constraint
          double cost = (T_i / K) * omega * wei_sph_bounds_ * pow(obs_static_pen,3);
          double pd_t_T_i = (j / K); // P.D. of time t w.r.t T_i

          // Partial derivatives of Cost J
          Eigen::Matrix<double, 6, 3> pd_cost_c_i = pd_cost_constr * pd_constr_c_i; // P.D. of cost w.r.t c_i. Uses chain rule // (m,2s)
          double pd_cost_t = cost / T_i  + pd_cost_constr * pd_constr_t * pd_t_T_i;// P.D. of cost w.r.t t // (1,1)

          // Sum up sampled costs
          mjo.get_gdC().block<6, 3>(i * 6, 0) += pd_cost_c_i; // Gradient of cost w.r.t polynomial coefficients, shape is (m,2s)
          gdT(i) += pd_cost_t; // Gradient of cost w.r.t time
          costs(0) += cost; 
        }

        /**
         * Penalty on clearance to swarm/dynamic obstacles
         */
        double gradt, grad_prev_t;
        if (swarmGradCostP(idx_cp, t + step * j, pos, vel, gradp, gradt, grad_prev_t, costp))
        {
          gradViolaPc = beta0 * gradp.transpose();
          gradViolaPt = alpha * gradt;
          mjo.get_gdC().block<6, 3>(i * 6, 0) += omega * step * gradViolaPc;
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
        double vel_pen = vel.squaredNorm() - max_vel_ * max_vel_; 
        if (vel_pen > 0)
        {  
          // Partial derivatives of Constraint
          Eigen::Matrix<double, 6, 3> pd_constr_c_i = 2 * beta1 * vel.transpose(); // Partial derivative of constraint w.r.t c_i // (2s, m) = (2s, 1) * (1, m)
          double pd_constr_t =  2 * beta2.transpose() * c * vel;// P.D. of constraint w.r.t t // (1,1) = (1, 2s) * (2s, m) * (m, 1)

          // Intermediate calculations for chain rule to get partial derivatives of cost J
          double pd_cost_constr = 3 * (T_i / K) * omega * wei_feas_ * pow(vel_pen,2) ; // P.D. of cost w.r.t constraint
          double cost = (T_i / K) * omega * wei_feas_ * pow(vel_pen,3);
          double pd_t_T_i = (j / K); // P.D. of time t w.r.t T_i

          // Partial derivatives of Cost J
          //    w.r.t coefficients c_i
          Eigen::Matrix<double, 6, 3> pd_cost_c_i = pd_cost_constr * pd_constr_c_i; // P.D. of cost w.r.t c_i. Uses chain rule // (m,2s)
          //    w.r.t time t
          double pd_cost_t = cost / T_i  + pd_cost_constr * pd_constr_t * pd_t_T_i;// P.D. of cost w.r.t t // (1,1)

          // Sum up sampled costs
          mjo.get_gdC().block<6, 3>(i * 6, 0) += pd_cost_c_i; // Gradient of cost w.r.t polynomial coefficients
          gdT(i) += pd_cost_t; 
          costs(2) += cost; 
        }

        /**
         * Penalty on acceleration constraint, vector of (x,y,z)
         */
        double acc_pen = acc.squaredNorm() - max_acc_ * max_acc_; 
        if (acc_pen > 0)
        {  
          // Partial derivatives of Constraint
          Eigen::Matrix<double, 6, 3> pd_constr_c_i = 2 * beta2 * acc.transpose(); // Partial derivative of constraint w.r.t c_i // (2s, m) = (2s, 1) * (1, m)
          double pd_constr_t =  2 * beta3.transpose() * c * acc;// P.D. of constraint w.r.t t // (1,1) = (1, 2s) * (2s, m) * (m, 1)

          // Intermediate calculations for chain rule to get partial derivatives of cost J
          double pd_cost_constr = 3 * (T_i / K) * omega * wei_feas_ * pow(acc_pen,2) ; // P.D. of cost w.r.t constraint
          double cost = (T_i / K) * omega * wei_feas_ * pow(acc_pen,3);
          double pd_t_T_i = (j / K); // P.D. of time t w.r.t T_i

          // Partial derivatives of Cost J 
          //    w.r.t coefficients c_i
          Eigen::Matrix<double, 6, 3> pd_cost_c_i = pd_cost_constr * pd_constr_c_i; // P.D. of cost w.r.t c_i. Uses chain rule // (m,2s)
          //    w.r.t time t
          double pd_cost_t = cost / T_i  + pd_cost_constr * pd_constr_t * pd_t_T_i;// P.D. of cost w.r.t t // (1,1)

          // Sum up sampled costs
          mjo.get_gdC().block<6, 3>(i * 6, 0) += pd_cost_c_i; // Gradient of cost w.r.t polynomial coefficients
          gdT(i) += pd_cost_t; 
          costs(2) += cost; 
        }

        // printf("L\n");
        s1 += step; // Step to next time step

        // If current CP is not last in the segment
        // OR current CP is last point of last segment
        if (j != K || (j == K && i == N - 1))
        {
          ++idx_cp;
        }
      } // end iteration through all constraint points in segment

      t += mjo.get_T1()(i);
    } // end iteration through all segments

    /**
     * Penalty on quadratic variance
     */
    Eigen::MatrixXd gdp;
    double var;
    distanceSqrVarianceWithGradCost2p(cstr_pts_q_, gdp, var);
    // std::cout << "var=" << var <<std::endl;

    // What cost is this?
    idx_cp = 0;
    for (int i = 0; i < N; ++i) // for each piece/segment number
    {
      step = mjo.get_T1()(i) / K;
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
        vel = mjo.get_b().block<6, 3>(i * 6, 0).transpose() * beta1;

        omega = (j == 0 || j == K) ? 0.5 : 1.0;

        gradViolaPc = beta0 * gdp.col(idx_cp).transpose();
        gradViolaPt = alpha * gdp.col(idx_cp).transpose() * vel;
        mjo.get_gdC().block<6, 3>(i * 6, 0) += omega * gradViolaPc;
        gdT(i) += omega * (gradViolaPt);

        s1 += step;
        if (j != K || (j == K && i == N - 1))
        {
          ++idx_cp;
        }
      } // end iteration through all constraint points in segment
    } // end iteration through all pieces

    costs(3) += var;

    /* Publish visualization of gradients*/
    // visualization_->displayIntermediateGrad("smoothness", ctrl_pts_q_, mjo.get_gdC());
    visualization_->displayIntermediateGrad("dist_variance", cstr_pts_q_, gdp);

  } // end func addPVAGradCost2CT_SFC

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

    if (swarm_local_trajs_ == nullptr){
      return false;
    }

    for (auto& it : *swarm_local_trajs_){ // Iterate through trajectories

      int id = it.first;

      if ((id < 0) || id == drone_id_)
      {
        // Ignore 
        continue;
      }

      double traj_i_start_time = it.second.start_time;

      Eigen::Vector3d swarm_p, swarm_v;
      if (pt_time < traj_i_start_time + it.second.duration)
      {
        swarm_p = it.second.traj.getPos(pt_time - traj_i_start_time);
        swarm_v = it.second.traj.getVel(pt_time - traj_i_start_time);
      }
      else
      {
        double exceed_time = pt_time - (traj_i_start_time + it.second.duration);
        swarm_v = it.second.traj.getVel(it.second.duration);
        swarm_p = it.second.traj.getPos(it.second.duration) +
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

    pnh.param("optimization/constraint_points_perPiece", cps_num_perPiece_, -1);
    pnh.param("optimization/weight_spherical_bounds", wei_sph_bounds_, -1.0);
    pnh.param("optimization/weight_obstacle", wei_obs_, -1.0);
    pnh.param("optimization/weight_obstacle_soft", wei_obs_soft_, -1.0);
    pnh.param("optimization/weight_swarm", wei_swarm_, -1.0);
    pnh.param("optimization/weight_feasibility", wei_feas_, -1.0);
    pnh.param("optimization/weight_sqrvariance", wei_sqrvar_, -1.0);
    pnh.param("optimization/weight_time", wei_time_, -1.0);
    pnh.param("optimization/obstacle_clearance", obs_clearance_, -1.0);
    pnh.param("optimization/obstacle_clearance_soft", obs_clearance_soft_, -1.0);
    pnh.param("optimization/swarm_clearance", swarm_clearance_, -1.0);
    pnh.param("optimization/max_vel", max_vel_, -1.0);
    pnh.param("optimization/max_acc", max_acc_, -1.0);
  }

  void PolyTrajOptimizer::setEnvironment(const std::shared_ptr<GridMap> &map)
  {
    grid_map_ = map;

    a_star_.reset(new AStar);
    a_star_->initGridMap(grid_map_, Eigen::Vector3i(200, 200, 200));
  }

  void PolyTrajOptimizer::setVisualizer(PlanningVisualization::Ptr vis)
  {
    visualization_ = vis;
  }

  void PolyTrajOptimizer::assignSwarmTrajs(
    std::shared_ptr<std::unordered_map<int, ego_planner::LocalTrajData>> swarm_local_trajs) {
    swarm_local_trajs_ = swarm_local_trajs;
  }

  void PolyTrajOptimizer::setSwarmTrajs(SwarmTrajData *swarm_trajs_ptr) { 
    swarm_trajs_ = swarm_trajs_ptr; 
  }

  void PolyTrajOptimizer::setDroneId(const int drone_id) { 
    drone_id_ = drone_id; 
  }

  void PolyTrajOptimizer::setIfTouchGoal(const bool touch_goal) { 
    touch_goal_ = touch_goal; 
  }

  int PolyTrajOptimizer::earlyExitCallback(void *func_data, const double *x, const double *g, const double fx, const double xnorm, const double gnorm, const double step, int n, int k, int ls)
  {
    PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(func_data);

    return (opt->force_stop_type_ == STOP_FOR_ERROR || opt->force_stop_type_ == STOP_FOR_REBOUND);
  }


  /* callbacks by the L-BFGS optimizer */
  double PolyTrajOptimizer::costFunctionCallback(void *func_data, const double *x, double *grad, const int n)
  {
    // // Pointer to current instance of PolyTrajOptimizer
    // PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(func_data);

    // opt->min_ellip_dist2_ = std::numeric_limits<double>::max();

    // // x is pointer to start of the array/matrix
    // Eigen::Map<const Eigen::MatrixXd> P(x, 3, opt->num_segs_ - 1); // 3 x (M-1)
    // // Eigen::VectorXd T(Eigen::VectorXd::Constant(piece_nums, opt->t2T(x[n - 1]))); // same t
    // // we specify "x + (3 * (opt->num_segs_ - 1))" because that is the address, its not the value
    // Eigen::Map<const Eigen::VectorXd> t(x + (3 * (opt->num_segs_ - 1)), opt->num_segs_);

    // // Gradient values of P and t
    // Eigen::Map<Eigen::MatrixXd> gradP(grad, 3, opt->num_segs_ - 1);
    // Eigen::Map<Eigen::VectorXd> gradt(grad + (3 * (opt->num_segs_ - 1)), opt->num_segs_);

    // // from virtual time t to real time T
    // Eigen::VectorXd T(opt->num_segs_);
    // opt->VirtualT2RealT(t, T);

    // Eigen::VectorXd gradT(opt->num_segs_); // P.D. of costs w.r.t time t, A vector of size (M, 1)
    // double jerk_cost = 0, time_cost = 0;
    // Eigen::VectorXd obs_swarm_feas_qvar_costs(4); // Vector of costs containing (Static obstacles, swarm, dynamic, feasibility, qvar)

    // opt->mjo_xi_.generate(P, T);

    // /* 1. Smoothness cost */
    // // jerk_cost is the cost of the jerk minimization trajectory
    // opt->mjo_xi_.initGradCost(gradT, jerk_cost); // does addGradJbyT(gdT) and addGradJbyC(gdC);

    // /** 2. Time integral cost 
    //   *   2a. Static obstacle cost
    //   *   2b. Swarm cost
    //   *   2c. Dynamical feasibility
    //   *   2d. Uniformity of points
    // */
    // opt->addPVAGradCost2CT(gradT, obs_swarm_feas_qvar_costs, opt->cps_num_perPiece_); 

    // if (opt->iter_num_ > 3 && jerk_cost / opt->num_segs_ < 10.0) // 10.0 is an experimental value that indicates the trajectory is smooth enough.
    // {
    //   opt->roughlyCheckConstraintPoints();
    // }

    // opt->mjo_xi_.getGrad2TP(gradT, gradP);
    // // time_cost += opt->rho_ * T(0) * piece_nums;  // same t
    // // grad[n - 1] = (gradT.sum() + opt->rho_ * piece_nums) * gdT2t(x[n - 1]);  // same t

    // opt->VirtualTGradCost(T, t, gradT, gradt, time_cost);

    // opt->iter_num_ += 1;
    // return jerk_cost + obs_swarm_feas_qvar_costs.sum() + time_cost;

    return 0;
  }

} // namespace ego_planner