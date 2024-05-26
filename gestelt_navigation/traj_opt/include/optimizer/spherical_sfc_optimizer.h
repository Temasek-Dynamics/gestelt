#ifndef _SPHERICAL_SFC_OPTIMIZER_H_
#define _SPHERICAL_SFC_OPTIMIZER_H_

#include <Eigen/Eigen>
#include <ros/ros.h>
#include <grid_map/grid_map.h>
#include <path_searching/dyn_a_star.h>
#include <traj_utils/plan_container.hpp>
#include <traj_utils/planning_visualization.h>
#include "optimizer/lbfgs.hpp"
#include "poly_traj_utils.hpp"

namespace back_end
{
  struct OptCosts{
    /* Spatial */
    std::vector<double> jerk;
    std::vector<double> obs;
    std::vector<double> swarm_avoidance;
    std::vector<double> vel_acc_penalty;
    std::vector<double> dist_var;
    /* Temporal */
    std::vector<double> time_cost;
    
    void addCosts(
      const double& jerk_, 
      const double& obs_, 
      const double& swarm_avoidance_, 
      const double& vel_acc_penalty_,
      const double& dist_var_,
      const double& time_cost_)
    {
      jerk.push_back(jerk_);
      obs.push_back(obs_);
      swarm_avoidance.push_back(swarm_avoidance_);
      vel_acc_penalty.push_back(vel_acc_penalty_);
      dist_var.push_back(dist_var_);
      time_cost.push_back(time_cost_);
    }

    void printAll()
    { 
      std::cout<< "Number of iterations: " << jerk.size() << std::endl;

      std::cout<< "obs: " << std::endl;
      for (size_t i = 0; i < obs.size(); i++){
        std::cout << obs[i] << ", ";
      }
      std::cout << std::endl;

      std::cout<< "Jerk: " << std::endl;
      for (size_t i = 0; i < jerk.size(); i++){
        std::cout << jerk[i] << ", ";
      }
      std::cout << std::endl;

      std::cout<< "vel_acc_penalty: " << std::endl;
      for (size_t i = 0; i < vel_acc_penalty.size(); i++){
        std::cout << vel_acc_penalty[i] << ", ";
      }
      std::cout << std::endl;

      std::cout<< "dist_var: " << std::endl;
      for (size_t i = 0; i < dist_var.size(); i++){
        std::cout << dist_var[i] << ", ";
      }
      std::cout << std::endl;

      std::cout<< "time_cost: " << std::endl;
      for (size_t i = 0; i < time_cost.size(); i++){
        std::cout << time_cost[i] << ", ";
      }
      std::cout << std::endl;

    }

    void reset()
    {
      jerk.clear();
      obs.clear();
      swarm_avoidance.clear();
      vel_acc_penalty.clear();
      dist_var.clear();
      time_cost.clear();
    }

  };

  class SphericalSFCOptimizer
  {
  public:
    typedef std::unique_ptr<SphericalSFCOptimizer> Ptr;

    /* Params */
    int cps_num_perPiece_;   // number of distinctive constraint points per piece

    /* Data structures */
    std::vector<Eigen::MatrixXd> intermediate_cstr_pts_xi_; // Intermediate constraint points unconstrained xi coordinates
    std::vector<Eigen::MatrixXd> intermediate_cstr_pts_q_; // Intermediate constraint points constrained q coordinates
      
    OptCosts opt_costs_; // Structure containing vector of costs throughout all iterations

  private:
    std::shared_ptr<GridMap> grid_map_;
    AStar::Ptr a_star_;
    ego_planner::PlanningVisualization::Ptr visualization_;

    poly_traj::MinJerkOpt mjo_q_;   // Minimum jerk trajectory in q space
    poly_traj::MinJerkOpt mjo_xi_;  // Minimum jerk trajectory in xi space

    std::shared_ptr<std::vector<ego_planner::LocalTrajData>> swarm_local_trajs_; // Swarm MINCO trajectories

    int drone_id_;            // ID of drone
    int variable_num_;       // optimization variables
    int num_segs_;          // poly traj piece numbers
    int iter_num_;           // iteration of the solver
    double min_ellip_dist2_; // min trajectory distance in swarm

    enum FORCE_STOP_OPTIMIZE_TYPE
    {
      DONT_STOP,
      STOP_FOR_REBOUND,
      STOP_FOR_ERROR
    } force_stop_type_;

    /* optimization parameters */
    double wei_sph_bounds_;                                       // Weight to keep trajectory within bounds of sphere
    double wei_obs_;                                              // obstacle weight
    double wei_swarm_;                                            // swarm weight
    double wei_feas_;                                             // feasibility weight
    double wei_sqrvar_;                                           // squared variance weight
    double wei_time_;                                             // time weight
    double obs_clearance_, swarm_clearance_; // safe distance
    double max_vel_, max_acc_;                                    // dynamic limits

    double t_now_;

    /* Parameters for barrier function */
    double bar_alp_{-1.0};
    double bar_buf_{-1.0};
    double wei_barrier_{-1.0};

    /* Data structures */
    std::vector<Eigen::Vector3d> spheres_center_;   // Vector of sphere centers, size is no. of segments/pieces
    std::vector<double> spheres_radius_;            // Vector of sphere radius, size is no. of segments/pieces
    std::vector<Eigen::Vector3d> intxn_plane_vec_;  // Vector to center of spherical cap (the intersection between 2 spheres)
    std::vector<double> intxn_plane_dist_;          // Distance to center of spherical cap (the intersection between 2 spheres)
    std::vector<Eigen::Vector3d> intxn_center_;     // Center of intersection
    std::vector<double> intxn_circle_radius_;        // 


    Eigen::MatrixXd cstr_pts_xi_; // inner CONSTRAINT points of trajectory (excludes boundary points), this is finer than the inner CONTROL points
    Eigen::MatrixXd cstr_pts_q_; // inner CONSTRAINT points of trajectory (excludes boundary points), this is finer than the inner CONTROL points

    Eigen::MatrixXd ctrl_pts_q_optimal_; // Optimized control points in q


  public:
    SphericalSFCOptimizer(){}
    ~SphericalSFCOptimizer() {}

    /* set variables */
    void setParam(ros::NodeHandle &pnh);
    void setEnvironment(const std::shared_ptr<GridMap> &map);
    void setVisualizer(ego_planner::PlanningVisualization::Ptr vis);

    void assignSwarmTrajs(std::shared_ptr<std::vector<ego_planner::LocalTrajData>> swarm_local_trajs);

    /**
     * Returns the minimum jerk optimizer object
    */
    const poly_traj::MinJerkOpt &getOptimizedMJO(void) { 
      return mjo_q_; 
    }

    /**
     * Returns the minimum jerk optimizer object
    */
    const Eigen::MatrixXd &getOptimizedCtrlPts(void) { 
      return ctrl_pts_q_optimal_; 
    }

    /**
     * @brief Get the user-defined value for number of constraint points per segment.
     * 
     * @return int 
     */
    int getNumCstrPtsPerSeg(void) { return cps_num_perPiece_; }

    /**
     * @brief Get the user-defined swarm clearance parameter
     * 
     * @return double 
     */
    double getSwarmClearance(void) { return swarm_clearance_; }

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
    bool optimizeTrajectory( const Eigen::Matrix3d &startPVA, const Eigen::Matrix3d &endPVA,
                                const Eigen::MatrixXd &inner_ctrl_pts, const Eigen::VectorXd &initT,
                                const std::vector<Eigen::Vector3d>& spheres_center, const std::vector<double>& spheres_radius, 
                                const std::vector<Eigen::Vector3d>& spheres_intxn_plane_vec, const std::vector<double>& spheres_intxn_plane_dist,
                                const std::vector<Eigen::Vector3d>& sfc_traj_waypoints, const std::vector<double>& intxn_circle_radius,
                                double &final_cost);

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

    /**
     * @brief Convert from real to virtual time
     * 
     * @tparam EIGENVEC 
     * @param RT 
     * @param VT 
     */
    template <typename EIGENVEC>
    void RealT2VirtualT(const Eigen::VectorXd &RT, EIGENVEC &VT);

    /**
     * @brief Convert from virtual to real time
     * 
     * @tparam EIGENVEC 
     * @param RT 
     * @param VT 
     */
    template <typename EIGENVEC>
    void VirtualT2RealT(const EIGENVEC &VT, Eigen::VectorXd &RT);

    template <typename EIGENVEC, typename EIGENVECGD>
    void VirtualTGradCost(const Eigen::VectorXd &RT, const EIGENVEC &VT,
                          const Eigen::VectorXd &gdRT, EIGENVECGD &gdVT,
                          double &costT);

    /**
     * @brief Get cost for constraints on PVA
     * 
     * @tparam EIGENVEC 
     * @param gdT Gradient of size of number of pieces
     * @param costs a vector of costs
     * @param K Constraint points per piece, or total sample number
     */
    template <typename EIGENVEC>
    void addPVAGradCost2CT(
      EIGENVEC &gdT, Eigen::VectorXd &costs, const int &K, poly_traj::MinJerkOpt& mjo);

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

  /* Constraint elimination methods */
  public:

    /**
     * @brief Stereographic projection from (n) hyperplane onto (n+1) sphere and then orthographic projection onto (n) circle 
     * 
     * @param xi_gbl  Decision variable in global coordinates
     * @param O   Center of sphere 
     * @param r   radius of sphere
     * @param N Normal to intersection plane
     * @param f_dist offset distance to intersection plane from sphere center
     * @return Eigen::Vector3d 
     */
    Eigen::Vector3d f_B(
      const Eigen::Vector3d& xi_gbl, 
      const Eigen::Vector3d& O, 
      const double& r,
      const Eigen::Vector3d& N)
    {
      // /* 1. Assuming north pole sphere */
      Eigen::Vector3d xi = xi_gbl - O;
      Eigen::Vector3d P =     xi * (2 * (r*r) ) / (xi.squaredNorm() + r*r);

      return  O + P;
      
      /* 2. Project to plane passing through center of intersection sphere*/
      // Normalized normal vector to plane
      // Eigen::Vector3d N_norm = N.normalized();

      // Eigen::Vector3d xi = xi_gbl - O; // Get xi relative to cneter of sphere
      // // xi = xi - (xi).transpose().dot(N_norm) * N_norm; // Get xi projected to plane

      // Eigen::Vector3d P = N - (xi - N) * (2 * N.dot(xi-N) )/ ((xi-N).squaredNorm()) ;

      // Eigen::Vector3d Q = P - P.dot(N_norm) * N_norm;

      // return  O + Q;
    }

    /**
     * @brief Transformation from decision variable xi to variable q
     * Part of constraint elimination concept.
     * @param xi Decision variables to be optimized
     * @return eigen::Vector 
     */
    Eigen::MatrixXd f_B_ctrl_pts(
      const Eigen::MatrixXd& xi, 
      const std::vector<Eigen::Vector3d>& spheres_center,  const std::vector<double>& sphere_radius,
      const std::vector<Eigen::Vector3d>& intxn_plane_vec, const std::vector<double>& intxn_plane_dist,
      const std::vector<Eigen::Vector3d>& intxn_center, const std::vector<double>& intxn_circle_radius)
    {
      // Expects array of size (3, M-1)
      size_t M = xi.cols() + 1; // Number of segments

      Eigen::MatrixXd q(3, M-1);

      //for each segment i (excluding boundary points)
      for (size_t i = 0; i < M-1; i++)
      {
        double r =            intxn_circle_radius[i];
        Eigen::Vector3d O =   intxn_center[i];
        auto xi_gbl =         xi.block<3,1>(0, i);
        Eigen::Vector3d N =   intxn_plane_vec[i];

        q.block<3,1>(0, i) =  f_B(xi_gbl, O, r, N);
      }

      return q;
    }


    /**
     * @brief Constrain points to intersection plane
     */
    Eigen::MatrixXd constrainToIntxnPlane(
      const Eigen::MatrixXd& xi, 
      const std::vector<Eigen::Vector3d>& intxn_plane_vec, 
      const std::vector<Eigen::Vector3d>& intxn_center)
    {
      // Expects array of size (3, M-1)
      size_t M = xi.cols() + 1; // Number of segments

      Eigen::MatrixXd q(3, M-1);

      //for each segment i (excluding boundary points)
      for (size_t i = 0; i < M-1; i++)
      {
        Eigen::Vector3d O =   intxn_center[i];
        auto xi_gbl =         xi.block<3,1>(0, i);
        Eigen::Vector3d N =   intxn_plane_vec[i];

        Eigen::Vector3d N_norm = N.normalized();
        Eigen::Vector3d Q = xi_gbl - (xi_gbl - O).dot(N_norm) * N_norm;

        q.block<3,1>(0, i) =  Q;
      }

      return q;
    }


    // /**
    //  * @brief Inverse orthographic projection from (n) circle to (n+1) sphere
    //  * then inverse stereographic projection to (n) hyperplane
    //  * 
    //  * @param xi  Decision variable
    //  * @param o   Center of sphere 
    //  * @param r   radius of sphere
    //  * @param f   offset plane distance from sphere center
    //  * @return Eigen::MatrixXd 
    //  */
    // Eigen::Vector3d f_B_inv(
    //   const Eigen::Vector3d& q, 
    //   const Eigen::Vector3d& o, 
    //   const double& r,
    //   const Eigen::Vector3d& f_vec, 
    //   const double& f_dist)
    // {
    //   /* 1. Assuming north pole sphere */
    //   auto v = q - o; // Vector from center of sphere to point q
    //   auto b = r - sqrt( r*r - v.squaredNorm());

    //   return o + v * (r/b);

    //   /* 2. Assuming north pole sphere but with offset plane */
    //   // auto v = q - o; // Vector from center of sphere to point q
    //   // auto b = r - sqrt( r*r - v.squaredNorm())- f_dist;

    //   // return o + v * ((r - f_dist)/b);

    //   /* 3. With arbitary vector (with length of sphere's radius) to project onto sphere */ 

    //   // // v: q relative to center of sphere
    //   // auto v = q - o; 
    //   // // t: scalar to scale the vector 
    //   // double t = (f_vec(2))/( f_vec(2) - sqrt( r*r - (v).squaredNorm()) );

    //   // // pt_wrt_o: point with respect to center of sphere
    //   // Eigen::Vector3d pt_wrt_o = f_vec + t * (v - f_vec);

    //   // // pt: point with respect to global origin
    //   // Eigen::Vector3d pt = o + pt_wrt_o;

    //   // return pt;
    // }

  }; // class SphericalSFCOptimizer

} // namespace back_end

#endif //_SPHERICAL_SFC_OPTIMIZER_H_