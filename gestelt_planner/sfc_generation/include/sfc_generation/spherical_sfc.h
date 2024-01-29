#ifndef _SPHERICAL_SFC_H_
#define _SPHERICAL_SFC_H_

#include <sfc_generation/sfc_base.h>

#include <random>
#include <chrono>
#include <queue>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

class SphericalSFC : public SFCBase
{
public: // Public structs

  struct Sphere {
    double radius;
    double radius_sqr;
    Eigen::Vector3d center;
    double score{-1.0};

    Sphere():
      radius(-1.0), radius_sqr(-1.0), center(Eigen::Vector3d{0.0, 0.0, 0.0})
    {}

    Sphere(const double& x, const double& y, const double& z, const double& radius):
      radius(radius), radius_sqr(radius*radius), center(Eigen::Vector3d{x,y,z})
    {}

    Sphere(const Eigen::Vector3d& center, const double& radius):
      radius(radius), radius_sqr(radius*radius), center(center)
    {}

    // Copy constructor from shared_ptr
    Sphere(const std::shared_ptr<Sphere>& sphere): 
      radius(sphere->radius), 
      radius_sqr(sphere->radius*sphere->radius), 
      center(sphere->center),
      score(sphere->score)
    {}

    /**
     * @brief Returns true if point is contained inside sphere (including at it's
     * boundary). Else return false
     * 
     * @param pt point to check
     * @return true 
     * @return false 
     */
    bool contains(const Eigen::Vector3d& pt) const
    {
      return (center-pt).squaredNorm() <= this->radius_sqr;
    }

    double getDiameter() const
    {
      return this->radius * 2;
    }

    double getVolume() const
    {
      return 1.333 * M_PI * this->radius * this->radius * this->radius ;
    }

    void setRadius(const double& radius)
    {
      this->radius = radius;
      this->radius_sqr = radius * radius;
    }

    // Comparison operator between pointers
    struct CompareScorePtr
    {
      bool operator()(const std::shared_ptr<Sphere>& l_sphere, const std::shared_ptr<Sphere>& r_sphere)
      {
        return l_sphere->score < r_sphere->score;
      }
    };

  }; // struct Sphere

  struct Sampler {
    std::default_random_engine gen; 

    std::normal_distribution<double> x_dis;
    std::normal_distribution<double> y_dis;
    std::normal_distribution<double> z_dis;

    Sampler(){}

    Sampler(const Eigen::Vector3d& mean, const Eigen::Vector3d& stddev)
    {
      x_dis = std::normal_distribution<double>(mean(0), stddev(0));
      y_dis = std::normal_distribution<double>(mean(1), stddev(1));
      z_dis = std::normal_distribution<double>(mean(2), stddev(2));
    }

    void setParams(const Eigen::Vector3d& mean, const Eigen::Vector3d& stddev)
    {
      x_dis = std::normal_distribution<double>(mean(0), stddev(0));
      y_dis = std::normal_distribution<double>(mean(1), stddev(1));
      z_dis = std::normal_distribution<double>(mean(2), stddev(2));
    }

    void setSeed(uint64_t seed){
      gen.seed(seed);
    }

    Eigen::Vector3d sample() {
      return Eigen::Vector3d{x_dis(gen), y_dis(gen), z_dis(gen)};
    }

  }; // struct Sampler

  struct SphericalSFCParams{
    /* SFC Generation */
    int max_itr; // Corresponds to maximum number of spheres
    bool debug_viz; // If true, publish visualization for debugging

    /* Sampling */
    int max_sample_points; // Maximum allowed sampling points
    double mult_stddev_x; // Multiplier for x standard deviation in sampling 
    double W_cand_vol;    // Weight of candidate volume
    double W_intersect_vol; // Weight of intersection of volumes

    double min_sphere_vol; // Minimum volume of sphere
    double max_sphere_vol; // Maximum volume of spheres
    double min_sphere_intersection_vol; // Minimum volume of intersection between 2 spheres

    // /* Time allocation */
    double avg_vel; // Average velocity
    double max_vel; // Maximum velocity

  }; // struct SphericalSFCParams

  struct SFCTrajectory{ // SFCTrajectory contains the spheres, trajectory waypoints and time allocation
    std::vector<SphericalSFC::Sphere> spheres;  // Vector of Spheres 
    std::vector<Eigen::Vector3d> waypoints;     // Vector of 3d waypoint positions {p1, p2, ... p_M+1}
    std::vector<double> segs_t_dur;          // Vector of time durations of each segment {t_1, t_2, ..., t_M}

    SFCTrajectory(){} // Default constructor

    void clear(){
      spheres.clear();
      segs_t_dur.clear();
      waypoints.clear();
    }

    Eigen::Vector3d const getStartPos(){
      if (waypoints.size() < 2){
        throw std::runtime_error("SFCTrajectory does not contain at least 2 waypoints");
      }
      return waypoints[0];
    }

    Eigen::Vector3d const getGoalPos(){
      if (waypoints.size() < 2){
        throw std::runtime_error("SFCTrajectory does not contain at least 2 waypoints");
      }
      return waypoints.back();
    }

  }; // struct SFCTrajectory

public:
  SphericalSFC(std::shared_ptr<GridMap> grid_map, const SphericalSFCParams& sfc_params);


  /**
   * @brief Clear existing data structures
   * 
   */
  void clear();
    
  void addVizPublishers(ros::Publisher& p_cand_viz_pub, 
    ros::Publisher& dist_viz_pub, ros::Publisher& sfc_spherical_viz_pub,
    ros::Publisher&  sfc_waypoints_viz_pub);

  /**
   * @brief Generate a spherical safe flight corridor given a path
   * 
   * @param path 
   * @return true 
   * @return false 
   */
  bool generateSFC(const std::vector<Eigen::Vector3d> &path);

  SphericalSFC::SFCTrajectory const getSFCTrajectory(){
    return sfc_traj_;
  }

private: // Private methods

  /**
   * @brief Initialize sampler 
   * 
   */
  void initSampler();

  /**
   * @brief Generate a free space sphere the given point. It's radius is determined by
   * the distance to the nearest obstacle
   * 
   * @param point 
   * @param sphere 
   * @return true 
   * @return false 
   */
  bool generateFreeSphere(const Eigen::Vector3d& point, Sphere& B);

  /**
   * @brief Get nearest point on guide path outside the given sphere
   * 
   * @param path 
   * @param start_idx 
   * @param B_prev Previous sphere 
   * @return true 
   * @return false 
   */
  bool getForwardPointOnPath(
    const std::vector<Eigen::Vector3d> &path, size_t& start_idx, const Sphere& B_prev);

  bool BatchSample(const Eigen::Vector3d& point, Sphere& B_cur);

  /**
   * @brief Get the center of the curve of intersection made between spheres B_a and B_b
   * 
   * @param B_a 
   * @param B_b 
   * @return Eigen::Vector3d 
   */
  Eigen::Vector3d getIntersectionCenter(const Sphere& B_a, const Sphere& B_b);

  /**
   * @brief Compute the score of the candidate sphere, given the previous sphere in the SFC
   * 
   * @param B_cand 
   * @param B_prev 
   * @return double 
   */
  double computeCandSphereScore(Sphere& B_cand, Sphere& B_prev);

  void transformPoints(std::vector<Eigen::Vector3d>& pts, Eigen::Vector3d origin, const Eigen::Matrix<double, 3, 3>& rot_mat);

  /**
   * @brief Get the volume of intersection between 2 spheres
   * 
   * @param B_a 
   * @param B_b 
   * @return double Returns -1 if 2 spheres do not intersect, else return volume of intersection 
   */
  double getIntersectingVolume(Sphere& B_a, Sphere& B_b);

  /**
   * @brief Get rotation matrix that aligns vector z to vector d
   * 
   * @param z 
   * @param d 
   * @return Eigen::Matrix<double, 3, 3> 
   */
  Eigen::Matrix<double, 3, 3> rotationAlign(const Eigen::Vector3d & z, const Eigen::Vector3d & d);

  /**
   * @brief Compute a SFC Trajectory given a sequence of intersecting spheres
   * 
   * @param sfc_spheres 
   * @param goal_pos 
   * @param sfc_traj 
   */
  void computeSFCTrajectory(const std::vector<SphericalSFC::Sphere>& sfc_spheres, const Eigen::Vector3d& goal_pos, SphericalSFC::SFCTrajectory& sfc_traj);

  /* Visualization methods */

  void publishVizPoints(const std::vector<Eigen::Vector3d>& pts, 
                        ros::Publisher& publisher, Eigen::Vector3d color = Eigen::Vector3d{0.0, 0.0, 0.0}, 
                        double radius = 0.025, const std::string& frame_id = "world");

  void publishVizPiecewiseTrajectory( const std::vector<Eigen::Vector3d>& pts, 
                                      ros::Publisher& publisher, const std::string& frame_id = "world");

  void publishVizSphericalSFC(const std::vector<SphericalSFC::Sphere>& sfc_spheres, 
                              ros::Publisher& publisher, const std::string& frame_id = "world");

  visualization_msgs::Marker createVizSphere( const Eigen::Vector3d& center, const double& diameter, 
                                              const std::string& frame_id, const int& id);

  visualization_msgs::Marker createVizEllipsoid(const Eigen::Vector3d& center, const Eigen::Vector3d& stddev, const Eigen::Quaterniond& orientation, const std::string& frame_id, const int& id);

private: // Private members
  Sampler sampler_; // Sampler for sampling SFC waypoints

  ros::Publisher p_cand_viz_pub_; // Visualization of sampling points
  ros::Publisher dist_viz_pub_; // Visualization of sampling distribution
  ros::Publisher sfc_spherical_viz_pub_; // Visualization of spherical SFC
  ros::Publisher sfc_waypoints_viz_pub_; // Visualization of trajectory waypoints

  /* Params */
  int itr_; // Iteration number
  SphericalSFCParams sfc_params_; // SFC parameters
  
  /* Data structs */
  std::shared_ptr<GridMap> grid_map_; 
  std::vector<SphericalSFC::Sphere> sfc_spheres_; // Waypoints of the spherical flight corridor
  SphericalSFC::SFCTrajectory sfc_traj_;          // SFC Trajectory 

}; // class SphericalSFC

#endif // _SPHERICAL_SFC_H_