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

    /**
     * @brief Get the diameter of the sphere
     * 
     * @return double 
     */
    double getDiameter() const
    {
      return this->radius * 2;
    }

    double getVolume() const
    {
      return 1.333 * M_PI * this->radius * this->radius * this->radius ;
    }

    /**
     * @brief Set the radius of the sphere
     * 
     * @return double 
     */
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

    Sampler()
    {
      
    }

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
    int max_itr; // maximum iterations allowed

    /* Sampling */
    int max_sample_points; // Maximum allowed sampling points
    double mult_stddev_x; // Multiplier for x standard deviation in sampling 
    double W_cand_vol; // Weight of candidate volume
    double W_intersect_vol; // Weight of intersection of volumes
  }; // struct SphericalSFCParams

public:
  SphericalSFC(std::shared_ptr<GridMap> grid_map, const SphericalSFCParams& sfc_params);

  /**
   * @brief Clear existing data structures
   * 
   */
  void reset();
    
  void addVizPublishers(
    ros::Publisher& p_cand_viz_pub, ros::Publisher& dist_viz_pub, ros::Publisher& sfc_spherical_viz_pub);

  /**
   * @brief Generate a spherical safe flight corridor given a path
   * 
   * @param path 
   * @return true 
   * @return false 
   */
  bool generateSFC(const std::vector<Eigen::Vector3d> &path);

  /**
   * @brief Get the spheres making up the safe flight corridor
   * 
   * @return std::vector<SphericalSFC::Sphere> const 
   */
  std::vector<SphericalSFC::Sphere> const getSFCWaypoints(){
      return sfc_spheres_;
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
   * @param sphere 
   * @return true 
   * @return false 
   */
  bool getForwardPointOnPath(
    const std::vector<Eigen::Vector3d> &path, size_t& start_idx, const Sphere& B);

  bool BatchSample(const Eigen::Vector3d& point, Sphere& B_cur);

  double computeCandSphereScore(Sphere& B_cand, Sphere& B_prev);

  void transformPoints(std::vector<Eigen::Vector3d>& pts, Eigen::Vector3d origin, const Eigen::Matrix<double, 3, 3>& ellipse_rot_mat);

  double getIntersectingVolume(Sphere& B_a, Sphere& B_b);

  Eigen::Matrix<double, 3, 3> rotationAlign(const Eigen::Vector3d & z, const Eigen::Vector3d & d);

  void publishVizPoints(const std::vector<Eigen::Vector3d>& pts, const std::string& frame_id, ros::Publisher& publisher);

  void publishVizSphericalSFC(const std::vector<SphericalSFC::Sphere>& sfc_spheres, const std::string& frame_id, ros::Publisher& publisher) {
    for (int i = 0; i < sfc_spheres.size(); i++){
      publisher.publish(createVizSphere(sfc_spheres[i].center, sfc_spheres[i].getDiameter(), frame_id, i));
    }
  }

  visualization_msgs::Marker createVizSphere(const Eigen::Vector3d& center, const double& diameter, const std::string& frame_id, const int& id)
  {
    visualization_msgs::Marker sphere;

    sphere.header.frame_id = frame_id;
    sphere.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE;
    sphere.action = visualization_msgs::Marker::ADD;
    sphere.id = id;
    sphere.pose.orientation.w = 1.0;

    sphere.color.r = 0.0;
    sphere.color.g = 0.0;
    sphere.color.b = 1.0;
    sphere.color.a = 0.5;
    sphere.scale.x = diameter;
    sphere.scale.y = diameter;
    sphere.scale.z = diameter;

    sphere.pose.position.x = center(0);
    sphere.pose.position.y = center(1);
    sphere.pose.position.z = center(2);

    return sphere;
  }

  visualization_msgs::Marker createVizEllipsoid(const Eigen::Vector3d& center, const Eigen::Vector3d& stddev, const Eigen::Quaterniond& orientation, const std::string& frame_id, const int& id);

private: // Private members
  Sampler sampler_; // Sampler for sampling SFC waypoints

  std::vector<SphericalSFC::Sphere> sfc_spheres_; // Waypoints of the spherical flight corridor

  ros::Publisher p_cand_viz_pub_; // Visualization of sampling points
  ros::Publisher dist_viz_pub_; // Visualization of sampling distribution
  ros::Publisher sfc_spherical_viz_pub_; // Visualization of spherical SFC

  /* Params */
  int itr_; // Iteration number

  SphericalSFCParams sfc_params_; // SFC parameters

  /* Data structs */
  std::shared_ptr<GridMap> grid_map_; 
  std::priority_queue<std::shared_ptr<Sphere>, std::vector<std::shared_ptr<Sphere>>, Sphere::CompareScorePtr> B_cand_pq_; // priority queue for candidate spheres in sampling phase

}; // class SphericalSFC

#endif // _SPHERICAL_SFC_H_