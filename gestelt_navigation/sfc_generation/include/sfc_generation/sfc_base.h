#ifndef _SFC_BASE_H_
#define _SFC_BASE_H_

#include <Eigen/Eigen>
#include <memory>

#include <grid_map/grid_map.h>

#include <decomp_geometry/polyhedron.h>

namespace SSFC{

  struct Sphere {
    double radius{-1.0};
    double radius_sqr{-1.0};
    Eigen::Vector3d center{0.0, 0.0, 0.0};
    double score{-1.0};
    double volume{-1.0};

    Sphere(){}

    Sphere(const double& x, const double& y, const double& z, const double& radius):
      center(Eigen::Vector3d{x,y,z}), radius(radius), radius_sqr(radius*radius), volume(1.333 * M_PI * radius * radius * radius)
    {}

    Sphere(const Eigen::Vector3d& center, const double& radius):
      center(center), radius(radius), radius_sqr(radius*radius), volume(1.333 * M_PI * radius * radius * radius)
    {}

    // Copy constructor from shared_ptr
    Sphere(const std::shared_ptr<Sphere>& sphere): 
      radius(sphere->radius), 
      radius_sqr(sphere->radius_sqr), 
      center(sphere->center),
      score(sphere->score),
      volume(sphere->volume)
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
      return (this->center - pt).squaredNorm() <= this->radius_sqr;
    }

    double getDiameter() const
    {
      return this->radius * 2;
    }

    double getVolume() const
    {
      return this->volume;
    }

    void setRadius(const double& radius)
    {
      this->radius = radius;
      this->radius_sqr = radius * radius;
      this->volume = 1.333 * M_PI * radius * radius * radius;
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

  struct SFCTrajectory{ // SFCTrajectory contains the spheres, trajectory waypoints and time allocation
    SFCTrajectory(){} // Default constructor

    /**
     * @brief Clear all data structures
     * 
     */
    void reset(){
      spheres.clear();
      segs_t_dur.clear();
      waypoints.clear();
    }

    Eigen::Vector3d  getStartPos() const {
      if (waypoints.size() < 2){
        throw std::runtime_error("SFCTrajectory does not contain at least 2 waypoints");
      }
      return waypoints[0];
    }

    Eigen::Vector3d  getGoalPos() const {
      if (waypoints.size() < 2){
        throw std::runtime_error("SFCTrajectory does not contain at least 2 waypoints");
      }
      return waypoints.back();
    }

    std::vector<double> getSpheresRadii() const {
      std::vector<double> spheres_radii;

      for (auto sphere : spheres){
        spheres_radii.push_back(sphere.radius);
      }

      return spheres_radii;
    }

    std::vector<Eigen::Vector3d> getSpheresCenter() const {
      std::vector<Eigen::Vector3d> spheres_center;

      for (auto sphere : spheres){
        spheres_center.push_back(Eigen::Vector3d{sphere.center(0), sphere.center(1), sphere.center(2)});
      }

      return spheres_center;
    }

    Eigen::MatrixXd getInnerWaypoints() const {
      if (waypoints.size() <= 2){
        Eigen::MatrixXd inner_wps; // matrix of inner waypoints
        return inner_wps;
      }
      Eigen::MatrixXd inner_wps(3, waypoints.size()-2); // matrix of inner waypoints
      
      for (size_t i = 1, inner_wp_idx = 0 ; i < waypoints.size()-1; i++, inner_wp_idx++){
        inner_wps.col(inner_wp_idx) = waypoints[i];
      }

      return inner_wps;
    }

    Eigen::VectorXd getSegmentTimeDurations() const {
      return Eigen::VectorXd::Map(segs_t_dur.data(), static_cast<Eigen::Index>(segs_t_dur.size()));
    }

    std::vector<Eigen::Vector3d> getIntxnPlaneVec() const{
      return intxn_plane_vec;
    }

    std::vector<double> getIntxnPlaneDist() const{
      return intxn_plane_dist;
    }
    std::vector<double> getIntxnCircleRadius() const{
      return intxn_circle_radius;
    }

    std::vector<Eigen::Vector3d> getIntxnCenters() const {
      if (waypoints.size() <= 2){
        return waypoints;
      }
      std::vector<Eigen::Vector3d> inner_waypoints;      
      inner_waypoints.resize(waypoints.size()-2);
      
      for (size_t i = 1, inner_wp_idx = 0 ; i < waypoints.size()-1; i++, inner_wp_idx++){
        inner_waypoints[inner_wp_idx] = waypoints[i];
      }

      return inner_waypoints;
    }

    int getNumSegments() const{
      return spheres.size();
    }

    std::vector<SSFC::Sphere> spheres;  // Vector of Spheres 
    std::vector<Eigen::Vector3d> waypoints;     // Vector of 3d waypoint positions {p1, p2, ... p_M+1}
    std::vector<double> segs_t_dur;             // Vector of time durations of each segment {t_1, t_2, ..., t_M}

    std::vector<SSFC::Sphere> intxn_spheres;
    std::vector<Eigen::Vector3d> intxn_plane_vec;       // Vector to center of spherical cap (the intersection between 2 spheres)
    std::vector<double> intxn_plane_dist;       // Distance to center of spherical cap (the intersection between 2 spheres)
    std::vector<double> intxn_circle_radius;       // Radius of intersection circle.


  }; // struct SFCTrajectory

}

class SFCBase
{
public:
  /**
   * @brief Generate a safe flight corridor
   * 
   * @param path 
   * @return true 
   * @return false 
   */
  virtual bool generateSFC(const std::vector<Eigen::Vector3d> &path) = 0;

  /**
   * @brief Add ROS Publishers to the SFCBase class
   * 
   * @param publisher_map 
   */
  virtual void addPublishers(std::unordered_map<std::string, ros::Publisher> &publisher_map) = 0;

  // Get Spherical SFC Trajectory
  virtual SSFC::SFCTrajectory const getSSFCTrajectory() = 0;

  // Get Polytope SFC Trajectory
  virtual std::vector<Polyhedron3D, Eigen::aligned_allocator<Polyhedron3D>> getPolySFC() = 0;

protected:
  SFCBase(){};
}; // class SFCBase

#endif // _SFC_BASE_H_