#ifndef _SFC_BASE_H_
#define _SFC_BASE_H_

#include <Eigen/Eigen>
#include <memory>

#include <grid_map/grid_map.h>

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

    Eigen::VectorXd const getSegmentTimeDurations() const {
      return Eigen::VectorXd::Map(segs_t_dur.data(), static_cast<Eigen::Index>(segs_t_dur.size()));
    }

    std::vector<double> const getSegmentDurations() const {
      return segs_t_dur;
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

    /**
     * @brief Get the number of segments that have been traversed since start of plan until the given time,
     * INCLUDING the segment it is currently traversing. That is to say if I am travelling on the first segment,
     * it will return 1
     * 
     * @param time_now 
     * @return int 
     */
    int getNumSegmentsTraversed(const double& time_now){
      int segments_traversed = 0; // Number of segments traversed given time_now
      
      {
        double et_since_plan_start = time_now - start_time; // elapsed time since start of previous plan
        double et_sfc_segments = 0.0; // Cumulative time elapsed from iterative summing of sfc segments 
        for (auto& seg_dur : segs_t_dur){
          if (et_sfc_segments > et_since_plan_start){ // Cumul. sum of trajectories exceed the time elapsed
            break;
          }
          et_sfc_segments += seg_dur;
          segments_traversed++;
        } 
      }

      return segments_traversed;
    }

    /**
     * @brief Get maximum waypoint index within a given distance of the given point pt
     * 
     * @param time_now 
     * @return int Maximum waypoint index within a distance of the given point pt
     */
    int getMaxWaypointWithinDist(const Eigen::Vector3d& pt, const double& dist){
      int max_wp_idx = -1;

      for (const auto& wp: waypoints)
      {
        if ((wp - pt).norm() >= dist){
          break;
        }
        max_wp_idx++;
      }

      return max_wp_idx;
    }

    /**
     * @brief keep a given number of segments from start_idx to end_idx (INCLUSIVE)
     * 
     * @param num_seg_prunes 
     */
    void keep(const int& end_idx) {

      if ((end_idx + 1) >= getNumSegments() ){
        std::cout << "[SSFC] Error pruning SSFC, number of segments to be kept exceeds total" << std::endl;
        return;
      }

      spheres.erase(spheres.begin()+end_idx+1, spheres.end());
      waypoints.erase(waypoints.begin()+end_idx+1, waypoints.end());
      segs_t_dur.erase(segs_t_dur.begin()+end_idx+1, segs_t_dur.end());
      intxn_spheres.erase(intxn_spheres.begin()+end_idx+1, intxn_spheres.end());
      intxn_plane_vec.erase(intxn_plane_vec.begin()+end_idx+1, intxn_plane_vec.end());
      intxn_plane_dist.erase(intxn_plane_dist.begin()+end_idx+1, intxn_plane_dist.end());
      intxn_circle_radius.erase(intxn_circle_radius.begin()+end_idx+1, intxn_circle_radius.end());
    }

    /**
     * @brief Prune a given number of segments from start_idx to end_idx (EXCLUSIVE OF END_IDX)
     * 
     * @param start_idx 
     * @param end_idx 
     */
    void prune(const int& start_idx, const int& end_idx) {

      if ((end_idx-start_idx)+1 >= getNumSegments()){
        std::cout << "[SSFC] Error pruning SSFC, number to be segments to be pruned exceeds total" << std::endl;
        return;
      }

      spheres.erase(spheres.begin()+start_idx, spheres.begin()+end_idx);
      waypoints.erase(waypoints.begin()+start_idx, waypoints.begin()+end_idx);
      segs_t_dur.erase(segs_t_dur.begin()+start_idx, segs_t_dur.begin()+end_idx);
      intxn_spheres.erase(intxn_spheres.begin()+start_idx, intxn_spheres.begin()+end_idx);
      intxn_plane_vec.erase(intxn_plane_vec.begin()+start_idx, intxn_plane_vec.begin()+end_idx);
      intxn_plane_dist.erase(intxn_plane_dist.begin()+start_idx, intxn_plane_dist.begin()+end_idx);
      intxn_circle_radius.erase(intxn_circle_radius.begin()+start_idx, intxn_circle_radius.begin()+end_idx);
    }

    double start_time; // Trajectory start time
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
  virtual bool generateSFC( const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &path,
                            const bool& enable_rhc_plan,
                            const double& rhc_dist,
                            const Eigen::Vector3d& start_pos,
                            const double& req_plan_time) = 0;

  /**
   * @brief Add ROS Publishers to the SFCBase class
   * 
   * @param publisher_map 
   */
  virtual void addPublishers(std::unordered_map<std::string, ros::Publisher> &publisher_map) = 0;

  // Get Spherical SFC Trajectory
  virtual SSFC::SFCTrajectory const getSSFCTrajectory(const double& traj_start_time) = 0;

  SSFC::SFCTrajectory sfc_traj_;          // SFC Trajectory 

protected:
  SFCBase(){};
}; // class SFCBase

#endif // _SFC_BASE_H_