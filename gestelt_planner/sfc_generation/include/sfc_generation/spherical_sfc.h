#ifndef _SPHERICAL_SFC_H_
#define _SPHERICAL_SFC_H_

#include <sfc_generation/sfc_base.h>

class SphericalSFC : public SFCBase
{
public: // Public structs

  struct Sphere {
    double radius;
    double radius_sqr;
    Eigen::Vector3d center;

    Sphere(const double& radius, const double& x, const double& y, const double& z):
      radius(radius), radius_sqr(radius*radius), center(Eigen::Vector3d{x,y,z})
    {}

    Sphere(const double& radius, const Eigen::Vector3d& center):
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
    bool contains(const Eigen::Vector3d& pt)
    {
      return (center-pt).squaredNorm() <= radius_sqr;
    }
  };


public:
  SphericalSFC(std::shared_ptr<GridMap> grid_map);

  /**
   * @brief Clear existing data structures
   * 
   */
  void reset();
  
  bool generateSFC(const std::vector<Eigen::Vector3d> &path);

  /**
   * @brief Get waypoints of spherical safe flight corridor. 
   * The waypoints represent the center of the sphere or their intersection points 
   * 
   * @return std::vector<Eigen::Vector3d> const 
   */
  std::vector<Sphere> const SphericalSFC::getSFCWaypoints();

private: // Private methods

private: // Private members
  std::vector<Sphere> sfc_spheres_; // Waypoints of the spherical flight corridor

  /* Params */

  /* Data structs */
  std::shared_ptr<GridMap> grid_map_; 

}; // class SphericalSFC

#endif // _SPHERICAL_SFC_H_