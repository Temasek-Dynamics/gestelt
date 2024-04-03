#ifndef _POLYTOPE_SFC_H_
#define _POLYTOPE_SFC_H_

#include <sfc_generation/sfc_base.h>

#include <jps_basis/data_type.h>
#include <jps_collision/map_util.h>

#include <decomp_geometry/polyhedron.h>
#include <convex_decomp.hpp>

#include <queue>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

class PolytopeSFC : public SFCBase
{
public: // Public structs

  enum CVXDecompType
  {
    LIU,        // Use Liu's convex decomposition method
    TOUMIEH_OLD,  // Use Toumieh's old convex decomposition method
    TOUMIEH_NEW,  // Use Toumieh's new convex decomposition method
  };

  struct PolytopeSFCParams{
    /* SFC Generation */
    int max_itr; // Corresponds to maximum number of spheres
    bool debug_viz; // If true, publish visualization for debugging

    // /* Time allocation */
    double max_vel; // Maximum velocity
    double max_acc; // Maximum acceleration

  }; // struct PolytopeSFCParams

  PolytopeSFC(std::shared_ptr<GridMap> grid_map, const PolytopeSFCParams& sfc_params);


  /**
   * @brief Generate a spherical safe flight corridor given a path
   * 
   * @param path 
   * @return true 
   * @return false 
   */
  bool generateSFC(const std::vector<Eigen::Vector3d> &path);

  /* Getter methods */

private: // Private methods

  /* Visualization methods */

private: // Private members
  std::shared_ptr<GridMap> map_; // Pointer to occupancy voxel map

  /* Params */
  PolytopeSFCParams params_; // SFC parameters
  int poly_hor_; // number of polyhedra to consider at each planning iteration
  CVXDecompType cvx_decomp_type_{CVXDecompType::TOUMIEH_NEW};
  int n_it_decomp_{60};  // no. of expansion iterations to generate the convex polyhedron

  /* Data structs */
  std::vector<LinearConstraint3D> poly_const_vec_;  // Polytope constraints
  std::vector<Polyhedron3D> poly_vec_;                        // Polytope vectors
  std::vector<::std::vector<double>> poly_seeds_;       // Polytope seeds

  // vector to save the polyhedra that were in the optimization; it is resized
  // to size poly_hor_ and if ith idx is true, it means we are using the ith poly
  std::vector<bool> poly_used_idx_;                   



}; // class PolytopeSFC

#endif // _POLYTOPE_SFC_H_