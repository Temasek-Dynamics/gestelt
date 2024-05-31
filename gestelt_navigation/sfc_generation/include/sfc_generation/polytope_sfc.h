#ifndef _POLYTOPE_SFC_H_
#define _POLYTOPE_SFC_H_

#include <Eigen/Eigen>
#include <grid_map/grid_map.h>

#include <jps_basis/data_type.h>
#include <jps_collision/map_util.h>

#include <decomp_geometry/polyhedron.h>
#include <convex_decomp.hpp>
#include <decomp_ros_msgs/PolyhedronArray.h>
#include <decomp_ros_utils/data_ros_utils.h>

#include <queue>
#include <unordered_map>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "geo_utils.hpp"
// #include "firi.hpp"
// #include "sfc_gen.hpp"

class PolytopeSFC 
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
    bool debug_viz; // If true, publish visualization for debugging

    std::string world_frame{"world"};
    int poly_hor{-1}; // number of polyhedra to consider at each planning iteration
    CVXDecompType cvx_decomp_type{CVXDecompType::TOUMIEH_NEW};
    int n_it_decomp{60};  // no. of expansion iterations to generate the convex polyhedron

  }; // struct PolytopeSFCParams

  // Constructor
  PolytopeSFC(std::shared_ptr<GridMap> grid_map, const PolytopeSFCParams& sfc_params);

  // Reset all data structures
  void reset();

  /**
   * @brief Generate a spherical safe flight corridor given a path
   * 
   * @param path 
   * @return true 
   * @return false 
   */
  bool generateSFC(const std::vector<Eigen::Vector3d> &path,
                    const bool& enable_rhc_plan,
                    const double& rhc_dist,
                    const Eigen::Vector3d& start_pos,
                    const double& req_plan_time);

  void addPublishers(std::unordered_map<std::string, ros::Publisher> &publisher_map);

  /* Getter methods */
  std::vector<::std::vector<double>> getPolySeeds() const {
    return poly_seeds_;
  }
  std::vector<LinearConstraint3D> getPolyConstraints() const {
    return poly_constr_vec_;
  }

  // std::vector<Polyhedron3D, Eigen::aligned_allocator<Polyhedron3D>> getPolySFCHyperplanes() const {
  //   return poly_vec_;
  // }

 std::vector<Eigen::MatrixX4d> getPolySFCHyperplanes() const {
    return poly_vec_hyp_;
  }

  // Return matrix of Size (3, M). Representing an array of vertex-based polyhedrons.
  std::vector<Eigen::Matrix3Xd> getPolySFCVertices() const 
  {
    return poly_vec_vtx_;
  }



  bool processCorridor(const std::vector<Eigen::MatrixX4d> &hPs,
                                      std::vector<Eigen::Matrix3Xd> &vPs)
  {
      const int sizeCorridor = hPs.size() - 1;

      vPs.clear();
      vPs.reserve(2 * sizeCorridor + 1);

      int nv;
      Eigen::MatrixX4d curIH;
      Eigen::Matrix3Xd curIV, curIOB;
      for (int i = 0; i < sizeCorridor; i++) // For each halfspace polyhedron
      {
          if (!geo_utils::enumerateVs(hPs[i], curIV)) // get vertex polyhedron of size (3, M)
          {
            return false;
          }
          nv = curIV.cols(); // num vertices
          curIOB.resize(3, nv); // curIOB: current polyhedron in barycentric coordinates
          curIOB.col(0) = curIV.col(0);
          curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
          vPs.push_back(curIOB);
          
          // curIH: polyhedron formed from intersections between 2 consecutive polyhedrons
          curIH.resize(hPs[i].rows() + hPs[i + 1].rows(), 4);
          curIH.topRows(hPs[i].rows()) = hPs[i];
          curIH.bottomRows(hPs[i + 1].rows()) = hPs[i + 1];
          if (!geo_utils::enumerateVs(curIH, curIV))
          {
            return false;
          }
          nv = curIV.cols();
          curIOB.resize(3, nv);
          curIOB.col(0) = curIV.col(0);
          curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
          vPs.push_back(curIOB);
      }

      if (!geo_utils::enumerateVs(hPs.back(), curIV))
      {
        return false;
      }
      nv = curIV.cols();
      curIOB.resize(3, nv);
      curIOB.col(0) = curIV.col(0);
      curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
      vPs.push_back(curIOB);

      return true;
  }

private: // Private methods
  /* Visualization methods */
  void PublishPolyhedra(
      const std::vector<Polyhedron3D, Eigen::aligned_allocator<Polyhedron3D>>& poly_vec);

private: // Private members
  std::shared_ptr<GridMap> map_; // Pointer to occupancy voxel map

  /* ROS */
  ros::Publisher poly_sfc_pub_;

  /* Params */
  PolytopeSFCParams params_; // SFC parameters

  /* Data structs */
  std::vector<LinearConstraint3D> poly_constr_vec_;  // Polytope constraints
  std::vector<Polyhedron3D, Eigen::aligned_allocator<Polyhedron3D>> poly_vec_; // Vector of polyhedrons represented as hyperplanes
  std::vector<Eigen::MatrixX4d> poly_vec_hyp_; // Matrix of size (N, 4) polyhedrons represented as vertices
  std::vector<Eigen::Matrix3Xd> poly_vec_vtx_; // Matrix of size (3, M) polyhedrons represented as vertices
  std::vector<::std::vector<double>> poly_seeds_;       // Polytope seeds

  // vector to save the polyhedra that were in the optimization; it is resized
  // to size poly_hor_ and if ith idx is true, it means we are using the ith poly
  std::vector<bool> poly_used_idx_;                   


}; // class PolytopeSFC

#endif // _POLYTOPE_SFC_H_