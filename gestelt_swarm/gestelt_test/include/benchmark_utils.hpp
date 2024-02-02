// Code inspired by benchmark_utils.cpp from Bonxai

#ifndef _BENCHMARK_UTILS_HPP
#define _BENCHMARK_UTILS_HPP

#include <Eigen/Eigen>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <filesystem>

struct PlannerParameters {
  double voxel_resolution;  // Resolution of the voxel map
  double inflation;         // Inflation of the map
  std::string pcd_filename; // filename of the point cloud to load from
  Eigen::Vector3d map_size; // (size_x, size_y, size_z)
};

static PlannerParameters TestParameters[] = {
  { 0.1, 
    0.2, 
    "forest_10x10_1.pcd",
    Eigen::Vector3d{20.0, 20.0, 20.0}},
  { 0.1, 
    0.2, 
    "tunnel_2x2x20.pcd",
    Eigen::Vector3d{20.0, 20.0, 20.0}},

};

//----------------------------------------

template <typename PointT = pcl::PointXYZ> inline
typename pcl::PointCloud<PointT>::Ptr
ReadCloud(const std::string& filename)
{
  auto path = std::filesystem::path(DATA_PATH);
  auto full_path = (path / filename).generic_string();

  if(!std::filesystem::exists(full_path))
  {
    std::cout << "File not found: " << full_path << std::endl;
    exit(-1);
  }
  typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
  if (pcl::io::loadPCDFile<PointT>(full_path, *cloud) == -1)
  {
    std::cout << "Problem loading file " << full_path << std::endl;
    exit(-1);
  }
  return cloud;
}

#endif // _BENCHMARK_UTILS_HPP