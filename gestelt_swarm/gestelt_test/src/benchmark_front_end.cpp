// #include <ros/ros.h>
#include <Eigen/Eigen>

#include <benchmark/benchmark.h>
#include "benchmark_utils.hpp"

#include <sfc_generation/spherical_sfc.h>

#include <grid_map/grid_map.h>

#include <global_planner/a_star.h>
#include <sfc_generation/spherical_sfc.h>

// Read map from point cloud

// Benchmark the front end planner 
// static void frontEndPlan(benchmark::State& state)
// {
//   const auto& params = TestParameters[state.range(0)];
//   auto pcd = ReadCloud(params.pcd_filename);

//   // Initialize map
//   std::shared_ptr<GridMap> map_ = std::make_shared<GridMap>();
//   map_->initMap(pcd, params.map_size, params.inflation, params.voxel_resolution);

//   // Initialize front end planner
//   AStarPlanner::AStarParams astar_params;
//   astar_params.max_iterations = 9999;
//   astar_params.tie_breaker = 1.0001;
//   astar_params.debug_viz = false;

//   std::unique_ptr<AStarPlanner> front_end_planner_ = std::make_unique<AStarPlanner>(map_, astar_params);

//   std::cout << params.start_pos << " to " << params.goal_pos << std::endl;
//   for (auto _ : state)
//   {
//     if (!front_end_planner_->generatePlan(params.start_pos, params.goal_pos)){
//       state.SkipWithError("Failed to find front end Path");
//       return; // Early return is allowed when SkipWithError() has been used.
//     }
//   }
// }


// Benchmark the SFC Planner
static void SFCPlan(benchmark::State& state)
{
  const auto& params = TestParameters[state.range(0)];
  auto pcd = ReadCloud(params.pcd_filename);

  // Initialize map
  std::shared_ptr<GridMap> map_ = std::make_shared<GridMap>();
  map_->initMap(pcd, params.map_size, params.inflation, params.voxel_resolution);

  // Initialize front end planner
  AStarPlanner::AStarParams astar_params;
  astar_params.max_iterations = 999999;
  astar_params.tie_breaker = 1.0001;
  astar_params.debug_viz = false;

  std::unique_ptr<AStarPlanner> front_end_planner_ = std::make_unique<AStarPlanner>(map_, astar_params);

  // Initialize safe flight corridor
  SphericalSFC::SphericalSFCParams sfc_params; 
  sfc_params.max_itr = 100;
  sfc_params.debug_viz = false;

  sfc_params.max_sample_points = 2000;
  sfc_params.mult_stddev_x = 0.2;
  sfc_params.W_cand_vol = 1;
  sfc_params.W_intersect_vol = 5;

  sfc_params.min_sphere_vol = 0.1;
  sfc_params.max_sphere_vol = 1000.0;
  sfc_params.min_sphere_intersection_vol = 0.05;

  sfc_params.avg_vel = 1,5;
  sfc_params.max_vel = 3.0;

  std::unique_ptr<SphericalSFC> sfc_generation_ = std::make_unique<SphericalSFC>(map_, sfc_params);

  front_end_planner_->generatePlan(params.start_pos, params.goal_pos);

  std::vector<Eigen::Vector3d> front_end_path = front_end_planner_->getPathPos();

  for (auto _ : state)
  { 
    if (!sfc_generation_->generateSFC(front_end_path)){
      state.SkipWithError("Failed to find SFC Path");
      return; // Early return is allowed when SkipWithError() has been used.
    }
  }
}

// Register the function as a benchmark
// BENCHMARK(frontEndPlan)->Arg(0)->Arg(1)->Unit(benchmark::kMillisecond);
BENCHMARK(SFCPlan)->Arg(0)->Arg(1)->Unit(benchmark::kMillisecond);
// Run the benchmark
BENCHMARK_MAIN();