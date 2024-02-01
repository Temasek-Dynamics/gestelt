#include <ros/ros.h>
#include <benchmark/benchmark.h>

#include <Eigen/Eigen>

#include <random>

#include <optimizer/poly_traj_optimizer.h>
#include <traj_utils/planning_visualization.h>

#include <front_end_planner/front_end_planner.h>



static void getTrajJerkCost(benchmark::State& state)
{
  Eigen::MatrixXd waypoints = generateRandom3DWaypoints(100, 69, Eigen::Vector3d{0, 0, 0}, Eigen::Vector3d{2, 2, 2});
  poly_traj::MinJerkOpt mjo = generateMJO(waypoints);

  for (auto _ : state)
  {
    double new_jerk_cost = mjo.getTrajJerkCost();
  }
}

// Register the function as a benchmark
BENCHMARK(getTrajJerkCost);
// Run the benchmark
BENCHMARK_MAIN();


// int main(int argc, char **argv)
// {
//   ros::init(argc, argv, "ego_planner_fsm_node");
//   ros::NodeHandle nh;

//   std::shared_ptr<ego_planner::PlanningVisualization> visualization_; 
//   visualization_.reset(new ego_planner::PlanningVisualization(nh));
//   ros::Duration(2.0).sleep();
//   std::cout << "test_traj_opt" << std::endl;

//   // Eigen::MatrixXd waypoints(5, 3);

//   // waypoints <<  0, 0, 0,
//   //               1, 0, 0,
//   //               1, 1, 0,
//   //               0, 1, 0,
//   //               0, 0, 0;

//   Eigen::MatrixXd waypoints = generateRandom3DWaypoints(100, 69, Eigen::Vector3d{0, 0, 0}, Eigen::Vector3d{2, 2, 2});

//   poly_traj::MinJerkOpt mjo = generateMJO(waypoints);

//   // // std::cout << mjo.constructQ(7, 4) << std::endl;
//   // std::cout << mjo.constructQ(5, 3) << std::endl;

//   double new_jerk_cost = mjo.getTrajJerkCost();
//   double og_jerk_cost = mjo.getTrajJerkCostOg();

//   std::cout << "New Control effort (minimizing jerk): " << new_jerk_cost  << std::endl;
//   std::cout << "Og Control effort (minimizing jerk): " << og_jerk_cost  << std::endl;

//   // Register the function as a benchmark
//   BENCHMARK(mjo.getTrajJerkCost());
//   BENCHMARK(mjo.getTrajJerkCostOg());
//   // Run the benchmark
//   BENCHMARK_MAIN();

//   Eigen::MatrixXd cstr_pts_mjo = mjo.getInitConstraintPoints(10);

//   visualization_->displayOptimalList(cstr_pts_mjo, 0);

//   ros::MultiThreadedSpinner spinner(1);
//   spinner.spin();

//   return 0;
// }