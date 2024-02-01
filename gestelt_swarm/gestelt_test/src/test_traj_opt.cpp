#include <ros/ros.h>
#include <benchmark/benchmark.h>

#include <Eigen/Eigen>

#include <random>

// #include <visualization_msgs/Marker.h>
// #include <cassert>

#include <optimizer/poly_traj_optimizer.h>
#include <traj_utils/planning_visualization.h>

poly_traj::MinJerkOpt generateMJO(const Eigen::MatrixXd& waypoints){
  poly_traj::MinJerkOpt mjo;

  // Fixed parameters
  double max_vel = 1.5;
  double avg_vel = 1.0; 
  double seg_length = 1.0;
  size_t num_waypoints = waypoints.cols(); // Number of inner waypoints 
  if (num_waypoints < 2){
    throw std::runtime_error("size of waypoints is not at least 2");
  }
  size_t num_inner_wp = num_waypoints - 2; // Number of inner waypoints 
  size_t num_segments = num_waypoints -1; // Number of inner path segments 

  // Set boundary conditions
  Eigen::Matrix3d startPVA; // Boundary start condition: Matrix consisting of 3d (position, velocity acceleration) 
  Eigen::Matrix3d endPVA;   // Boundary end condition: Matrix consisting of 3d (position, velocity acceleration) 
  startPVA << waypoints.block<3,1>(0,0), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
  endPVA << waypoints.block<3,1>(0,num_waypoints-1), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();

  // Time allocation: Calculate duration per segment
  double duration_per_seg =  seg_length / max_vel;
  Eigen::VectorXd seg_durations(num_segments);
  seg_durations = Eigen::VectorXd::Constant(num_segments, duration_per_seg);        

  // std::cout << "waypoints: " << std::endl;
  // std::cout << waypoints << std::endl;

  mjo.reset(startPVA, endPVA, num_segments);

  mjo.generate( waypoints.block(0, 1, 3, num_inner_wp), seg_durations);

  return mjo;
}

Eigen::MatrixXd generateRandom3DWaypoints(  const int& n, const uint64_t& seed, 
                                            const Eigen::Vector3d& mean, 
                                            const Eigen::Vector3d& stddev)
{
  Eigen::MatrixXd waypoints(3, n);

  std::default_random_engine gen; 
  gen.seed(seed);

  std::normal_distribution<double> x_dis(mean(0), stddev(0));
  std::normal_distribution<double> y_dis(mean(1), stddev(1));
  std::normal_distribution<double> z_dis(mean(2), stddev(2));

  for (int i = 0; i < n; i++){
    waypoints.col(i) <<  x_dis(gen), y_dis(gen), z_dis(gen);
  }
  
  return waypoints;
}

static void getTrajJerkCost(benchmark::State& state)
{
  Eigen::MatrixXd waypoints = generateRandom3DWaypoints(100, 69, Eigen::Vector3d{0, 0, 0}, Eigen::Vector3d{2, 2, 2});
  poly_traj::MinJerkOpt mjo = generateMJO(waypoints);

  for (auto _ : state)
  {
    double new_jerk_cost = mjo.getTrajJerkCost();
  }
}

static void getTrajJerkCostOg(benchmark::State& state)
{
  Eigen::MatrixXd waypoints = generateRandom3DWaypoints(100, 69, Eigen::Vector3d{0, 0, 0}, Eigen::Vector3d{2, 2, 2});
  poly_traj::MinJerkOpt mjo = generateMJO(waypoints);

  for (auto _ : state)
  {
    double new_jerk_cost = mjo.getTrajJerkCostOg();
  }
}

// Register the function as a benchmark
BENCHMARK(getTrajJerkCost);
BENCHMARK(getTrajJerkCostOg);
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