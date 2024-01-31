#include <ros/ros.h>

#include <Eigen/Eigen>

// #include <traj_utils/PolyTraj.h>
// #include <traj_utils/MINCOTraj.h>
// #include <visualization_msgs/Marker.h>
// #include <cassert>

#include <optimizer/poly_traj_optimizer.h>
#include <traj_utils/planning_visualization.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "ego_planner_fsm_node");
  ros::NodeHandle nh;

  std::shared_ptr<ego_planner::PlanningVisualization> visualization_; 
  visualization_.reset(new ego_planner::PlanningVisualization(nh));

  ros::Duration(2.0).sleep();

  std::cout << "test_traj_opt" << std::endl;

  // Fixed parameters
  double max_vel = 1.5;
  double avg_vel = 1.0; 
  double seg_length = 1.0;
  size_t num_inner_wp = 2; // Number of inner waypoints 
  size_t num_segments = num_inner_wp + 1;

  // Set boundary conditions
  Eigen::Vector3d start_pos{0, 0, 0};
  Eigen::Vector3d goal_pos{0, 1, 0};
  // Set waypoints
  Eigen::MatrixXd inner_ctrl_pts(3, num_inner_wp); // matrix of inner waypoints
  inner_ctrl_pts << 1, 1,   
                    0, 1, 
                    0, 0;

  Eigen::Matrix3d startPVA; // Boundary start condition: Matrix consisting of 3d (position, velocity acceleration) 
  Eigen::Matrix3d endPVA;   // Boundary end condition: Matrix consisting of 3d (position, velocity acceleration) 
  startPVA << start_pos, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
  endPVA << goal_pos, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();

  // Calculate duration per segment
  double duration_per_seg =  seg_length / max_vel;
  Eigen::VectorXd seg_durations(num_segments);
  seg_durations = Eigen::VectorXd::Constant(num_segments, duration_per_seg);        

  std::cout << "Start: " << start_pos <<std::endl;
  std::cout << "Inner: " << inner_ctrl_pts << std::endl;
  std::cout << "Goal: " << goal_pos <<std::endl;

  poly_traj::MinJerkOpt mjo;
  mjo.reset(startPVA, endPVA, num_segments);
  mjo.generate(inner_ctrl_pts, seg_durations);

  std::cout << "Control effort (minimizing jerk): " << mjo.getTrajJerkCost()  << std::endl;


  Eigen::MatrixXd cstr_pts_mjo = mjo.getInitConstraintPoints(10);

  // // std::cout << mjo.constructQ(7, 4) << std::endl;
  // std::cout << mjo.constructQ(5, 3) << std::endl;

  visualization_->displayOptimalList(cstr_pts_mjo, 0);

  ros::MultiThreadedSpinner spinner(1);
  spinner.spin();

  return 0;
}