/*
 * Simple example that shows a trajectory planner using
 *  mav_trajectory_generation.
 *
 *
 * Launch via
 *   roslaunch trajectory_planner example.launch
 *
 * Wait for console to run through all gazebo/rviz messages and then
 * you should see the example below
 *  - After Enter, it receives the current uav position
 *  - After second enter, publishes trajectory information
 *  - After third enter, executes trajectory (sends it to the sampler)
 */
#include  "ros/ros.h"
#include <trajectory_planner/example_planner.h>

#include <iostream>

// int main(int argc, char** argv) {

//   ros::init(argc, argv, "simple_planner");

//   ros::NodeHandle n;
//   ExamplePlanner planner(n);
//   ROS_WARN_STREAM("SLEEPING FOR 1s TO WAIT FOR CLEAR CONSOLE");
//   ros::Duration(1.0).sleep();
//   ROS_WARN_STREAM("WARNING: CONSOLE INPUT/OUTPUT ONLY FOR DEMONSTRATION!");

//   // define set point
//   Eigen::Vector3d position, velocity;
  
//   Eigen::Vector3d world_position;
//   world_position << 2.0, 2.0, 2.0;

//   // Goal is given in drone origin frame
//   position = world_position;
//   velocity << 0.0, 0.0, 0.0;

//   // THIS SHOULD NORMALLY RUN INSIDE ROS::SPIN!!! JUST FOR DEMO PURPOSES LIKE THIS.
//   ROS_WARN_STREAM("PRESS ENTER TO UPDATE CURRENT POSITION AND SEND TRAJECTORY");
//   std::cin.get();
//   for (int i = 0; i < 10; i++) {
//     ros::spinOnce();  // process a few messages in the background - causes the uavPoseCallback to happen
//   }

//   mav_trajectory_generation::Trajectory trajectory;
//   planner.planTrajectory(position, velocity, &trajectory);
//   planner.publishTrajectory(trajectory);
//   ROS_WARN_STREAM("DONE. GOODBYE.");

//   return 0;
// }


int main(int argc, char** argv) {

  ros::init(argc, argv, "simple_planner");

  ros::NodeHandle n;
  ExamplePlanner planner(n);

  ROS_INFO("Started up trajectory_planner_node");

  ros::spin();  // process a few messages in the background - causes the uavPoseCallback to happen

  return 0;
}