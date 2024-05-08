#include  "ros/ros.h"
#include <gestelt_perception/drone_vision.hpp>
#include <iostream>

int main(int argc, char** argv) {

  ros::init(argc, argv, "vision_node");

  ros::NodeHandle n;
  Vision vision(n);

  ROS_INFO("Started up vision_node");

  ros::spin();  // process a few messages in the background - causes the uavPoseCallback to happen

  return 0;
}