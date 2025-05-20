#include "model_inference/inference_node.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "model_inference");
  ros::NodeHandle nh("~");

  InferenceNode inference_node;
  inference_node.init(nh);
  ros::spin();
  return 0;
}
