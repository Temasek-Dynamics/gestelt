#include <ros/ros.h>
#include <learning_agile/learning_agile.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "learning_agile_node");
  ros::NodeHandle nh("~");

  LearningAgile learning_agile;

  learning_agile.init(nh);

  ros::MultiThreadedSpinner spinner(4);
  spinner.spin();

  return 0;
}