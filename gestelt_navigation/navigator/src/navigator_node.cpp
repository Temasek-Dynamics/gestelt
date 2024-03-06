#include <csignal>

#include <navigator/navigator.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "navigator_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  Navigator navigator;

  navigator.init(nh, pnh);

  ros::MultiThreadedSpinner spinner(4);
  spinner.spin();

  return 0;
}
