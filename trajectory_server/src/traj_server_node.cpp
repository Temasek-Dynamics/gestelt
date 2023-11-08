#include <ros/ros.h>
#include <traj_server/traj_server.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_server");
  ros::NodeHandle nh("~");

  TrajServer traj_server;

  traj_server.init(nh);

  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();

  return 0;
}