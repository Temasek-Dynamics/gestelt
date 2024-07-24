#include <ros/ros.h>
#include <mpc_ros_wrapper/mpc_ros_wrapper.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mpc_ros_wrapper_node");
  ros::NodeHandle nh("~");

  mpcRosWrapper mpc_ros_wrapper;

  mpc_ros_wrapper.init(nh);

  ros::MultiThreadedSpinner spinner(4);
  spinner.spin();

  return 0;
}