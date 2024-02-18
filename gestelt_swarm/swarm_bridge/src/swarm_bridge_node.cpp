#include <swarm_bridge/swarm_bridge.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "swarm_bridge_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  SwarmBridge swarm_bridge;

  swarm_bridge.init(nh, pnh);

  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();

  return 0;
}