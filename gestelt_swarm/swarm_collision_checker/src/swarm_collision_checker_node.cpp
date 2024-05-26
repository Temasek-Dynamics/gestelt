#include <swarm_collision_checker/swarm_collision_checker.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "swarm_collision_checker");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  SwarmCollisionChecker swarm_collision_checker;

  swarm_collision_checker.init(nh, pnh);

  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();

  return 0;
}
