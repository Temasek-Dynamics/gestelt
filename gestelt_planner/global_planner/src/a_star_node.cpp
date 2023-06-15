#include <global_planner/a_star.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "a_star_planner");
  ros::NodeHandle nh("~");

  AStarPlanner a_star_planner(nh);

  ros::spin();

  return 0;
}