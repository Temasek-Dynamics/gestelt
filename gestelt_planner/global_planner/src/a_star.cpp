#include "global_planner/planner_base.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "global_planner");
  ros::NodeHandle nh("~");

  PlannerBase planner_base(nh);

  ros::spin();

  return 0;
}