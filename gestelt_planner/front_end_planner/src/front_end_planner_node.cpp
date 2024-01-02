#include <csignal>

#include <front_end_planner/front_end_planner.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ego_planner_fsm_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  FrontEndPlanner front_end_planner;

  front_end_planner.init(nh, pnh);

  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();

  return 0;
}
