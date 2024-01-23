#include <csignal>

#include <back_end_planner/back_end_planner.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ego_planner_fsm_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  BackEndPlanner back_end_planner;

  back_end_planner.init(nh, pnh);

  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();

  return 0;
}
