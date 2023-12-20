#include <planner_adaptor/ego_planner_adaptor.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ego_planner_adaptor");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  EgoPlannerAdaptor ego_planner_adaptor;

  ego_planner_adaptor.init(nh, pnh);

  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();

  return 0;
}