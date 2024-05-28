#include <static_brushfire/static_brushfire.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "static_brushfire");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  StaticBrushfire static_brushfire;

  static_brushfire.init(nh, pnh);

  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();

  return 0;
}
