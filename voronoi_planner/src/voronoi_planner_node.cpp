#include <voronoi_planner/voronoi_planner.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "voronoi_planner");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  VoronoiPlanner voronoi_planner;

  voronoi_planner.init(nh, pnh);

  ros::MultiThreadedSpinner spinner(4);
  spinner.spin();

  return 0;
}
