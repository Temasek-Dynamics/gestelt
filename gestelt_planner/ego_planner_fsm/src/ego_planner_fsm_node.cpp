#include <csignal>

#include <ego_planner_fsm/ego_planner_fsm.h>

void SignalHandler(int signal) {
  if(ros::isInitialized() && ros::isStarted() && ros::ok() && !ros::isShuttingDown()){
    ros::shutdown();
  }
}

using namespace ego_planner;

int main(int argc, char **argv)
{
  signal(SIGINT, SignalHandler);
  signal(SIGTERM,SignalHandler);

  ros::init(argc, argv, "ego_planner_fsm_node");
  ros::NodeHandle nh("~");
  ros::NodeHandle pnh;

  EGOReplanFSM ego_replan_fsm;

  ego_replan_fsm.init(nh, pnh);

  // ros::AsyncSpinner async_spinner(4);
  // async_spinner.start();
  // ros::waitForShutdown();

  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();

  return 0;
}
