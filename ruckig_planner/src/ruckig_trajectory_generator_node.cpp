#include <ruckig_trajectory_generator/ruckig_trajectory_generator.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "RuckigPlanner_node");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");
    RuckigPlanner ruckig_planner(nh, nh_private);
    ROS_INFO("Started up Ruckig Trajectory Generator NODE");
    ros::spin();

    return 0;
}
