#include <fake_drone/grid_agent.h>

int main (int argc, char** argv) 
{        
    ros::init (argc, argv, "grid_agent");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    GridAgent grid_agent(nh, pnh);
    
    ros::spin();

    return 0;
}