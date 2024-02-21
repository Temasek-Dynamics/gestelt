#include <fake_drone/fake_drone.h>

int main (int argc, char** argv) 
{        
    ros::init (argc, argv, "fake_drone");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    FakeDrone fake_drone(nh, pnh);
    
    ros::spin();

    return 0;
}