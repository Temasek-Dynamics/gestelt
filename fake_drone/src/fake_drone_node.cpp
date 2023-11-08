#include <fake_drone/fake_drone.h>

int main (int argc, char** argv) 
{        
    ros::init (argc, argv, "fake_drone");
    ros::NodeHandle nh("~");
    FakeDrone fake_drone(nh);
    ros::spin();

    return 0;
}