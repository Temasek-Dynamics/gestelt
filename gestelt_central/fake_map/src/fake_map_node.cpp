#include <iostream>
#include <ros/ros.h>
#include <fake_map/fake_map.h>

int main (int argc, char** argv) 
{        
    ros::init (argc, argv, "fake_map");
    ros::NodeHandle nh("~");
    FakeMap fake_map(nh);
    ros::spin();
    return 0;
}