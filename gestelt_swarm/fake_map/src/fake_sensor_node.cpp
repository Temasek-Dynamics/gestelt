#include <iostream>
#include <ros/ros.h>
#include <fake_map/fake_sensor.h>

int main (int argc, char** argv) 
{        
    ros::init (argc, argv, "fake_sensor");
    ros::NodeHandle nh("~");
    FakeSensor fake_sensor(nh);
    ros::spin();
    return 0;
}