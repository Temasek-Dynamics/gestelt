#include <iostream>
#include <ros/ros.h>
#include <fake_map/fake_sensor.h>

int main (int argc, char** argv) 
{        
    ros::init (argc, argv, "fake_sensor");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    FakeSensor fake_sensor(nh, pnh);

    ros::AsyncSpinner async_spinner(2);
    async_spinner.start();
    ros::waitForShutdown();

    return 0;
}