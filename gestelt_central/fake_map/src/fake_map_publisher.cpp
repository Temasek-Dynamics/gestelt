#include "ros/ros.h"

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>

#include <pcl_ros/point_cloud.h>

#include <sensor_msgs/PointCloud2.h>

#define KNRM  "\033[0m"
#define KRED  "\033[31m"
#define KGRN  "\033[32m"
#define KYEL  "\033[33m"
#define KBLU  "\033[34m"
#define KMAG  "\033[35m"
#define KCYN  "\033[36m"
#define KWHT  "\033[37m"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_publisher");
    ros::NodeHandle nh("~");

    double map_pub_freq;
    std::string _map_path; // Path to map
    pcl::PointCloud<pcl::PointXYZ> cloud_map; // map in point cloud form

    nh.param<std::string>("map/path", _map_path, "");
    nh.param<double>("map_publish_freq", map_pub_freq, 0.1);

    ros::Publisher map_pub = 
        nh.advertise<sensor_msgs::PointCloud2>("/global_map", 3);

    printf("[map_publisher] pcd path: %s\n", _map_path.c_str());

    // try to load the file

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(
        _map_path, cloud_map) == -1) 
    {
        printf("[map_publisher] %sno valid pcd used%s!\n", KRED, KNRM);
    }

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud_map, cloud_msg);
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = "world";

    ros::Rate loop_rate(map_pub_freq);

    while (ros::ok())
    {
        map_pub.publish(cloud_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::shutdown();
    return 0;
}