#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gate_display_node");
    ros::NodeHandle nh;
    ros::Rate rate(1); // Adjust the rate as needed

    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("gate_marker", 1);
    ros::Publisher room_area_marker_pub = nh.advertise<visualization_msgs::Marker>("area_marker", 1);

    while (ros::ok())
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";  // Adjust the frame_id according to your setup
        marker.header.stamp = ros::Time::now();
        marker.ns = "gate";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 1.2;  // Adjust position as needed
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 1.4;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.707;
        marker.pose.orientation.w = 0.707;
        marker.scale.x = 0.05;  // Adjust scale as needed
        marker.scale.y = 0.6;
        marker.scale.z = 0.8;
        marker.color.a = 1.0;
        marker.color.r = 0.0;  // Adjust color as needed
        marker.color.g = 1.0;
        marker.color.b = 0.0;


        visualization_msgs::Marker room_area_marker;
        room_area_marker.header.frame_id = "map";  // Adjust the frame_id according to your setup
        room_area_marker.header.stamp = ros::Time::now();
        room_area_marker.ns = "room_area";
        room_area_marker.id = 0;
        room_area_marker.type = visualization_msgs::Marker::CUBE;
        room_area_marker.action = visualization_msgs::Marker::ADD;
        room_area_marker.pose.position.x = 0.0;  // Adjust position as needed
        room_area_marker.pose.position.y = 0.0;
        room_area_marker.pose.position.z = 0.0;
        room_area_marker.pose.orientation.x = 0.0;
        room_area_marker.pose.orientation.y = 0.0;
        room_area_marker.pose.orientation.z = 0.0;
        room_area_marker.pose.orientation.w = 1.0;
        room_area_marker.scale.x = 4.0;  // Adjust scale as needed
        room_area_marker.scale.y = 4.0;
        room_area_marker.scale.z = 0.01;
        room_area_marker.color.a = 0.5;
        room_area_marker.color.r = 0.5;  // Adjust color as needed
        room_area_marker.color.g = 0.5;
        room_area_marker.color.b = 1.0;


        marker_pub.publish(marker);
        room_area_marker_pub.publish(room_area_marker); 

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
