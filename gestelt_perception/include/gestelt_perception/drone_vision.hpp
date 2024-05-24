#ifndef DRONE_VISION_H
#define DRONE_VISION_H

#include <iostream>
#include <ros/ros.h>
// #include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <gestelt_msgs/CommanderCommand.h>
#include <gestelt_msgs/CommanderState.h>


/* State machine events */
enum ServerEvent
{
  TAKEOFF,        // 0
  LAND,           // 1
  MISSION,        // 2
  HOVER,          // 3
  E_STOP,         // 4
  EMPTY,          // 5
};

class Vision{
    public:

        // Structure to hold 3D coordinates  
        struct Coordinates{
            double x;
            double y;
            double z;
        };

        Vision(ros::NodeHandle& nh);

        // void depthImageCallback(const sensor_msgs::Image::ConstPtr& msg);
        void imageCallback(const sensor_msgs::Image::ConstPtr& msg);

        void serverEventCallback(const gestelt_msgs::CommanderCommand::ConstPtr& msg);

        void startPerceptionImpl(const cv_bridge::CvImagePtr& cv_ptr);

        void tfCameraToWorld(const Vision::Coordinates& camera_coordinates, Vision::Coordinates& world_coordinates);

        void convertToPointStamp(const Coordinates& coordinates, geometry_msgs::PointStamped& point);

        void convertToCoordinates(const geometry_msgs::PointStamped& point, Coordinates& coordinates);
        
        void pixelToCameraCoords(const cv::Point2f& pixel, const double& depth, Coordinates& camera_coords);

        double estimateDepth(double realLength, const cv::Point2f& p1, const cv::Point2f& p2) {
            double focalLength = (cameraMatrix_.at<double>(0, 0) + cameraMatrix_.at<double>(1, 1) )/2;
            double pixelLength = cv::norm(p2 - p1);
            double depth = (focalLength * realLength) / pixelLength;    
            return depth;
        }


    private:
        // ROS Subscriber 
        ros::Subscriber drone_camera_image_sub__;
        ros::Subscriber server_event_sub__;
        // ros::Subscriber drone_camera_depth_image_sub_;
        
        //ROS Publisher
        
        //opencv images variables
        cv::Mat gray_image_;
        cv::Mat binary_image_;
        
        cv::Mat cameraMatrix_ = (cv::Mat_<double>(3, 3) << 
                                                        171.57953655346768, 0.0, 160.5, 
                                                        0.0, 171.57953655346768, 120.5, 
                                                        0.0, 0.0, 1.0);
        
        cv::Mat distCoeffs_ = (cv::Mat_<double>(1, 5) << 0.0, 0.0, 0.0, 0.0, 0.0);

        std::string gate_name_;
        double true_vertical_length_;
        double true_horizontal_length_;

        tf2_ros::Buffer tfBuffer_;
        tf2_ros::TransformListener tfListener_;

        bool start_image_pipeline__ = false;

};  //Vision 

#endif //DRONE_VISION_H