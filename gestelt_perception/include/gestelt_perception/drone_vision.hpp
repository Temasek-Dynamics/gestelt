#ifndef DRONE_VISION_H
#define DRONE_VISION_H

#include <iostream>
#include <ros/ros.h>
// #include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>


class Vision{
    public:
        Vision(ros::NodeHandle& nh);

        void imageCallback(const sensor_msgs::Image::ConstPtr& msg);
        void convertToBinaryImage(const cv_bridge::CvImagePtr& cv_ptr);

    private:
        // ROS Subscriber 
        ros::Subscriber drone_camera_image_sub_;
        
        //ROS Publisher
        
        //opencv images variables
        cv::Mat gray_image_;
        cv::Mat binary_image_;

};  //Vision 

#endif //DRONE_VISION_H