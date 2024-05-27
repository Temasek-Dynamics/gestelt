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
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Accel.h>
#include <geometry_msgs/Twist.h>
#include <gestelt_msgs/CommanderCommand.h>
#include <gestelt_msgs/CommanderState.h>
#include <gestelt_msgs/Goals.h>
#include <std_msgs/Bool.h>

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

        geometry_msgs::Pose createPose(const double& x, const double& y, const double& z);

        std::pair<geometry_msgs::Accel, std_msgs::Bool> createAcceleration(
                                                            const double& acc_x, 
                                                            const double& acc_y, 
                                                            const double& acc_z
                                                            );

        std::pair<geometry_msgs::Twist, std_msgs::Bool> createVelocity(const double& vel_x, 
                                                                        const double& vel_y, 
                                                                        const double& vel_z);

        void waypointsPublisher(const std::vector<geometry_msgs::Pose> &waypoints, 
                                const std::vector<std::pair<geometry_msgs::Accel, std_msgs::Bool>> &accels, 
                                const std::vector<std::pair<geometry_msgs::Twist, std_msgs::Bool>> &vels, 
                                const float& time_factor_terminal, const float& time_factor, 
                                const float& max_vel, const float& max_accel);

        double estimateDepth(double realLength, const cv::Point2f& p1, const cv::Point2f& p2) {
            double focalLength = (cameraMatrix_.at<double>(0, 0) + cameraMatrix_.at<double>(1, 1) )/2;
            double pixelLength = cv::norm(p2 - p1);
            double depth = (focalLength * realLength) / pixelLength;    
            return depth;
        }

        inline double None(){
            return std::numeric_limits<double>::max();
        }


    private:
        // ROS Subscriber 
        ros::Subscriber drone_camera_image_sub__;
        ros::Subscriber server_event_sub__;
        // ros::Subscriber drone_camera_depth_image_sub_;
        
        //ROS Publisher
        ros::Publisher waypoints_pub__;

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

        std::vector<geometry_msgs::Pose> position_waypoints;
        std::vector<std::pair<geometry_msgs::Accel, std_msgs::Bool>> acceleration_waypoints;
        std::vector<std::pair<geometry_msgs::Twist, std_msgs::Bool>> velocity_waypoints;
};  //Vision 

#endif //DRONE_VISION_H