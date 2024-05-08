#include <gestelt_perception/drone_vision.hpp>

Vision::Vision(ros::NodeHandle& nh){
    drone_camera_image_sub_ = nh.subscribe("/drone/camera/rgb/image_raw", 1, &Vision::imageCallback, this);
    ros::spin();
}

void Vision::imageCallback(const sensor_msgs::Image::ConstPtr& msg){
    try
    {
        // Convert the ROS image message to a CvImage suitable for OpenCV
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        convertToBinaryImage(cv_ptr);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}



void Vision::convertToBinaryImage(const cv_bridge::CvImagePtr& cv_ptr) {
    cv::cvtColor(cv_ptr->image, gray_image_, cv::COLOR_BGR2GRAY);  // Convert to grayscale

    // Apply a high threshold to convert the grayscale image to binary
    double threshold = 150; 
    cv::threshold(gray_image_, binary_image_, threshold, 255, cv::THRESH_BINARY);

    // Display the binary image using OpenCV
    cv::imshow("Drone Image - Binary", binary_image_);
    cv::waitKey(30);  // Wait for a key press for 30 milliseconds
}

