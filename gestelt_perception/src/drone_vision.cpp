#include <gestelt_perception/drone_vision.hpp>

Vision::Vision(ros::NodeHandle& nh) : 
        tfListener_(tfBuffer_)
{
    nh.getParam("gates/name", gate_name_);
    nh.getParam("gates/length_horizontal", true_horizontal_length_);
    nh.getParam("gates/length_vertical", true_vertical_length_);

    server_event_sub__ = nh.subscribe("/traj_server/command", 1, &Vision::serverEventCallback, this);
    drone_camera_image_sub__ = nh.subscribe("/drone/camera/rgb/image_raw", 1, &Vision::imageCallback, this);
    // drone_camera_depth_image_sub_ = nh.subscribe("/drone/camera/depth/image_raw", 1, &Vision::depthImageCallback, this);
    
    waypoints_pub__ = nh.advertise<gestelt_msgs::Goals>("/planner/goals", 10);
    

    ros::spin();
}

void Vision::serverEventCallback(const gestelt_msgs::CommanderCommand::ConstPtr& msg){
    uint16_t command = msg->command;
    if (command == MISSION){
        start_image_pipeline__ = true;
    }
}

void Vision::imageCallback(const sensor_msgs::Image::ConstPtr& msg){
    try
    {
        // Convert the ROS image message to a CvImage suitable for OpenCV
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        // Checking for trigger
        if (start_image_pipeline__){
            startPerceptionImpl(cv_ptr);
        }
        
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void Vision::startPerceptionImpl(const cv_bridge::CvImagePtr& cv_ptr) {
    cv::cvtColor(cv_ptr->image, gray_image_, cv::COLOR_BGR2GRAY);  // Convert to grayscale

    // Apply a high threshold to convert the grayscale image to binary
    double threshold = 150; 
    cv::threshold(gray_image_, binary_image_, threshold, 255, cv::THRESH_BINARY);

    // Invert the binary image if necessary
    cv::bitwise_not(binary_image_, binary_image_);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    // Find all contours in the binary image
    cv::findContours(binary_image_, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // Loop through the contours to find the largest one, assumed to be the rectangle
    double maxArea = 0;
    std::vector<cv::Point> largestContour;
    cv::Rect boundingRect;
    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area > maxArea) {
            maxArea = area;
            largestContour = contour;
            boundingRect = cv::boundingRect(contour);  // Get bounding box for the largest contour
        }
    }

    // Draw a green rectangle around the largest contour
    cv::Scalar greenColor(0, 255, 0);  
    cv::rectangle(cv_ptr->image, boundingRect, greenColor, 2);  

    // Show Image
    cv::imshow("Image with Green Rectangle", cv_ptr->image);
    cv::waitKey(30);

    // Calculate the center of the bounding rectangle
    cv::Point2f center(boundingRect.x + boundingRect.width / 2, boundingRect.y + boundingRect.height / 2);

    // Calculate the corners of the bounding rectangle
    cv::Point2f topLeft(boundingRect.x, boundingRect.y);
    cv::Point2f topRight(boundingRect.x + boundingRect.width, boundingRect.y);
    cv::Point2f bottomLeft(boundingRect.x, boundingRect.y + boundingRect.height);
    cv::Point2f bottomRight(boundingRect.x + boundingRect.width, boundingRect.y + boundingRect.height);
    
    // Calculate depth based on true size and pixel coordinates
    double depth_1 = Vision::estimateDepth(true_horizontal_length_ , topLeft, topRight);
    double depth_2 = Vision::estimateDepth(true_vertical_length_ , topRight, bottomRight);
    double depth_3 = Vision::estimateDepth(true_horizontal_length_ , bottomRight, bottomLeft);
    double depth_4 = Vision::estimateDepth(true_vertical_length_ , bottomLeft, topLeft);

    // Taking mean of all 4 depths
    double depth_mean = (depth_1 + depth_2 + depth_3 + depth_4) / 4;

    // To convert pixels to camera coordinates   
    Vision::Coordinates camera_coords;
    Vision::pixelToCameraCoords(center, depth_mean, camera_coords);

    std::cout<<"Camera: "<<camera_coords.x <<", "<<camera_coords.y<<", "<<camera_coords.z<<std::endl;
    
    // Convert coordinates in camera frame to world
    Vision::Coordinates world_coords;
    Vision::tfCameraToWorld(camera_coords, world_coords);
    std::cout<<"world: "<<world_coords.x <<", "<<world_coords.y<<", "<<world_coords.z<<std::endl;

    /////////////////////////////////////////////////////////////////////////////////////
    // ----------------------------- Creating trajectory ----------------------------- //
    float TIME_FACTOR_TERMINAL = 1;
    float TIME_FACTOR = 1;
    float MAX_VEL = 3;
    float MAX_ACCEL = 8;

    position_waypoints.push_back(Vision::createPose(world_coords.x, world_coords.y, world_coords.z));
    position_waypoints.push_back(Vision::createPose(world_coords.x, world_coords.y+2, world_coords.z));
    
    acceleration_waypoints.push_back(Vision::createAcceleration(None(), 
                                                                None(),
                                                                None()));

    acceleration_waypoints.push_back(Vision::createAcceleration(None(), 
                                                                None(),
                                                                None()));

    velocity_waypoints.push_back(Vision::createVelocity(None(), 
                                                        None(),
                                                        None()));

    velocity_waypoints.push_back(Vision::createVelocity(None(), 
                                                        None(),
                                                        None()));

    
    // ------------------------------------------------------------------------------- //
    /////////////////////////////////////////////////////////////////////////////////////

    // Publish the transformed coordinates to trajectory planner
    Vision::waypointsPublisher(position_waypoints, acceleration_waypoints, velocity_waypoints,
                                TIME_FACTOR_TERMINAL, TIME_FACTOR, MAX_VEL, MAX_ACCEL);
    
}

void Vision::tfCameraToWorld(const Coordinates& camera_coordinates, Coordinates& world_coordinates){
    geometry_msgs::PointStamped cam_point;
    Vision::convertToPointStamp(camera_coordinates, cam_point);
    
    try
    {
        geometry_msgs::TransformStamped transformStamped;

        // Look up the transform from cam_link to world
        transformStamped = tfBuffer_.lookupTransform("map", "cam_link", ros::Time(0));

        geometry_msgs::PointStamped world_point;
        tf2::doTransform(cam_point, world_point, transformStamped);

        // ROS_INFO("cam_link coordinates: x = %f, y = %f, z = %f", cam_point.point.x, cam_point.point.y, cam_point.point.z);
        // ROS_INFO("world coordinates: x = %f, y = %f, z = %f", world_point.point.x, world_point.point.y, world_point.point.z);

        Vision::convertToCoordinates(world_point, world_coordinates);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
    } 
}

void Vision::pixelToCameraCoords(const cv::Point2f& pixel, const double& depth, Coordinates& camera_coords) {
            // Convert pixel to normalized image coordinates
            std::vector<cv::Point2f> distortedPoints(1, pixel);
            std::vector<cv::Point2f> undistortedPoints;
            cv::undistortPoints(distortedPoints, undistortedPoints, cameraMatrix_, distCoeffs_);

            // Convert to camera coordinates using the provided depth
            cv::Point2f normalized = undistortedPoints[0];
            camera_coords.x = static_cast<double>(normalized.x) * depth;
            camera_coords.y = static_cast<double>(normalized.y) * depth;
            camera_coords.z = depth;
        }

void Vision::convertToPointStamp(const Coordinates& coordinates, geometry_msgs::PointStamped& point){
    point.header.frame_id = "cam_link";
    point.header.stamp = ros::Time::now();
    point.point.x = coordinates.x;
    point.point.y = coordinates.y;
    point.point.z = coordinates.z;
}

void Vision::convertToCoordinates(const geometry_msgs::PointStamped& point, Coordinates& coordinates){
    coordinates.x = point.point.x;
    coordinates.y = point.point.y;
    coordinates.z = point.point.z;
}

geometry_msgs::Pose Vision::createPose(const double& x, const double& y, const double& z){
    geometry_msgs::Pose wp__;
    wp__.position.x = x;
    wp__.position.y = y;
    wp__.position.z = z;

    wp__.orientation.x = 0;
    wp__.orientation.y = 0;
    wp__.orientation.z = -0.707;
    wp__.orientation.w = 0.707;

    return wp__;
}

std::pair<geometry_msgs::Accel, std_msgs::Bool> Vision::createAcceleration(
        const double& acc_x, const double& acc_y, const double& acc_z){
    
    geometry_msgs::Accel acc;
    std_msgs::Bool acc_mask;
    if (acc_x != None()) {
        acc.linear.x = acc_x;
        acc.linear.y = acc_y;
        acc.linear.z = acc_z;
        acc_mask.data = false;
    } else {
        acc_mask.data = true;
    }
    return std::make_pair(acc, acc_mask);
}

std::pair<geometry_msgs::Twist, std_msgs::Bool> Vision::createVelocity(const double& vel_x, 
                                                                        const double& vel_y, 
                                                                        const double& vel_z) {
    geometry_msgs::Twist vel;
    std_msgs::Bool vel_mask;
    if (vel_x != None()) {
        vel.linear.x = vel_x;
        vel.linear.y = vel_y;
        vel.linear.z = vel_z;
        vel_mask.data = false;
    } else {
        vel_mask.data = true;
    }
    return std::make_pair(vel, vel_mask);
}


void Vision::waypointsPublisher(const std::vector<geometry_msgs::Pose> &waypoints, 
                                const std::vector<std::pair<geometry_msgs::Accel, std_msgs::Bool>> &accels, 
                                const std::vector<std::pair<geometry_msgs::Twist, std_msgs::Bool>> &vels,
                                const float& time_factor_terminal, const float& time_factor, 
                                const float& max_vel, const float& max_accel) {
    
    gestelt_msgs::Goals wp_msg;
    // geometry_msgs::PoseArray wp_pos_msg;
    // geometry_msgs::AccelStamped wp_acc_msg;

    wp_msg.header.frame_id = "world";
    // wp_pos_msg.header.frame_id = "world";
    // wp_acc_msg.header.frame_id = "world";

    wp_msg.waypoints = waypoints;
    for (const auto &accel : accels) {
        wp_msg.accelerations.push_back(accel.first);
        wp_msg.accelerations_mask.push_back(accel.second);
    }
    for (const auto &vel : vels) {
        wp_msg.velocities.push_back(vel.first);
        wp_msg.velocities_mask.push_back(vel.second);
    }

    // wp_pos_msg.poses = waypoints;
    // if (!accels.empty()) {
    //     wp_acc_msg.accel = accels[0].first;
    // }
    wp_msg.time_factor_terminal.data = time_factor_terminal;
    wp_msg.time_factor.data = time_factor;
    wp_msg.max_vel.data = max_vel;
    wp_msg.max_acc.data = max_accel;

    waypoints_pub__.publish(wp_msg);
    // waypoints_pos_pub.publish(wp_pos_msg);
    // waypoints_acc_pub.publish(wp_acc_msg);
    // waypoints_pub__.shutdown();
}