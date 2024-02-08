// #ifndef RUCKIG_TRAJECTORY_GENERATOR_H
// #define RUCKIG_TRAJECTORY_GENERATOR_H


#include <ros/ros.h>
#include <gestelt_msgs/Goals.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/PositionTarget.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
// #include <trajectory_msgs/MultiDOFJointTrajectoryPoint.msg> 
#include <std_srvs/Empty.h>
#include <std_msgs/Header.h>
#include <ruckig/ruckig.hpp>
#include <iostream>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <eigen_conversions/eigen_msg.h>

class RuckigPlanner {
public:
    RuckigPlanner(ros::NodeHandle& nh_,
                    ros::NodeHandle& nh_private_);
    ~RuckigPlanner();

    void waypointCB(const gestelt_msgs::Goals::ConstPtr& msg);
    
    void planner(const std::vector<Eigen::Vector3d>& wp_pos,
                            const std::vector<Eigen::Vector3d>& wp_vel,
                            const std::vector<Eigen::Vector3d>& wp_acc);
    
    void generate_trajectory_points(const ruckig::Trajectory<3>& trajectory);
    
    void pubTrajectory(const std::array<double, 3>& pos, 
                                const std::array<double, 3>& vel, 
                                const std::array<double, 3>& acc, 
                                double time_stamp);

    void processTrajectory();

    bool stopSamplingCallback(std_srvs::Empty::Request& request,
                            std_srvs::Empty::Response& response);

    void uavOdomCallback(const nav_msgs::Odometry::ConstPtr& pose);

    void commandTimerCallback(const ros::TimerEvent&);
private:
    //--------PUBLISHER-------//
    ros::Publisher referencePublisher_;
    ros::Publisher command_pub_;

    //--------SUBSCRIBER-------//
    ros::Subscriber goalsSubscriber_;
    ros::Timer publish_timer_;
    ros::Subscriber sub_odom_;

    ros::Time start_time_;  
    ros::ServiceServer stop_srv_;

    std::vector<Eigen::Vector3d> goal_waypoints_;
    std::vector<Eigen::Vector3d> goal_waypoints_vel_;
    std::vector<Eigen::Vector3d> goal_waypoints_acc_;
    
    //not in use//
    std::vector<std::array<double, 3>> setPosition;
    std::vector<std::array<double, 3>> setVelocity;
    std::vector<std::array<double, 3>> setAcceleration;
    
    
    // Service client for getting the MAV interface to listen to our sent
    // commands.
    ros::ServiceClient position_hold_client_;

    // Flag whether to publish entire trajectory at once or not.
    bool publish_whole_trajectory_;
    // Trajectory sampling interval.
    double dt_;
    // Time at currently published trajectory sample.
    double current_sample_time_;
    //trajectory
    ruckig::Trajectory<3> trajectory_;

    Eigen::Affine3d current_pose_;
    Eigen::Vector3d current_velocity_;
    Eigen::Vector3d current_angular_velocity_;

    std::array<double, 3> new_position, new_velocity, new_acceleration;


};
