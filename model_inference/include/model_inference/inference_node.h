#pragma once
#include "model_inference/inference_engine.h"
#include <ros/ros.h>
// #include <Eigen/Dense>
#include <memory>
#include <Eigen/Geometry> 

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>


#include <gestelt_msgs/close_loop_NN_output.h>
#include <gestelt_msgs/Goals.h>

class InferenceEngine;  // 前向声明

class InferenceNode
{
public:
    void init(ros::NodeHandle& nh);

private:
    void inference_cb(const ros::TimerEvent& e);
    void drone_state_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void drone_state_twist_cb(const geometry_msgs::TwistStamped::ConstPtr& msg);
    void gate_points_cb(const geometry_msgs::PoseArray::ConstPtr& msg);
    void mission_start_cb(const gestelt_msgs::GoalsPtr &msg);
    void waypoint_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void get_obs();
   
    ros::Subscriber drone_pose_sub_;
    ros::Subscriber drone_twist_sub_;
    ros::Subscriber gate_points_sub_;
    ros::Subscriber waypoint_sub_;

    ros::Publisher  NN_output_pub_;
    ros::Timer inference_timer_;
    std::unique_ptr<InferenceEngine> engine_;

    double NN_freq_,vel_norm_factor_, pos_norm_factor_;
    int ctl_mode_=0;
    double drone_mass_=0.248;
    double t_tra_abs_=1.0;
    double mission_period_=5.0;
    int i_=0;
    bool MISSION_START_=false,RECEIVED_DRONE_TWIST_=false, RECEIVED_DRONE_POSE_=false;
    std::vector<float> std_model_input_;
    Eigen::Vector3d des_goal_point_={0,0,0},drone_pos_= {0,0,0};
    Eigen::Vector4d des_goal_quat_={1,0,0,0},drone_quat_= {1,0,0,0};
    Eigen::Vector3d des_goal_vel_={0,0,0},drone_vel_= {0,0,0};
    Eigen::Vector3d des_goal_ang_vel_={0,0,0},drone_ang_vel_= {0,0,0};
    Eigen::VectorXd des_goal_state_,  drone_state_, model_input_, gate_points_, last_gate_points_;
    geometry_msgs::TransformStamped transformStamped_;
};

