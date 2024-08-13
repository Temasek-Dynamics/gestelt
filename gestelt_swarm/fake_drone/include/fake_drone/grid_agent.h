#ifndef GRID_AGENT_H
#define GRID_AGENT_H

#include <mutex>
// #include <random>
#include <Eigen/Dense>

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>

#include <tf/tf.h>
#include <tf2_ros/transform_broadcaster.h>

#include <gestelt_msgs/FrontEndPlan.h>

#include "tinysplinecxx.h"  // For spline interpolation

#define KNRM  "\033[0m"
#define KRED  "\033[31m"
#define KGRN  "\033[32m"
#define KYEL  "\033[33m"
#define KBLU  "\033[34m"
#define KMAG  "\033[35m"
#define KCYN  "\033[36m"
#define KWHT  "\033[37m"

class GridAgent
{
    private:
        // Name given to node
        std::string node_name_; 

        /* Publishers, subscribers, timers and services */
        ros::Subscriber fe_plan_sub_;  // Front end plan subscription

        ros::Publisher odom_pub_, pose_pub_;

        ros::Timer sim_update_timer_; // TImer to update simulation
        
        /* Params */
        int drone_id_{-1};
        double t_unit_{0.1};     // [s] Time duration of each space-time A* unit

        std::string uav_origin_frame_, base_link_frame_;

        double tf_broadcast_freq_; // frequency that tf is broadcasted 
        double pose_pub_freq_; // frequency that pose was published
        Eigen::Vector3d init_pos_;

        /* Data */

        std::mutex state_mutex_;

        ros::Time last_pose_pub_time_; // Last timestamp that pose was published
        ros::Time last_tf_broadcast_time_; // Last timestamp that tf was broadcasted

        // TF transformation 
        tf2_ros::TransformBroadcaster tf_broadcaster_;
        
        gestelt_msgs::FrontEndPlan fe_plan_msg_; // Front end plan 
        bool plan_received_{false}; // indicates that plan is received

        nav_msgs::Odometry odom_msg_;
        geometry_msgs::PoseStamped pose_msg_;

        double plan_start_exec_t_; // Time that plan started execution

        std::shared_ptr<tinyspline::BSpline> spline_; // Spline formed from interpolating control points of front end path

    public:

        GridAgent(ros::NodeHandle &nh, ros::NodeHandle &pnh);
        ~GridAgent();

        /* Timer callbacks */

        // Main timer for updating UAV simulation 
        void simUpdateTimer(const ros::TimerEvent &);

        /* Subscription callbacks */

        void frontEndPlanCB(const gestelt_msgs::FrontEndPlan::ConstPtr &msg);

        /* Checks */

        /** Helper methods */
        void setStateFromPlan(   const gestelt_msgs::FrontEndPlan &msg, 
                                            const double& exec_start_t);

};

#endif // GRID_AGENT_H
