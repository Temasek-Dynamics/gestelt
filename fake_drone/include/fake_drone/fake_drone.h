#ifndef FAKE_DRONE_H
#define FAKE_DRONE_H

#include <mutex>
// #include <random>
#include <Eigen/Dense>

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>

#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>

#include <tf/tf.h>
#include <tf2_ros/transform_broadcaster.h>

#define KNRM  "\033[0m"
#define KRED  "\033[31m"
#define KGRN  "\033[32m"
#define KYEL  "\033[33m"
#define KBLU  "\033[34m"
#define KMAG  "\033[35m"
#define KCYN  "\033[36m"
#define KWHT  "\033[37m"

struct MavrosState {
    bool armed;
    // For more custom modes, refer to http://wiki.ros.org/mavros/CustomModes
    std::string custom_mode;
};

class FakeDrone
{
    private:
        struct Command
        {
            mavros_msgs::PositionTarget pos_targ; // position, velocity, acceleration
            Eigen::Quaterniond q;
            double yaw; // in units of rad
        };

        struct State 
        {
           geometry_msgs::PoseStamped pose;
           nav_msgs::Odometry odom;
        };

        // Name given to node
        std::string node_name_; 

        /* Publishers, subscribers, timers and services */
        ros::Subscriber setpoint_raw_local_sub_; 

        ros::Publisher odom_pub_, pose_pub_;
        ros::Publisher mavros_state_pub_;

        ros::ServiceServer arming_srv_server_; // Arm uav
        ros::ServiceServer set_mode_srv_server_; // Set mavros mode

        ros::Timer sim_update_timer_; // TImer to update simulation
        
        /* Params */
        int uav_id_;

        std::string uav_origin_frame_, base_link_frame_;

        double tf_broadcast_freq_; // frequency that tf is broadcasted 
        double pose_pub_freq_; // frequency that pose was published
        double offboard_timeout_; // Timeout for PX4 offboard to continuously receive commands
        Eigen::Vector3d init_pos_;

        /* Data */

        FakeDrone::State state_cur_; // Current state (pose, odom)
        FakeDrone::Command cmd_des_; // Desired command

        std::mutex cmd_mutex_;
        std::mutex state_mutex_;

        ros::Time last_pose_pub_time_; // Last timestamp that pose was published
        ros::Time last_tf_broadcast_time_; // Last timestamp that tf was broadcasted
        ros::Time last_cmd_received_time_; // Last timestamp that command was received 
        ros::Time last_mavros_state_pub_time_; // Last timestamp that mavros state was published 

        // Current state of the UAV (Mavros)
        MavrosState mavros_state_{
            false, 
            "AUTO.LOITER"
        };

        // TF transformation 
        tf2_ros::TransformBroadcaster tf_broadcaster_;

        // Unused: color for the trajectory using random values
        // Eigen::Vector4d color_vect_;

    public:

        FakeDrone(ros::NodeHandle &nh, ros::NodeHandle &pnh);

        ~FakeDrone();

        /* Timer callbacks */

        // Main timer for updating UAV simulation 
        void simUpdateTimer(const ros::TimerEvent &);

        /* Subscription callbacks */

        // Callback for UAV commands
        void setpointRawCmdCb(
            const mavros_msgs::PositionTarget::ConstPtr &msg);

        /* Service callbacks */
        bool armSrvCb(mavros_msgs::CommandBool::Request &req,
                mavros_msgs::CommandBool::Response &res);

        bool setModeCb(mavros_msgs::SetMode::Request &req,
                mavros_msgs::SetMode::Response &res);

        /* Checks */

        // Checks if UAV is in OFFBOARD mode and armed
        bool isUAVReady();

        // Check if UAV command is sent within the given timeout
        bool isOffboardCmdTimeout(double timeout);

        /** Helper methods */

        // Calculate orientation of drone from acceleration and current yaw
        Eigen::Quaterniond calcUAVOrientation(
            const geometry_msgs::Vector3& acc, const double& yaw_rad);

        // Publish mavros state
        void pubMavrosState();

        // Stop and hover the drone in the last given position
        void stopAndHover(FakeDrone::Command& cmd);

        void setStateFromCmd(FakeDrone::State& state, const FakeDrone::Command& cmd);
};

#endif // FAKE_DRONE_H
