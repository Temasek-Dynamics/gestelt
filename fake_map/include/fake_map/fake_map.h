#ifndef FAKE_MAP_H
#define FAKE_MAP_H

#include <mutex>

#include <Eigen/Dense>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/point_cloud.h>

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <fake_map/fake_laser.h>

#define KNRM  "\033[0m"
#define KRED  "\033[31m"
#define KGRN  "\033[32m"
#define KYEL  "\033[33m"
#define KBLU  "\033[34m"
#define KMAG  "\033[35m"
#define KCYN  "\033[36m"
#define KWHT  "\033[37m"

class FakeMap
{
    public:
        FakeMap(ros::NodeHandle &nodeHandle);
        ~FakeMap();

        // Main timer for updating UAV state 
        void TFListenCB(const ros::TimerEvent &);

        // Main timer for refreshing sensor for rendering point clouds 
        void sensorRefreshTimerCB(const ros::TimerEvent &);

        /* Subscription callbacks */
        // void poseCB(const geometry_msgs::PoseStamped::ConstPtr &msg);

        /* TF methods */
        void getCamLinkTF();

    private:
        FakeLaser fake_laser_; // Laser object for rendering fake point clouds

        ros::NodeHandle _nh;

        /* Publishers, subscribers, timers and services */
        ros::Subscriber pose_sub_;

        ros::Publisher fake_sensor_cloud_pub_;

        ros::Timer tf_listen_timer_;
        ros::Timer sensor_refresh_timer_;

        // TF transformation 
        tf2_ros::Buffer tfBuffer_;
        std::unique_ptr<tf2_ros::TransformListener> tfListener_;

        geometry_msgs::TransformStamped sens_to_gbl_tf_;

        /* Params */
        int uav_id;
        std::string global_frame_;
        std::string uav_origin_frame_;
        std::string sensor_frame_;
        
        std::string _id; 
        std::string map_filepath_;

        double _map_interval;
        double _map_pub_rate;

        /* Data */
        pcl::PointCloud<pcl::PointXYZ> _global_map; // Global map point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr sensor_cloud_; // Point cloud from fake laser

        // Mutex to ensure that pose data is not written to while being read
        std::mutex pose_sensor_mutex;
        std::mutex sensor_tf_mutex_;
        
        // Current odom of UAV
        geometry_msgs::PoseStamped pose_;

        /* Flags */
        bool got_tf_{false};
};

#endif // FAKE_MAP_H