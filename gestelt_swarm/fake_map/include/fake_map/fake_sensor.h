#ifndef FAKE_MAP_H
#define FAKE_MAP_H

#include <mutex>

#include <Eigen/Dense>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
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

class FakeSensor
{
    public:
        FakeSensor(ros::NodeHandle &nh, ros::NodeHandle &pnh);
        ~FakeSensor();

        // reset all data structures
        void reset(); 

        // Main timer for updating UAV state 
        void TFListenCB(const ros::TimerEvent &);

        // Main timer for refreshing sensor for rendering point clouds 
        void sensorUpdateTimerCB(const ros::TimerEvent &);

        /* Subscription callbacks */
        // void poseCB(const geometry_msgs::PoseStamped::ConstPtr &msg);

    private:
        FakeLaser fake_laser_; // Laser object for rendering fake point clouds

        /* Publishers, subscribers, timers and services */
        // ros::Subscriber pose_sub_;
        ros::Publisher fake_sensor_cloud_pub_; // publisher for fake sensor point cloud

        ros::Timer tf_listen_timer_; // Timer for listening to tf transformations and publishing them
        ros::Timer sensor_update_timer_; // Timer for updating sensor output

        // TF transformation 
        tf2_ros::Buffer tfBuffer_;
        std::unique_ptr<tf2_ros::TransformListener> tfListener_;

        geometry_msgs::TransformStamped sens_to_gbl_tf_; // sensor to global frame transform

        /* Params */
        int uav_id_;    // ID of drone
        std::string global_frame_; // Global inertial reference frame
        std::string uav_origin_frame_; // Reference frame of UAV
        std::string sensor_frame_;  // Frame of camera sensor
        
        std::string map_filepath_;

        bool downsampler_active_; // True if downsampler is active
        double ds_voxel_size_; // downsampling voxel size

        /* Data */
        pcl::PointCloud<pcl::PointXYZ> global_map_; // Global point cloud map
        pcl::PointCloud<pcl::PointXYZ>::Ptr sensor_cloud_; // Point cloud from fake laser

        // Mutex to ensure that pose data is not written to while being read
        // std::mutex pose_sensor_mutex_;
        std::mutex sensor_tf_mutex_;
        
        // Current odom of UAV
        // geometry_msgs::PoseStamped pose_;
        
        /* Flags */
        bool cam_tf_valid_{false}; // True if camera transform is valid

        /* Voxel filter for downsampling*/
        std::shared_ptr<pcl::VoxelGrid<pcl::PointXYZ>> vox_grid_{nullptr};

};

#endif // FAKE_MAP_H