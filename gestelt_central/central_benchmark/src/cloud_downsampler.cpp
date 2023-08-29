#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

class CloudDownsampler {
public:
  void init(ros::NodeHandle& nh) {
    int queue_size;
    nh.param("queue_size", queue_size, 0);
    nh.param("voxel_size", voxel_size_, 0.1);

    // Subscribers
    cloud_in_sub_ = nh.subscribe<sensor_msgs::PointCloud2>("cloud_in", queue_size, &CloudDownsampler::cloudInCB, this);

    // Publishers
    cloud_ds_pub_ = nh.advertise<sensor_msgs::PointCloud2>("cloud_out", queue_size);
  
    cloud_out_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    msg_out_.reset(new sensor_msgs::PointCloud2());
  }

  void cloudInCB(const sensor_msgs::PointCloud2ConstPtr &msg_in)
  {
    if (msg_in->data.empty()){
      return;
    }

    pcl::fromROSMsg(*msg_in, *cloud_out_);

    pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
    vox_grid.setInputCloud(cloud_out_);
    vox_grid.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
    vox_grid.filter(*cloud_out_);

    pcl::toROSMsg(*cloud_out_, *msg_out_);

    if (cloud_ds_pub_.getNumSubscribers() > 0) {
      cloud_ds_pub_.publish(*msg_out_);
    }
  }

private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out_;
  sensor_msgs::PointCloud2Ptr msg_out_;

  double voxel_size_;

  // Subscriber to input point cloud
  ros::Subscriber cloud_in_sub_;

  // Publisher to downsampled point cloud
  ros::Publisher cloud_ds_pub_; 
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cloud_downsampler_node");
  ros::NodeHandle nh("~");

  ROS_INFO("Starting up cloud_downsampler_node");

  CloudDownsampler cloud_downsampler;

  cloud_downsampler.init(nh);

  ros::spin();

  return 0;
}