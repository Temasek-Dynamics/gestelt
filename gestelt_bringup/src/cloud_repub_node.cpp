#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <sensor_msgs/PointCloud2.h>
// #include <sensor_msgs/point_cloud_conversion.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <tf2_ros/transform_listener.h>

class CloudRepub {
public:

  void init() {
    // Subscribers
    cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/iris_depth_camera/camera/depth/points", 1, &CloudRepub::cloud_cb, this);

    // Publishers
    cloud_transformed_pub = nh.advertise<sensor_msgs::PointCloud2>("/global_cloud", 1);
    
    tfListener.reset(new tf2_ros::TransformListener(tfBuffer));
  }

  bool getTransform(const std::string &refFrame, const std::string &childFrame, geometry_msgs::TransformStamped &transform)
  {
      try
      {
        transform = tfBuffer.lookupTransform(refFrame, childFrame, ros::Time(0));
      }
      catch (const tf2::TransformException &ex)
      {
        ROS_ERROR_STREAM(
            "Pointcloud transform | Error in lookupTransform of " << childFrame << " in " << refFrame);
        ROS_WARN("%s",ex.what());
        return false;
      }
    return true;
  }

  void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &msg)
  {
    if (msg->data.empty()){
      return;
    }

    geometry_msgs::TransformStamped tf_between_frames_msg;
    if (!getTransform("map", msg->header.frame_id, tf_between_frames_msg) ){
      return;
    }

    // tf::StampedTransform tf_between_frames;
    // tf::transformStampedTFToMsg(tf_between_frames, tf_between_frames_geo);

    sensor_msgs::PointCloud2 msg_transformed;
    tf2::doTransform(*msg, msg_transformed, tf_between_frames_msg); // do transformation

    if (cloud_transformed_pub.getNumSubscribers() > 0) {
      cloud_transformed_pub.publish(msg_transformed);
    }

    ROS_INFO("Published global cloud");
  }

private:

  ros::NodeHandle nh;

  tf2_ros::Buffer tfBuffer;
  std::unique_ptr<tf2_ros::TransformListener> tfListener;

  ros::Subscriber cloud_sub;

  ros::Publisher cloud_transformed_pub;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cloud_repub_node");

  ROS_INFO("Starting up cloud_repub_node");

  CloudRepub cloud_repub;

  cloud_repub.init();

  ros::spin();

  return 0;
}