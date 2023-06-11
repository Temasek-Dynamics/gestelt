#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <tf2_ros/transform_listener.h>

class EgoGZBridge {
public:

  void init() {
    ROS_INFO("Bridge initialized");

    // Subscribers
    // TODO Move into a separate callback queue
    pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, &EgoGZBridge::pose_cb, this);

    // Publishers
    camera_pos_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/iris_depth_camera/camera/pose", 10);

    // Timers
    pub_camera_pose_timer_ = nh.createTimer(ros::Duration(1/30), &EgoGZBridge::pubCameraPoseTimerCb, this);

    // Transformations
    tfListener_.reset(new tf2_ros::TransformListener(tfBuffer_));
  }

  // Subscribe to robot pose and publish transformations
  void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
  {
    // Publish TF from 'base_link' to 'map'
    static tf::TransformBroadcaster br;
    // tf::Transform transform;
    // tf::Transform transform = tf::Transform(tf::Quaternion( msg->pose.orientation.x,
    //                                                 msg->pose.orientation.y,
    //                                                 msg->pose.orientation.z,
    //                                                 msg->pose.orientation.w),
    //                                     tf::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z));
    tf::Stamped<tf::Pose> map_to_base_link_tf;
    tf::poseStampedMsgToTF(*msg, map_to_base_link_tf);

    br.sendTransform(tf::StampedTransform(map_to_base_link_tf, ros::Time::now(), "world", "base_link"));

  }

  // Helper method to get transformation between 2 frames
  bool getTransform(const std::string &refFrame, const std::string &childFrame, geometry_msgs::TransformStamped &transform)
  {
      try
      {
        transform = tfBuffer_.lookupTransform(refFrame, childFrame, ros::Time(0));
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

  // Timer callback to publish camera pose 
  void pubCameraPoseTimerCb(const ros::TimerEvent &e){

    geometry_msgs::TransformStamped tf_between_frames_msg;
    if (!getTransform("world", "camera_link", tf_between_frames_msg)){
      return;
    }
    camera_pos_msg_.header.frame_id = "world";
    camera_pos_msg_.header.stamp = ros::Time::now(); 

    camera_pos_msg_.pose.position.x = tf_between_frames_msg.transform.translation.x;
    camera_pos_msg_.pose.position.y = tf_between_frames_msg.transform.translation.y;
    camera_pos_msg_.pose.position.z = tf_between_frames_msg.transform.translation.z;

    camera_pos_msg_.pose.orientation = tf_between_frames_msg.transform.rotation;

    camera_pos_pub_.publish(camera_pos_msg_);

    // if (!base_link_to_cam_tf_init_){
    //   geometry_msgs::TransformStamped tf_between_frames_msg;
    //   if (!getTransform("world", "camera_link", tf_between_frames_msg)){
    //     return;
    //   }
    //   camera_pos_msg_.header.frame_id = "world";
    //   camera_pos_msg_.header.stamp = ros::Time::now(); 

    //   camera_pos_msg_.pose.position.x = tf_between_frames_msg.transform.translation.x;
    //   camera_pos_msg_.pose.position.y = tf_between_frames_msg.transform.translation.y;
    //   camera_pos_msg_.pose.position.z = tf_between_frames_msg.transform.translation.z;

    //   camera_pos_msg_.pose.orientation = tf_between_frames_msg.transform.rotation;

    //   base_link_to_cam_tf_init_ = true;
    // }
    // else {
    //   camera_pos_pub_.publish(camera_pos_msg_);
    // }

  }

  ros::NodeHandle nh;

  // TF transformation listeners
  tf2_ros::Buffer tfBuffer_;
  std::unique_ptr<tf2_ros::TransformListener> tfListener_;

  // Subscribers
  ros::Subscriber pose_sub;

  // Publishers
  ros::Publisher camera_pos_pub_;

  // Timers
  ros::Timer pub_camera_pose_timer_;

  // Flags

  // Indicates if there is already a transformation between the base link and camera obtained
  bool base_link_to_cam_tf_init_{false};

  // Stored data

  // Store the camera pose relative to base_link
  geometry_msgs::PoseStamped camera_pos_msg_;


private:

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gestalt_bringup_node");

  EgoGZBridge bridge;

  bridge.init();

  ros::spin();

  return 0;
}