#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf2_ros/transform_broadcaster.h>

class UAVTFBroadcaster {
public:

  void init(ros::NodeHandle& nh) {
    nh.param("drone_id", drone_id_, 0);
    nh.param("origin_frame", origin_frame_, std::string("world"));
    nh.param("base_link_frame", base_link_frame_, std::string("base_link"));

    nh.param("tf_broadcast_freq", tf_broadcast_freq_, 50.0);

    // Subscribers
    // TODO Move into a separate callback queue
    pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, &UAVTFBroadcaster::pose_cb, this);

    // Publishers
    // camera_pos_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/iris_depth_camera/camera/pose", 10);

    // Timers
    // pub_camera_pose_timer_ = nh.createTimer(ros::Duration(1/30), &UAVTFBroadcaster::pubCameraPoseTimerCb, this);
  }

  // Subscribe to robot pose and publish transformations
  void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
  {
    origin_to_base_tf_.header.stamp = ros::Time::now();
    origin_to_base_tf_.header.frame_id = origin_frame_;
    origin_to_base_tf_.child_frame_id = base_link_frame_;
    origin_to_base_tf_.transform.translation.x = msg->pose.position.x;
    origin_to_base_tf_.transform.translation.y = msg->pose.position.y;
    origin_to_base_tf_.transform.translation.z = msg->pose.position.z;

    origin_to_base_tf_.transform.rotation.x = msg->pose.orientation.x;
    origin_to_base_tf_.transform.rotation.y = msg->pose.orientation.y;
    origin_to_base_tf_.transform.rotation.z = msg->pose.orientation.z;
    origin_to_base_tf_.transform.rotation.w = msg->pose.orientation.w;
    
    br.sendTransform(origin_to_base_tf_);

  }

  // void broadcastTFTimerCb(const ros::TimerEvent &e){
  // }

private: 

  int drone_id_; // ID of drone being commanded by trajectory server instance

  std::string origin_frame_; //Origin frame of UAV
  std::string base_link_frame_; //Base link frame of UAV
  std::string cam_link_frame_; //Base link frame of UAV

  // TF transformation 
  tf2_ros::TransformBroadcaster br;

  // Subscribers
  ros::Subscriber pose_sub;

  // Publishers

  // Timers

  // Params
  double tf_broadcast_freq_;

  // Flags

  // Stored data
  geometry_msgs::TransformStamped origin_to_base_tf_;

private:

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "uav_tf_broadcaster");
  ros::NodeHandle nh("~");

  UAVTFBroadcaster uav_tf_broadcaster;

  uav_tf_broadcaster.init(nh);

  ros::spin();

  return 0;
}