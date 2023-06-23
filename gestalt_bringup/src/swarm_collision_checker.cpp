#include <ros/ros.h>
#include <deque>
#include <eigen3/Eigen/Eigen>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

#include <gazebo_msgs/ContactsState.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

class SwarmCollisionChecker {
public:

  void init(ros::NodeHandle& nh) {

    nh.param("num_drones", num_drones_, 0);
    num_drones_--; // TODO: remove this after solving the num_drones issue

    nh.param("check_collision_freq", check_collision_freq_, 10.0);
    nh.param("collision_tolerance", col_tol_, 0.2);

    // Create an pre-allocated world_to_drone_base_ vector of size num_drones
    for (int i = 0; i < num_drones_; i++){
      geometry_msgs::PoseStamped pose;
      world_to_drone_base_.push_back(pose);
    }

    // Subscribers
    for (int i = 0; i < num_drones_; i++) {
      // Add pose topic subscription
      std::string pose_topic = std::string("/drone" + std::to_string(i) + "/mavros/local_position/pose");

      ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(
        pose_topic, 5, boost::bind(&SwarmCollisionChecker::poseCB, this, _1, i));
      drones_pose_sub_.push_back(pose_sub);

      // Add collision sensor subscription
      std::string collision_sensor_topic = std::string("/drone" + std::to_string(i) + "/collision_sensor");

      ros::Subscriber collision_sensor_sub = nh.subscribe<gazebo_msgs::ContactsState>(
        collision_sensor_topic, 5, boost::bind(&SwarmCollisionChecker::collisionCB, this, _1, i));
      collision_sensor_sub_.push_back(collision_sensor_sub);
    }

    // Publishers
    swarm_collision_pub_ = nh.advertise<visualization_msgs::Marker>("/swarm_collision_points", 10);
    obs_collision_pub_ = nh.advertise<visualization_msgs::Marker>("/obstacle_collision_points", 10);

    // Timers
    // pub_camera_pose_timer_ = nh.createTimer(ros::Duration(1/30), &SwarmCollisionChecker::pubCameraPoseTimerCb, this);
    broadcast_tf_timer_ = nh.createTimer(ros::Duration(1/check_collision_freq_), &SwarmCollisionChecker::checkCollisionTimerCb, this);

    // Transformations
    tfListener_.reset(new tf2_ros::TransformListener(tfBuffer_));

    // Sleep so that TF tree is set up during system startup
    ros::Duration(5.0).sleep();

    // Get transforms from world to drone origin frames
    for (int i = 0; i < num_drones_; i++) {
      std::string drone_origin_frame = "drone" + std::to_string(i) + "_origin";
    
      geometry_msgs::PoseStamped world_to_origin_tf;

      if (!getTransform("world", drone_origin_frame, world_to_origin_tf)){
        ROS_ERROR("[Collision Checker] Failed to get transform from world to drone origin frame, shutting down.");
        ros::shutdown();
      }
      world_to_drone_origin_tfs_.push_back(world_to_origin_tf);
    }

  }

  // Subscribe to robot pose
  void poseCB(const geometry_msgs::PoseStamped::ConstPtr &msg, int drone_id)
  {
    world_to_drone_base_[drone_id].header.stamp = msg->header.stamp;
    world_to_drone_base_[drone_id].header.frame_id = msg->header.frame_id;

    world_to_drone_base_[drone_id].pose.position.x = msg->pose.position.x + world_to_drone_origin_tfs_[drone_id].pose.position.x;
    world_to_drone_base_[drone_id].pose.position.y = msg->pose.position.y + world_to_drone_origin_tfs_[drone_id].pose.position.y;
    world_to_drone_base_[drone_id].pose.position.z = msg->pose.position.z + world_to_drone_origin_tfs_[drone_id].pose.position.z;
  }

  // Subscribe to collision sensor topic
  void collisionCB(const gazebo_msgs::ContactsState::ConstPtr &msg, int drone_id)
  {
    if (!msg->states.empty()){
      for (auto collision : msg->states){
        ROS_INFO("[Collision Checker] Collision detected between %s and %s at position (%f, %f, %f)!", 
          collision.collision1_name.c_str(), collision.collision2_name.c_str(),
          collision.contact_positions[0].x, collision.contact_positions[0].y, collision.contact_positions[0].z);

        geometry_msgs::Point collision_point;
        collision_point.x = collision.contact_positions[0].x;
        collision_point.y = collision.contact_positions[0].y;
        collision_point.z = collision.contact_positions[0].z;

        obs_collision_points_.push_back(collision_point);

        if (obs_collision_points_.size() > 500){
          // Prevent swarm_collision_points_ from getting too large
          obs_collision_points_.pop_front();
        }
      }
    }

    std_msgs::ColorRGBA sphere_color;
    sphere_color.r = 1.0;
    sphere_color.g = 1.0;
    sphere_color.b = 0.0;
    sphere_color.a = 0.8;
    
    // Publish point visualization at which obstacle collision occured
    obs_collision_pub_.publish(createSphereList(0, obs_collision_points_, sphere_color));
  }

  void checkCollisionTimerCb(const ros::TimerEvent &e){
    // Checks for collision between every pair of drone. Iterates for num_drones_**2 times 
    for (int i = 0; i < num_drones_; i++) {
      for (int j = i+1; j < num_drones_; j++) {
        // Check collision between drone i and j
        double dx = fabs(world_to_drone_base_[i].pose.position.x - world_to_drone_base_[j].pose.position.x);
        double dy = fabs(world_to_drone_base_[i].pose.position.y - world_to_drone_base_[j].pose.position.y);
        double dz = fabs(world_to_drone_base_[i].pose.position.z - world_to_drone_base_[j].pose.position.z);

        if ((Eigen::Vector3d(dx, dy, dz)).norm() <= col_tol_) 
        {
          // If collision occured, the collision point is an average of the 2 drone's position
          geometry_msgs::Point collision_point;
          collision_point.x = (world_to_drone_base_[i].pose.position.x + world_to_drone_base_[j].pose.position.x)/2;
          collision_point.y = (world_to_drone_base_[i].pose.position.y + world_to_drone_base_[j].pose.position.y)/2;
          collision_point.z = (world_to_drone_base_[i].pose.position.z + world_to_drone_base_[j].pose.position.z)/2;
        
          swarm_collision_points_.push_back(collision_point);

          if (swarm_collision_points_.size() > 500){
            // Prevent swarm_collision_points_ from getting too large
            swarm_collision_points_.pop_front();
          }
        } 
      }
    }

    std_msgs::ColorRGBA sphere_color;
    sphere_color.r = 1.0;
    sphere_color.g = 0.0;
    sphere_color.b = 0.0;
    sphere_color.a = 0.8;
    
    // Publish point visualization at which inter-agent collision occured
    swarm_collision_pub_.publish(createSphereList(0, swarm_collision_points_, sphere_color));
  }

  visualization_msgs::Marker createSphereList(const int& id, const std::deque<geometry_msgs::Point>& collision_points, const std_msgs::ColorRGBA& color) {

    // Publish collision points
    visualization_msgs::Marker sphere_ls;

    sphere_ls.header.frame_id = "world";
    sphere_ls.header.stamp = ros::Time::now();
    sphere_ls.type = visualization_msgs::Marker::SPHERE_LIST;
    sphere_ls.action = visualization_msgs::Marker::ADD;
    sphere_ls.id = id; 
    sphere_ls.pose.orientation.w = 1.0;

    sphere_ls.color = color;

    sphere_ls.scale.x = 0.4;
    sphere_ls.scale.y = 0.4;
    sphere_ls.scale.z = 0.4;

    for (auto collision_point : collision_points){
      sphere_ls.points.push_back(collision_point);
    }

    return sphere_ls;
  }

  // Helper method to get transformation between 2 frames
  bool getTransform(const std::string &refFrame, const std::string &childFrame, geometry_msgs::PoseStamped &posestamped)
  {
    geometry_msgs::TransformStamped transform;

    try
    {
      transform = tfBuffer_.lookupTransform(refFrame, childFrame, ros::Time(0));
    }
    catch (const tf2::TransformException &ex)
    {
      ROS_ERROR_STREAM(
          "Error in lookupTransform of " << childFrame << " in " << refFrame);
      ROS_WARN("%s",ex.what());
      return false;
    }
    
    posestamped.header.frame_id = transform.header.frame_id;
    posestamped.header.stamp = transform.header.stamp; 

    posestamped.pose.position.x = transform.transform.translation.x;
    posestamped.pose.position.y = transform.transform.translation.y;
    posestamped.pose.position.z = transform.transform.translation.z;

    posestamped.pose.orientation = transform.transform.rotation;

    return true;
  }

private: 
  int num_drones_; // Number of drones

  // TF transformation 
  tf2_ros::Buffer tfBuffer_;
  std::unique_ptr<tf2_ros::TransformListener> tfListener_;

  // Subscribers
  std::vector<ros::Subscriber> drones_pose_sub_; // Subscribers to pose of each UAV agent
  std::vector<ros::Subscriber> collision_sensor_sub_; // Subscribers to collision sensor of each UAV agent

  // Publishers
  // ros::Publisher camera_pos_pub_;
  ros::Publisher swarm_collision_pub_;
  ros::Publisher obs_collision_pub_;

  // Timers
  // ros::Timer pub_camera_pose_timer_;
  ros::Timer broadcast_tf_timer_;

  // Params
  double check_collision_freq_;
  double col_tol_;

  // Flags

  // Stored data
  std::vector<geometry_msgs::PoseStamped> world_to_drone_origin_tfs_; // Position of all drone origin frames relative to world frame
  std::vector<geometry_msgs::PoseStamped> world_to_drone_base_; // Position of all drones base links relative to world frame

  std::deque<geometry_msgs::Point> swarm_collision_points_; // Position of all collision points
  std::deque<geometry_msgs::Point> obs_collision_points_; // Position of all collision points
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "swarm_col_checker");
  ros::NodeHandle nh("~");

  SwarmCollisionChecker swarm_col_checker;

  swarm_col_checker.init(nh);

  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();

  // ros::spin();

  return 0;
}