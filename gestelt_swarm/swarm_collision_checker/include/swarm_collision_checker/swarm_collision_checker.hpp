#ifndef _SWARM_COLLISION_CHECKER_H_
#define _SWARM_COLLISION_CHECKER_H_

#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>

#include "nanoflann.hpp" // For nearest neighbors queries
#include "KDTreeVectorOfVectorsAdaptor.h" // For nearest neighbors queries

class SwarmCollisionChecker
{
public:
  SwarmCollisionChecker() {}

  /**
   * @brief Initialize ROS publishers, subscribers
   * 
   * @param nh 
   * @param pnh 
   */
  void init(ros::NodeHandle &nh, ros::NodeHandle &pnh){
    pnh.param("num_drones", num_drones_, 0);
    std::string pose_topic;
    pnh.param("pose_topic", pose_topic, std::string("mavros/local_position/pose"));
    pnh.param("check_collision_freq", check_collision_freq_, 10.0);
    pnh.param("collision_check/warn_radius", col_warn_radius_, 0.225);
    pnh.param("collision_check/fatal_radius", col_fatal_radius_, 0.14);

    // Publisher for collision visualizations
    collision_viz_pub_ = nh.advertise<visualization_msgs::Marker>("swarm_collision_checker/collision", 10);

    drone_poses_ = std::make_shared<std::vector<Eigen::Vector3d>>();
    // Subscribers
    for (int i = 0; i < num_drones_; i++) {
      std::string pose_topic_indiv = std::string("/drone" + std::to_string(i) + "/" + pose_topic);

      ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(
        pose_topic_indiv, 5, boost::bind(&SwarmCollisionChecker::poseCB, this, _1, i));

      drones_pose_sub_.push_back(pose_sub);

      (*drone_poses_).push_back(Eigen::Vector3d::Constant(999.9));
    }

    node_start_time_ = ros::Time::now().toSec(); // buffer time used to prevent checking for collision until all drone states are received

    // Timers
    check_collision_timer_ = nh.createTimer(ros::Duration(1/check_collision_freq_), &SwarmCollisionChecker::checkCollisionTimerCb, this);
  }

private: 
  // Subscribe to robot pose
  void poseCB(const geometry_msgs::PoseStamped::ConstPtr &msg, int drone_id)
  {
    // ROS_INFO("[SwarmCollisionChecker]: Pose callback for drone %d", drone_id);
    (*drone_poses_)[drone_id] = Eigen::Vector3d{msg->pose.position.x,  msg->pose.position.y,  msg->pose.position.z};
  }

  // Timer callback for checking collision between swarm agentss
  void checkCollisionTimerCb(const ros::TimerEvent &e)
  {
    if (ros::Time::now().toSec() - node_start_time_ < 3.0){
      return;
    }

    const size_t        num_closest = 1;
    std::vector<size_t> out_indices(num_closest);
    std::vector<double> out_distances_sq(num_closest);
    
    for (size_t i = 0; i < num_drones_; i++){ 
      // For every drone, get the nearest neighbor and see if distance is within tolerance
      // If not, publish a collision sphere.

      std::vector<double> query_pt(3);
      query_pt[0] = (*drone_poses_)[i](0);
      query_pt[1] = (*drone_poses_)[i](1);
      query_pt[2] = (*drone_poses_)[i](2);

      std::vector<Eigen::Vector3d> drone_poses_wo_self = *drone_poses_;
      drone_poses_wo_self.erase(drone_poses_wo_self.begin() + i);
      drone_poses_kdtree_ = 
          std::make_unique<KDTreeVectorOfVectorsAdaptor<std::vector<Eigen::Vector3d>, double>>(
              3, drone_poses_wo_self, 10);

      drone_poses_kdtree_->query(&query_pt[0], num_closest, &out_indices[0], &out_distances_sq[0]);

      double dist_to_nearest_drone = sqrt(out_distances_sq[0]);

      if (dist_to_nearest_drone <= col_warn_radius_){
        Eigen::Vector3d drone_nearest_pos = (*drone_poses_)[out_indices[0]];

        publishCollisionSphere(drone_nearest_pos, dist_to_nearest_drone, col_fatal_radius_, col_warn_radius_);
      }
    }
  }

  // Publish sphere representing collision
  void publishCollisionSphere(  
    const Eigen::Vector3d &pos, const double& dist_to_obs, 
    const double& fatal_radius, const double& warn_radius)
  {
    static int col_viz_id = 0;

    visualization_msgs::Marker sphere;
    sphere.header.frame_id = "world";
    sphere.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE;
    sphere.action = visualization_msgs::Marker::ADD;
    sphere.ns = "swarm_collision";
    sphere.pose.orientation.w = 1.0;
    sphere.id = col_viz_id++;

    // Make the alpha and red color value scale from 0.0 to 1.0 depending on the distance to the obstacle. 
    // With the upper limit being the warn_radius, and the lower limit being the fatal_radius
    double fatal_ratio = std::clamp((warn_radius - dist_to_obs)/(warn_radius - fatal_radius), 0.0, 1.001);

    if (fatal_ratio >= 1.0){
      sphere.color.r = 1.0;
      sphere.color.g = 0.0;
      sphere.color.b = 0.0; 
      sphere.color.a = 0.8;
    }
    else {
      // Goes from pink to orange
      sphere.color.r = 1.0;
      sphere.color.g = 0.5;
      sphere.color.b = 1.0 - fatal_ratio*(1.0); // If fatal, make blue value 0.0, so sphere is orange.
      sphere.color.a = 0.3 + (fatal_ratio*(0.8-0.3));
    }

    double scale = fatal_ratio < 1.0 ? 0.35 : 0.6;

    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    sphere.pose.position.x = pos(0);
    sphere.pose.position.y = pos(1);
    sphere.pose.position.z = pos(2);

    collision_viz_pub_.publish(sphere);
  }

private:
  /* ROS objects */
  ros::Timer check_collision_timer_; // Timer for querying KDTree to check collision
  std::vector<ros::Subscriber> drones_pose_sub_; // Vector of subscribers to drone pose

  ros::Publisher collision_viz_pub_;

  /* Data structs */
  std::unique_ptr<KDTreeVectorOfVectorsAdaptor<std::vector<Eigen::Vector3d>, double>>   
    drone_poses_kdtree_; // KD Tree for drone poses

  std::shared_ptr<std::vector<Eigen::Vector3d>> drone_poses_;

  double node_start_time_{-1.0}; // Time that node was started 

  /* Params */

  int num_drones_{-1};

  double check_collision_freq_; // Frequency to check for collisions

  double col_warn_radius_{-1.0};
  double col_fatal_radius_{-1.0};


}; // class SwarmCollisionChecker

#endif // _SWARM_COLLISION_CHECKER_H_
