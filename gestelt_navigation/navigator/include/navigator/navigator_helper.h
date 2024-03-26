#ifndef _NAVIGATOR_HELPER_
#define _NAVIGATOR_HELPER_

#include <Eigen/Eigen>
#include <memory>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

class Waypoint
{
// Waypoint class is a LIFO queue 

public:
  // Default constructor
  Waypoint(){
  }

  // Reset
  void reset(){
    wp_queue.clear();
  }
  
  /**
   * @brief Add multiple waypoints
   * 
   * @param wp 
   * @return true 
   * @return false 
   */
  bool addMultipleWP(const std::vector<Eigen::Vector3d>& wp_vec){
    // Reverse the waypoints and add on top of the stack
    std::vector<Eigen::Vector3d> wp_vec_reversed = wp_vec;
    std::reverse(wp_vec_reversed.begin(), wp_vec_reversed.end());

    for (auto wp : wp_vec_reversed){
      wp_queue.push_back(wp);
    }

    return true;
  }

  /**
   * @brief Add a single waypoint
   * 
   * @param wp 
   * @return true 
   * @return false 
   */
  bool addWP(const Eigen::Vector3d& wp){
    wp_queue.push_back(wp);
    return true;
  }

  /* Getter methods */

  /**
   * @brief Get the next waypoint
   * 
   * @return Eigen::Vector3d 
   */
  const Eigen::Vector3d& nextWP(){
    return wp_queue.back();
  }

  /**
   * @brief Get all waypoints as a vector
   * 
   * @return const Eigen::Vector3d& 
   */
  const std::vector<Eigen::Vector3d>& getQueue(){
    return wp_queue;
  }

  /**
   * @brief Get the size of the queue
   * 
   * @return Eigen::Vector3d 
   */
  size_t size() const {
    return wp_queue.size();
  }

  /**
   * @brief Get the size of the queue
   * 
   * @return Eigen::Vector3d 
   */
  bool empty() const {
    return wp_queue.empty();
  }

  /* Setter methods */

  /**
   * @brief Pop the last waypoint
   * 
   */
  void popWP() {
    if (wp_queue.empty()){
        return;
    }
    else {
      wp_queue.pop_back();
    }
  }

private:
  std::vector<Eigen::Vector3d> wp_queue;
};

template<typename ... Args>
std::string str_fmt( const std::string& format, Args ... args )
{
  int size_s = std::snprintf( nullptr, 0, format.c_str(), args ... ) + 1; // Extra space for '\0'
  if( size_s <= 0 ){ throw std::runtime_error( "Error during formatting." ); }
  auto size = static_cast<size_t>( size_s );
  std::unique_ptr<char[]> buf( new char[ size ] );
  std::snprintf( buf.get(), size, format.c_str(), args ... );
  return std::string( buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
}

/* Visualization methods */

namespace viz_helper{

  /**
   * @brief Publish cubes for visualization
   * 
   * @param closed_list 
   */
  inline void publishVizCubes(const std::vector<Eigen::Vector3d>& closed_list, const std::string& frame_id, ros::Publisher& publisher) {
    visualization_msgs::Marker closed_nodes;
    double cube_length = 0.1;
    double alpha = 0.5;

    closed_nodes.header.frame_id = frame_id;
    closed_nodes.header.stamp = ros::Time::now();
    closed_nodes.type = visualization_msgs::Marker::CUBE_LIST;
    closed_nodes.action = visualization_msgs::Marker::ADD;
    closed_nodes.id = 0; 
    closed_nodes.pose.orientation.w = 1.0;

    closed_nodes.color.r = 1.0;
    closed_nodes.color.g = 1.0;
    closed_nodes.color.b = 1.0;
    closed_nodes.color.a = alpha;

    closed_nodes.scale.x = cube_length;
    closed_nodes.scale.y = cube_length;
    closed_nodes.scale.z = cube_length;

    geometry_msgs::Point pt;
    for (auto pos : closed_list) {
      pt.x = pos(0);
      pt.y = pos(1);
      pt.z = pos(2);
      closed_nodes.points.push_back(pt);
    }

    publisher.publish(closed_nodes);
  }

  /**
   * @brief Publish spheres for visualization
   * 
   * @param path 
   */
  inline void publishVizSpheres(const std::vector<Eigen::Vector3d>& pts, const std::string& frame_id, ros::Publisher& publisher) {
    visualization_msgs::Marker sphere_list;
    double radius = 0.15;
    double alpha = 0.8;

    sphere_list.header.frame_id = frame_id;
    sphere_list.header.stamp = ros::Time::now();
    sphere_list.type = visualization_msgs::Marker::SPHERE_LIST;
    sphere_list.action = visualization_msgs::Marker::ADD;
    sphere_list.ns = "viz_spheres"; 
    sphere_list.id = 1; 
    sphere_list.pose.orientation.w = 1.0;

    sphere_list.color.r = 1.0;
    sphere_list.color.g = 0.5;
    sphere_list.color.b = 0.0;
    sphere_list.color.a = alpha;

    sphere_list.scale.x = radius;
    sphere_list.scale.y = radius;
    sphere_list.scale.z = radius;

    geometry_msgs::Point pt;
    for (int i = 0; i < pts.size(); i++){
      pt.x = pts[i](0);
      pt.y = pts[i](1);
      pt.z = pts[i](2);

      sphere_list.points.push_back(pt);
    }

    publisher.publish(sphere_list);
  }

  // /**
  //  * @brief Publish path for visualization
  //  * 
  //  * @param path 
  //  */
  // void publishVizSpheres(const std::vector<Eigen::Vector3d>& path, const std::string& frame_id, ros::Publisher& publisher) {
  //   visualization_msgs::Marker path_spheres, start_sphere, goal_sphere, path_line_strip;

  //   start_sphere.header.frame_id = goal_sphere.header.frame_id = frame_id;
  //   start_sphere.header.stamp = goal_sphere.header.stamp = ros::Time::now();
  //   start_sphere.type = goal_sphere.type = visualization_msgs::Marker::SPHERE;
  //   start_sphere.action = goal_sphere.action = visualization_msgs::Marker::ADD;
  //   start_sphere.id = 1;
  //   goal_sphere.id = 2; 
  //   start_sphere.pose.orientation.w = goal_sphere.pose.orientation.w = 1.0;

  //   start_sphere.color.r = goal_sphere.color.r = 0.0;
  //   start_sphere.color.g = goal_sphere.color.g = 0.0;
  //   start_sphere.color.b = goal_sphere.color.b = 1.0;
  //   start_sphere.color.a = goal_sphere.color.a = 0.6;

  //   start_sphere.scale.x = goal_sphere.scale.x = 0.2;
  //   start_sphere.scale.y = goal_sphere.scale.y = 0.2;
  //   start_sphere.scale.z = goal_sphere.scale.z = 0.2;

  //   path_line_strip.header.frame_id = frame_id;
  //   path_line_strip.header.stamp = ros::Time::now();
  //   path_line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  //   path_line_strip.action = visualization_msgs::Marker::ADD;
  //   path_line_strip.id = 1000;
  //   path_line_strip.pose.orientation.w = 1.0;

  //   path_line_strip.color.r = 0.0;
  //   path_line_strip.color.g = 0.0;
  //   path_line_strip.color.b = 1.0;
  //   path_line_strip.color.a = 0.6;

  //   path_line_strip.scale.x = 0.1;

  //   path_spheres.header.frame_id = frame_id;
  //   path_spheres.header.stamp = ros::Time::now();
  //   path_spheres.type = visualization_msgs::Marker::SPHERE_LIST;
  //   path_spheres.action = visualization_msgs::Marker::ADD;
  //   path_spheres.ns = "front_end_plan_spheres"; 
  //   path_spheres.id = 995; 
  //   path_spheres.pose.orientation.w = 1.0;

  //   path_spheres.color.r = 0.0;
  //   path_spheres.color.g = 0.0;
  //   path_spheres.color.b = 1.0;
  //   path_spheres.color.a = 0.6;

  //   path_spheres.scale.x = 0.1;
  //   path_spheres.scale.y = 0.1;
  //   path_spheres.scale.z = 0.1;

  //   start_sphere.pose.position.x = path[0](0);
  //   start_sphere.pose.position.y = path[0](1);
  //   start_sphere.pose.position.z = path[0](2);

  //   geometry_msgs::Point pt;
  //   for (int i = 1; i < path.size() - 1; i++){
  //     pt.x = path[i](0);
  //     pt.y = path[i](1);
  //     pt.z = path[i](2);

  //     path_spheres.points.push_back(pt);
  //     // path_line_strip.points.push_back(pt);
  //   }

  //   goal_sphere.pose.position.x = path.back()(0);
  //   goal_sphere.pose.position.y = path.back()(1);
  //   goal_sphere.pose.position.z = path.back()(2);

  //   // publisher.publish(start_sphere);
  //   // publisher.publish(goal_sphere);
  //   publisher.publish(path_spheres);
  //   // publisher.publish(path_line_strip);
  // }

} // namespace viz_helper


#endif // _NAVIGATOR_HELPER_