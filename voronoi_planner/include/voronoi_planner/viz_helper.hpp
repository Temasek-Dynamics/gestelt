#ifndef _VIZ_HELPER_HPP
#define _VIZ_HELPER_HPP

#include <Eigen/Eigen>

#include <ros/ros.h>

#include <visualization_msgs/Marker.h>

namespace viz_helper{

  inline void publishStartAndGoal(
    const Eigen::Vector3d& map_start, 
    const Eigen::Vector3d& map_goal, 
    const std::string& frame_id, 
    ros::Publisher& publisher1, ros::Publisher& publisher2)
  {
    visualization_msgs::Marker start_sphere, goal_sphere;
    double radius = 0.1;
    double alpha = 0.5; 

    /* Start/goal sphere*/
    start_sphere.header.frame_id = goal_sphere.header.frame_id = frame_id;
    start_sphere.header.stamp = goal_sphere.header.stamp = ros::Time::now();
    start_sphere.ns = goal_sphere.ns = "start_goal_points";
    start_sphere.type = goal_sphere.type = visualization_msgs::Marker::CUBE;
    start_sphere.action = goal_sphere.action = visualization_msgs::Marker::ADD;
    start_sphere.id = 1;
    goal_sphere.id = 2; 
    start_sphere.pose.orientation.w = goal_sphere.pose.orientation.w = 1.0;

    start_sphere.color.r = 1.0; 
    start_sphere.color.g = 1.0; 
    start_sphere.color.b = 0.0; 
    start_sphere.color.a = goal_sphere.color.a = alpha;

    goal_sphere.color.r = 0.0;
    goal_sphere.color.g = 1.0;
    goal_sphere.color.b = 0.0;

    start_sphere.scale.x = goal_sphere.scale.x = radius;
    start_sphere.scale.y = goal_sphere.scale.y = radius;
    start_sphere.scale.z = goal_sphere.scale.z = radius;

    /* Set Start */
    start_sphere.pose.position.x = map_start(0);
    start_sphere.pose.position.y = map_start(1);
    start_sphere.pose.position.z = map_start(2);

    /* Set Goal */
    goal_sphere.pose.position.x = map_goal(0);
    goal_sphere.pose.position.y = map_goal(1);
    goal_sphere.pose.position.z = map_goal(2);

    publisher1.publish(start_sphere);
    publisher2.publish(goal_sphere);
  }

  inline void publishSpaceTimePath(const std::vector<Eigen::Vector4d>& path, 
                                  const std::string& frame_id, ros::Publisher& publisher) {
    visualization_msgs::Marker cube;
    double radius = 0.1;
    double alpha = 0.8; 

    /* Fixed parameters */
    cube.header.frame_id = frame_id;
    cube.header.stamp = ros::Time::now();
    cube.ns = "front_end_path";
    cube.type = visualization_msgs::Marker::CUBE;
    cube.action = visualization_msgs::Marker::ADD;
    cube.pose.orientation.w = 1.0;
    cube.color.a = alpha;

    // size
    cube.scale.x = radius;
    cube.scale.y = radius;
    cube.scale.z = radius;

    for (size_t i = 0; i < path.size(); i++)
    {
      cube.id = i; 

      cube.pose.position.x = path[i](0);
      cube.pose.position.y = path[i](1);
      cube.pose.position.z = path[i](2);

      // Make the color value scale from 0.0 to 1.0 depending on the distance to the goal. 
      double time_ratio = std::clamp((path[i](3))/path.size(), 0.0, 1.0);

      cube.color.r = time_ratio ;
      cube.color.g = 0.0; 
      cube.color.b = 1.0 - time_ratio; 

      publisher.publish(cube);

      ros::Duration(0.005).sleep();
    }
  }

  inline void publishFrontEndPath(const std::vector<Eigen::Vector3d>& path, 
                                  const std::string& frame_id, ros::Publisher& publisher) {
    visualization_msgs::Marker wp_sphere_list, path_line_strip;
    visualization_msgs::Marker start_sphere, goal_sphere;
    double radius = 0.3;
    double alpha = 0.8; 

    geometry_msgs::Point pt;

    /* Start/goal sphere*/
    start_sphere.header.frame_id = goal_sphere.header.frame_id = frame_id;
    start_sphere.header.stamp = goal_sphere.header.stamp = ros::Time::now();
    start_sphere.ns = goal_sphere.ns = "start_end_points";
    start_sphere.type = goal_sphere.type = visualization_msgs::Marker::SPHERE;
    start_sphere.action = goal_sphere.action = visualization_msgs::Marker::ADD;
    start_sphere.id = 0;
    goal_sphere.id = 1; 
    start_sphere.pose.orientation.w = goal_sphere.pose.orientation.w = 1.0;

    start_sphere.color.r = 1.0; 
    start_sphere.color.g = 1.0; 
    start_sphere.color.b = 0.0; 
    start_sphere.color.a = goal_sphere.color.a = alpha;

    goal_sphere.color.r = 0.0;
    goal_sphere.color.g = 1.0;
    goal_sphere.color.b = 0.0;

    start_sphere.scale.x = goal_sphere.scale.x = radius;
    start_sphere.scale.y = goal_sphere.scale.y = radius;
    start_sphere.scale.z = goal_sphere.scale.z = radius;

    /* wp_sphere_list: Sphere list (Waypoints) */
    wp_sphere_list.header.frame_id = frame_id;
    wp_sphere_list.header.stamp = ros::Time::now();
    wp_sphere_list.ns = "front_end_sphere_list"; 
    wp_sphere_list.type = visualization_msgs::Marker::SPHERE_LIST;
    wp_sphere_list.action = visualization_msgs::Marker::ADD;
    wp_sphere_list.id = 1; 
    wp_sphere_list.pose.orientation.w = 1.0;

    wp_sphere_list.color.r = 1.0;
    wp_sphere_list.color.g = 0.5;
    wp_sphere_list.color.b = 0.0;
    wp_sphere_list.color.a = alpha;

    wp_sphere_list.scale.x = radius;
    wp_sphere_list.scale.y = radius;
    wp_sphere_list.scale.z = radius;

    /* path_line_strip: Line strips (Connecting waypoints) */
    path_line_strip.header.frame_id = frame_id;
    path_line_strip.header.stamp = ros::Time::now();
    path_line_strip.ns = "front_end_path_lines"; 
    path_line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    path_line_strip.action = visualization_msgs::Marker::ADD;
    path_line_strip.id = 1;
    path_line_strip.pose.orientation.w = 1.0;

    path_line_strip.color.r = 1.0;
    path_line_strip.color.g = 0.5;
    path_line_strip.color.b = 0.0;
    path_line_strip.color.a = alpha * 0.75;

    path_line_strip.scale.x = radius * 0.5;

    start_sphere.pose.position.x = path[0](0);
    start_sphere.pose.position.y = path[0](1);
    start_sphere.pose.position.z = path[0](2);

    pt.x = path[0](0);
    pt.y = path[0](1);
    pt.z = path[0](2);
    path_line_strip.points.push_back(pt);

    for (size_t i = 1; i < path.size()-1; i++){
      pt.x = path[i](0);
      pt.y = path[i](1);
      pt.z = path[i](2);

      wp_sphere_list.points.push_back(pt);
      path_line_strip.points.push_back(pt);
    }

    pt.x = path.back()(0);
    pt.y = path.back()(1);
    pt.z = path.back()(2);
    path_line_strip.points.push_back(pt);

    goal_sphere.pose.position.x = path.back()(0);
    goal_sphere.pose.position.y = path.back()(1);
    goal_sphere.pose.position.z = path.back()(2);

    publisher.publish(start_sphere);
    publisher.publish(goal_sphere);
    publisher.publish(wp_sphere_list);
    publisher.publish(path_line_strip);
  }

  inline void publishVertices(const std::vector<Eigen::Vector3d>& vor_verts, const std::string& frame_id, ros::Publisher& publisher)
  {
    visualization_msgs::Marker vertices;
    double radius = 0.2;
    double alpha = 0.8; 

    /* vertices: Sphere list (voronoi graph vertices) */
    vertices.header.frame_id = frame_id;
    vertices.header.stamp = ros::Time::now();
    vertices.ns = "voronoi_vertices"; 
    vertices.type = visualization_msgs::Marker::SPHERE_LIST;
    vertices.action = visualization_msgs::Marker::ADD;
    vertices.id = 1; 
    vertices.pose.orientation.w = 1.0;

    vertices.color.r = 0.0;
    vertices.color.g = 0.5;
    vertices.color.b = 1.0;
    vertices.color.a = alpha;

    vertices.scale.x = radius;
    vertices.scale.y = radius;
    vertices.scale.z = radius;

    geometry_msgs::Point pt;
    for (size_t i = 0; i < vor_verts.size(); i++){
      pt.x = vor_verts[i](0);
      pt.y = vor_verts[i](1);
      pt.z = vor_verts[i](2);

      vertices.points.push_back(pt);
    }

    publisher.publish(vertices);
  }

  inline void publishClosedList(const std::vector<Eigen::Vector3d>& pts, 
                        ros::Publisher& publisher, const std::string& frame_id = "map")
{
  if (pts.empty()){
    return;
  }

  Eigen::Vector3d color = Eigen::Vector3d{0.0, 0.0, 0.0};
  double radius = 0.1;
  double alpha = 0.7;

  visualization_msgs::Marker sphere_list;

  sphere_list.header.frame_id = frame_id;
  sphere_list.header.stamp = ros::Time::now();
  sphere_list.type = visualization_msgs::Marker::SPHERE_LIST;
  sphere_list.action = visualization_msgs::Marker::ADD;
  sphere_list.ns = "closed_list"; 
  sphere_list.id = 0; 
  sphere_list.pose.orientation.w = 1.0;

  sphere_list.color.r = color(0);
  sphere_list.color.g = color(1);
  sphere_list.color.b = color(2);
  sphere_list.color.a = alpha;

  sphere_list.scale.x = radius;
  sphere_list.scale.y = radius;
  sphere_list.scale.z = radius;

  geometry_msgs::Point pt;
  for (size_t i = 0; i < pts.size(); i++){
    pt.x = pts[i](0);
    pt.y = pts[i](1);
    pt.z = pts[i](2);

    sphere_list.points.push_back(pt);
  }

  publisher.publish(sphere_list);
}
}

#endif // _VIZ_HELPER_HPP
