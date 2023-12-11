#ifndef _PLANNER_BASE_H_
#define _PLANNER_BASE_H_

#include <algorithm>
#include <unordered_set>
#include <Eigen/Eigen>

#include <std_msgs/Empty.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <global_planner/planner_common.h>

using namespace Eigen;

class PlannerBase
{
public:
  virtual bool generate_plan(Vector3d start_pos, Vector3d goal_pos) = 0;

protected:
  PlannerBase(){};
};

#endif