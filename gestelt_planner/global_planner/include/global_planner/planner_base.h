#ifndef _PLANNER_BASE_H_
#define _PLANNER_BASE_H_

#include <algorithm>
#include <Eigen/Eigen>

#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

#include <global_planner/planner_common.h>

class PlannerBase
{
public:
  virtual bool generatePlan(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &goal_pos) = 0;
protected:
  PlannerBase(){};
};

#endif // _PLANNER_BASE_H_