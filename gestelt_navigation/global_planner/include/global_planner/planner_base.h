#ifndef _PLANNER_BASE_H_
#define _PLANNER_BASE_H_

#include <Eigen/Eigen>

#include <global_planner/planner_common.h>

class PlannerBase
{
public:
  virtual bool generatePlan(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &goal_pos) = 0;

  // Get refined planned path
  virtual std::vector<Eigen::Vector3d> getPathPos() = 0;

  // Get raw planned path
  virtual std::vector<Eigen::Vector3d> getPathPosRaw() = 0;

  /**
   * @brief Add ROS Publishers 
   * 
   * @param publisher_map 
   */
  virtual void addPublishers(std::unordered_map<std::string, ros::Publisher> &publisher_map) = 0;

protected:
  PlannerBase(){};
}; // class PlannerBase

#endif // _PLANNER_BASE_H_