#ifndef _PLANNER_ADAPTOR_H_
#define _PLANNER_ADAPTOR_H_

#include <ros/ros.h>

#include <Eigen/Eigen>

#include <std_msgs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <gestelt_msgs/CommanderCommand.h>
#include <gestelt_msgs/CommanderGoals.h>
#include <gestelt_msgs/CommanderTrajectory.h>

class PlannerAdaptor
{
public:

  /**
   * @brief Initialize the PlannerAdaptor ROS node
   * 
   * @param nh 
   */
  virtual void init(ros::NodeHandle& nh){
    nh.param("planner_heartbeat_timeout", planner_heartbeat_timeout_, 0.5);
    nh.param("ignore_heartbeat_checks", ignore_heartbeat_checks_, false);
    double checks_timer_freq;
    nh.param("checks_timer_freq", checks_timer_freq, 10.0);

    heartbeat_sub_ = nh.subscribe("/planner/heartbeat", 10, &PlannerAdaptor::plannerHeartbeatCB, this);
    goals_sub_ = nh.subscribe("/planner/goals", 10, &PlannerAdaptor::goalsCB, this);

    exec_traj_pub_ = nh.advertise<gestelt_msgs::CommanderTrajectoryPoint>("/planner/exec_trajectory", 50);

    checks_timer_ = nh.createTimer(ros::Duration(1/checks_timer_freq), &PlannerAdaptor::checksTimerCB, this);

    // plan_traj_sub_ = nh.subscribe("/planner/plan_trajectory", 10, &PlannerAdaptor::planTrajectoryCB, this);
  }

  /**
   * @brief Planner heartbeat callback
   * 
   */
  virtual void plannerHeartbeatCB(const std_msgs::EmptyConstPtr& msg){
    last_planner_heartbeat_time_ = ros::Time::now().toSec();
  }

  /**
   * @brief Goal callback
   * 
   */
  virtual void goalsCB(const trajectory_msgs::CommanderGoals& msg){
    std::vector<Eigen::Vector3d> goal_waypoints;

    for (auto& pos : msg->positions){
      goal_waypoints.push_back(Eigen::Vector3d{pos.translation.x, pos.translation.y, pos.translation.z});
    }

    forwardGoals(goal_waypoints);
  }

  /**
   * @brief Timer callback for crucial checks (such as planner heartbeat timeout)  
   * 
   * @param e 
   */
  virtual void checksTimerCB(const ros::TimerEvent &e){
    if (!ignore_heartbeat_checks_){
      if ((ros::Time::now().toSec() - last_planner_heartbeat_time_) > planner_heartbeat_timeout_){
        // If planner heartbeat timeout exceeded, send emergency stop
        gestelt_msgs::CommanderCommand traj_server_cmd_msg;
        traj_server_cmd_msg.header.stamp = ros::Time::now(); 
        traj_server_cmd_msg.command = gestelt_msgs::CommanderCommand::E_STOP;
        traj_server_cmd_pub_.publish(traj_server_cmd_msg);
      }
    }
  }

  /**
   * @brief Forward executable trajectory
   * 
   */
  virtual void forwardExecTrajectory(
    const Eigen::Vector3d& pos, 
    const Eigen::Vector3d& vel, 
    const Eigen::Vector3d& acc, 
    const Eigen::Vector3d& jerk)
  {
    gestelt_msg::CommanderTrajectoryPoint cmd_traj_msg; 

    cmd_traj_msg.transform.translation.x = pos(0);
    cmd_traj_msg.transform.translation.y = pos(1);
    cmd_traj_msg.transform.translation.z = pos(2);
    
    cmd_traj_msg.velocity.linear.x = vel(0);
    cmd_traj_msg.velocity.linear.y = vel(1);
    cmd_traj_msg.velocity.linear.z = vel(2);

    cmd_traj_msg.acceleration.linear.x = acc(0);
    cmd_traj_msg.acceleration.linear.y = acc(1);
    cmd_traj_msg.acceleration.linear.z = acc(2);

    cmd_traj_msg.jerk.linear.x = jerk(0);
    cmd_traj_msg.jerk.linear.y = jerk(1);
    cmd_traj_msg.jerk.linear.z = jerk(2);

    exec_traj_pub_.publish(cmd_traj_msg);
  }

  /**
   * @brief (PLANNER-SPECIFIC IMPLEMENTATION) Callback for planning trajectory
   * 
   */
  virtual void planTrajectoryCB( ... ){
    // Receive planned trajectory message
    // Sample according to current time
    // Send sample to Trajectory Server
    // forwardExecTrajectory(sample)    
  }


  /**
   * @brief (PLANNER-SPECIFIC IMPLEMENTATION) Forward goals to the planner
   * 
   * @param msg 
   * @return true 
   * @return false 
   */
  virtual void forwardGoals(const std::vector<Eigen::Vector3d> &goal_waypoints) = 0;

  /**
   * @brief (PLANNER-SPECIFIC IMPLEMENTATION) Sample the executable trajectory
   * 
   * @param goal_waypoints 
   * @return true 
   * @return false 
   */
  virtual void sampleExecTrajectory() = 0;

  /**
   * Params
   */
  bool ignore_heartbeat_checks_{false}; // True if heartbeat checks are ignored
  double planner_heartbeat_timeout_{0.5}; // Planner heartbeat timeout
  double last_planner_heartbeat_time_{0}; // Last timestamp of planner heartbeat

  /**
   * ROS Subscribers, Publishers and Timers
   */
  ros::Subscriber heartbeat_sub_; // Subscriber for heartbeat
  ros::Subscriber goals_sub_; // Subscriber for goals
  ros::Subscriber plan_traj_sub_; // Subscriber for plan trajectory server 

  ros::Publisher exec_traj_pub_; // Publisher for plan trajectory  
  ros::Publisher traj_server_cmd_pub_; // Publisher of commands to trajectory server 

  ros::Timer checks_timer_; 

// protected:
//   PlannerAdaptor(){};

}; // class PlannerAdaptor

#endif // _PLANNER_ADAPTOR_H_