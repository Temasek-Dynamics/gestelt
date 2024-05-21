#ifndef _PLANNER_ADAPTOR_H_
#define _PLANNER_ADAPTOR_H_

#include <ros/ros.h>

#include <Eigen/Eigen>

#include <std_msgs/Empty.h>
#include <geometry_msgs/Transform.h>

#include <gestelt_msgs/Command.h>
#include <gestelt_msgs/Goals.h>
#include <gestelt_msgs/ExecTrajectory.h>

class PlannerAdaptor
{
public:

  /**
   * @brief Initialize the PlannerAdaptor ROS node
   * 
   * @param nh 
   */
  virtual void init(ros::NodeHandle& nh, ros::NodeHandle& pnh){
    pnh.param("planner_hb_timeout", planner_heartbeat_timeout_, 0.5);
    pnh.param("ignore_hb_checks", ignore_heartbeat_checks_, false);
    double checks_freq, sample_plan_freq;
    pnh.param("checks_freq", checks_freq, 10.0);
    pnh.param("sample_plan_freq", sample_plan_freq, 50.0);

    // Subscribers
    adaptor_goals_sub_ = nh.subscribe("planner_adaptor/goals", 10, &PlannerAdaptor::goalsCB, this); // Subscription to user-defined goals, these goals are processed and sent to the planner
    heartbeat_sub_ = nh.subscribe("planner/heartbeat", 10, &PlannerAdaptor::plannerHeartbeatCB, this); // Subscription to hearbeat from the planner

    // Publishers
    traj_server_cmd_pub_ = nh.advertise<gestelt_msgs::Command>("traj_server/command", 1); // Publishes commands to trajectory server
    exec_traj_pub_ = nh.advertise<gestelt_msgs::ExecTrajectory>("planner_adaptor/exec_trajectory", 3); // Trajectory points to be published to Trajectory Server

    // Timers
    checks_timer_ = nh.createTimer(ros::Duration(1/checks_freq), &PlannerAdaptor::checksTimerCB, this); // Timer for checking planner heartbeat.
    sample_plan_timer_ = nh.createTimer(ros::Duration(1/sample_plan_freq), &PlannerAdaptor::samplePlanTimerCB, this); // Timer for sampling planner trajectory

    // Planner-specific code
    init_planner_specific_topics(nh);
  }

  /**
   * @brief Planner heartbeat callback
   * 
   */
  virtual void plannerHeartbeatCB(const std_msgs::Empty::ConstPtr& msg){
    last_planner_heartbeat_time_ = ros::Time::now().toSec();
  }

  /**
   * @brief Goal callback. Could be modified to pass on velocities and accelerations as goal waypoints to the planner
   * 
   */
  virtual void goalsCB(const gestelt_msgs::Goals::ConstPtr& msg){
    std::vector<Eigen::Vector3d> goal_waypoints;

    for (auto& pos : msg->transforms){
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
      if (isPlannerHBTimeout()){
        
        std::cerr << "PLANNER HEARTBEAT TIMEOUT!!" << std::endl;

        // If planner heartbeat timeout exceeded, send emergency stop
        gestelt_msgs::Command traj_server_cmd_msg;
        traj_server_cmd_msg.header.stamp = ros::Time::now(); 
        traj_server_cmd_msg.command = gestelt_msgs::Command::E_STOP;
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
    const Eigen::Quaterniond& quat, 
    const Eigen::Vector3d& vel, 
    const Eigen::Vector3d& acc, 
    const Eigen::Vector3d& jerk,
    const int16_t type_mask)
  {
    gestelt_msgs::ExecTrajectory exec_traj_msg; 

    exec_traj_msg.header.stamp = ros::Time::now(); 
    exec_traj_msg.header.frame_id = "world";

    exec_traj_msg.type_mask = type_mask;

    exec_traj_msg.transform.translation.x = pos(0);
    exec_traj_msg.transform.translation.y = pos(1);
    exec_traj_msg.transform.translation.z = pos(2);

    exec_traj_msg.transform.rotation.x = quat.x();
    exec_traj_msg.transform.rotation.y = quat.y();
    exec_traj_msg.transform.rotation.z = quat.z();
    exec_traj_msg.transform.rotation.w = quat.w();
    
    exec_traj_msg.velocity.linear.x = vel(0);
    exec_traj_msg.velocity.linear.y = vel(1);
    exec_traj_msg.velocity.linear.z = vel(2);

    exec_traj_msg.acceleration.linear.x = acc(0);
    exec_traj_msg.acceleration.linear.y = acc(1);
    exec_traj_msg.acceleration.linear.z = acc(2);

    exec_traj_msg.jerk.linear.x = jerk(0);
    exec_traj_msg.jerk.linear.y = jerk(1);
    exec_traj_msg.jerk.linear.z = jerk(2);

    exec_traj_pub_.publish(exec_traj_msg);
  }

  /**
   * @brief Indicates if planner heartbeat has timed out
   * 
   * @return true 
   * @return false 
   */
  bool isPlannerHBTimeout() {
    if (ignore_heartbeat_checks_){
      return false;
    }
    return (ros::Time::now().toSec() - last_planner_heartbeat_time_) > planner_heartbeat_timeout_;
  }

  /**
   * @brief (PLANNER-SPECIFIC IMPLEMENTATION) Initialization of planner-specific subscriptions and publishers 
   * 
   */
  virtual void init_planner_specific_topics(ros::NodeHandle& nh) = 0;
  // {
  //   planner_goals_pub_ = nh.advertise<msg::msg>("/planner/goals", 5); // Goals to be published to planner
  //   plan_traj_sub_ = nh.subscribe("/planner_adaptor/plan_trajectory", 10, &PlannerAdaptor::planTrajectoryCB, this); // Subscription to planner trajectory
  // }

  /**
   * @brief Timer callback for crucial checks (such as planner heartbeat timeout)  
   * 
   * @param e 
   */
  virtual void samplePlanTimerCB(const ros::TimerEvent &e) = 0;


  /**
   * @brief (PLANNER-SPECIFIC IMPLEMENTATION) Forward goals to the planner
   * 
   * @param msg 
   * @return true 
   * @return false 
   */
  virtual void forwardGoals(const std::vector<Eigen::Vector3d> &goal_waypoints) = 0;
  // {
  //   planner_goals_pub_.publish(goals);
  // }

  // /**
  //  * @brief (PLANNER-SPECIFIC IMPLEMENTATION) Callback for planning trajectory
  //  * 
  //  */
  // void planTrajectoryCB(msg::msgConstPtr& msg){
  //   // Save the message into a class member for use in trajectory sampling timer callback
  // }

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
  ros::Subscriber adaptor_goals_sub_; // Subscriber for goals
  ros::Subscriber plan_traj_sub_; // Subscriber for plan trajectory server 

  ros::Publisher exec_traj_pub_; // Publisher for plan trajectory  
  ros::Publisher traj_server_cmd_pub_; // Publisher of commands to trajectory server 

  ros::Publisher planner_goals_pub_; // Publisher of goals to planner 

  ros::Timer checks_timer_; // Timer for performing checks on the planner
  ros::Timer sample_plan_timer_; // Timer for sampling planner trajectory  

protected:
  PlannerAdaptor(){};

}; // class PlannerAdaptor

#endif // _PLANNER_ADAPTOR_H_