#ifndef _SWARM_BRIDGE_H_
#define _SWARM_BRIDGE_H_

#include <ros/ros.h>

#include <traj_utils/MINCOTraj.h>

class SwarmBridge
{
public:
  SwarmBridge(ros::NodeHandle &nh, ros::NodeHandle &pnh) {
    initROS(nh, pnh);
  }

  /**
   * @brief Initialize ROS publishers, subscribers
   * 
   * @param nh 
   * @param pnh 
   */
  void initROS(ros::NodeHandle &nh, ros::NodeHandle &pnh){
    swarm_minco_traj_sub_ = nh.subscribe("swarm/local/minco_traj_out", 100,
                                          &SwarmBridge::swarmMincoTrajCB,
                                          this,
                                          ros::TransportHints().tcpNoDelay());

    swarm_minco_traj_sub_ = nh.subscribe("/swarm/global/minco_traj", 100,
                                          &SwarmBridge::swarmMincoTrajCB,
                                          this,
                                          ros::TransportHints().tcpNoDelay());

    swarm_minco_traj_pub_ = nh.advertise<traj_utils::MINCOTraj>("/swarm/global/minco_traj", 10);
  }

  void swarmMincoTrajCB(const traj_utils::MINCOTrajConstPtr &msg){
    // Re-broadcast trajectories
    swarm_minco_traj_pub_.publish(msg);
  }

private:
  ros::Subscriber swarm_minco_traj_sub_;
  ros::Publisher swarm_minco_traj_pub_;
  
}; // class SwarmBridge

#endif // _SWARM_BRIDGE_H_
