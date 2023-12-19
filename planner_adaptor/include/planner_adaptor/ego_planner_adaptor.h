#ifndef _EGO_PLANNER_ADAPTOR_H_
#define _EGO_PLANNER_ADAPTOR_H_

#include <planner_adaptor/planner_adaptor.h>

#include <optimizer/poly_traj_utils.hpp>
#include <traj_utils/PolyTraj.h>

class EgoPlannerAdaptor : public PlannerAdaptor
{
public:
  EgoPlannerAdaptor(){};

  virtual void init_planner_specific_topics(ros::NodeHandle& nh){
    planner_goals_pub_ = nh.advertise<gestelt_msgs::Goals>("/planner/goals", 5); 
    plan_traj_sub_ = nh.subscribe("/planner/trajectory", 10, &EgoPlannerAdaptor::planTrajectoryCB, this);
  }

  /**
   * @brief (PLANNER-SPECIFIC IMPLEMENTATION) Forward goals to the planner
   * 
   * @param msg 
   * @return true 
   * @return false 
   */
  virtual void forwardGoals(const std::vector<Eigen::Vector3d> &goal_waypoints){
    gestelt_msgs::Goals goals_msg;
    goals_msg.header.frame_id="world";

    for (auto& pos : goal_waypoints){
      geometry_msgs::Transform tf_msg;
      tf_msg.translation.x = pos(0);
      tf_msg.translation.y = pos(1);
      tf_msg.translation.z = pos(2);
      goals_msg.transforms.push_back(tf_msg);
    }

    planner_goals_pub_.publish(goals_msg);
  }

  /**
   * @brief (PLANNER-SPECIFIC IMPLEMENTATION) Callback for planning trajectory
   * 
   */
  void planTrajectoryCB(traj_utils::PolyTrajPtr msg){
    if (msg->order != 5)
    {
      // Only support trajectory order equals 5 now!
      return;
    }
    if (msg->duration.size() * (msg->order + 1) != msg->coef_x.size())
    {
      // WRONG trajectory parameters
      return;
    }

    // The chunk of code below is just converting the received 
    // trajectories into poly_traj::Trajectory type and storing it

    // piece_nums is the number of Pieces in the trajectory 
    int piece_nums = msg->duration.size();
    std::vector<double> dura(piece_nums);
    std::vector<poly_traj::CoefficientMat> cMats(piece_nums);
    for (int i = 0; i < piece_nums; ++i)
    {
      int i6 = i * 6;
      cMats[i].row(0) << msg->coef_x[i6 + 0], msg->coef_x[i6 + 1], msg->coef_x[i6 + 2],
          msg->coef_x[i6 + 3], msg->coef_x[i6 + 4], msg->coef_x[i6 + 5];
      cMats[i].row(1) << msg->coef_y[i6 + 0], msg->coef_y[i6 + 1], msg->coef_y[i6 + 2],
          msg->coef_y[i6 + 3], msg->coef_y[i6 + 4], msg->coef_y[i6 + 5];
      cMats[i].row(2) << msg->coef_z[i6 + 0], msg->coef_z[i6 + 1], msg->coef_z[i6 + 2],
          msg->coef_z[i6 + 3], msg->coef_z[i6 + 4], msg->coef_z[i6 + 5];

      dura[i] = msg->duration[i];
    }

    traj_.reset(new poly_traj::Trajectory(dura, cMats));

    plan_start_time_ = msg->start_time;
  }

  virtual void samplePlanTimerCB(const ros::TimerEvent &e){
    /* no publishing before receive traj_ and have heartbeat */
    if (isPlannerHBTimeout())
    {
      // No heartbeat from the planner received
      return;
    }

    ros::Time time_now = ros::Time::now();
    // Time elapsed since start of trajectory
    double t_cur = (time_now - plan_start_time_).toSec();

    Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), jer(Eigen::Vector3d::Zero());
    std::pair<double, double> yaw_yawdot(0, 0);

    time_last_ = ros::Time::now();
    // IF time elapsed is below duration of trajectory, then continue to send command
    if (t_cur >= 0.0 && t_cur < traj_->getTotalDuration())
    {
      pos = traj_->getPos(t_cur);
      vel = traj_->getVel(t_cur);
      acc = traj_->getAcc(t_cur);
      jer = traj_->getJer(t_cur);

      /*** calculate yaw ***/
      yaw_yawdot = calculate_yaw(t_cur, pos, (time_now - time_last_).toSec());

      time_last_ = time_now;
    }
    // IF time elapsed is longer then duration of trajectory, then nothing is done
    else if (t_cur >= traj_->getTotalDuration()) // Finished trajectory
    {
    }
    else { // Time is negative
    }

    // Send sampled point to Trajectory Server
    forwardExecTrajectory(pos, vel, acc, jer, type_mask_);
  }

  std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, double dt)
  {
    std::pair<double, double> yaw_yawdot(0, 0);

    Eigen::Vector3d dir = t_cur + time_forward_ <= traj_->getTotalDuration()
                              ? traj_->getPos(t_cur + time_forward_) - pos
                              : traj_->getPos(traj_->getTotalDuration()) - pos;
    double yaw_temp = dir.norm() > 0.1
                          ? atan2(dir(1), dir(0))
                          : last_mission_yaw_;

    double yawdot = 0;
    double d_yaw = yaw_temp - last_mission_yaw_;
    if (d_yaw >= M_PI)
    {
      d_yaw -= 2 * M_PI;
    }
    if (d_yaw <= -M_PI)
    {
      d_yaw += 2 * M_PI;
    }

    const double YDM = d_yaw >= 0 ? YAW_DOT_MAX_PER_SEC : -YAW_DOT_MAX_PER_SEC;
    const double YDDM = d_yaw >= 0 ? YAW_DOT_DOT_MAX_PER_SEC : -YAW_DOT_DOT_MAX_PER_SEC;
    double d_yaw_max;
    if (fabs(last_mission_yaw_dot_ + dt * YDDM) <= fabs(YDM))
    {
      // yawdot = last_mission_yaw_dot_ + dt * YDDM;
      d_yaw_max = last_mission_yaw_dot_ * dt + 0.5 * YDDM * dt * dt;
    }
    else
    {
      // yawdot = YDM;
      double t1 = (YDM - last_mission_yaw_dot_) / YDDM;
      d_yaw_max = ((dt - t1) + dt) * (YDM - last_mission_yaw_dot_) / 2.0;
    }

    if (fabs(d_yaw) > fabs(d_yaw_max))
    {
      d_yaw = d_yaw_max;
    }
    yawdot = d_yaw / dt;

    double yaw = last_mission_yaw_ + d_yaw;
    if (yaw > M_PI)
      yaw -= 2 * M_PI;
    if (yaw < -M_PI)
      yaw += 2 * M_PI;
    yaw_yawdot.first = yaw;
    yaw_yawdot.second = yawdot;

    last_mission_yaw_ = yaw_yawdot.first;
    last_mission_yaw_dot_ = yaw_yawdot.second;

    yaw_yawdot.second = yaw_temp;

    return yaw_yawdot;
  }

private:

  // Plan trajectory data
  boost::shared_ptr<poly_traj::Trajectory> traj_;
  ros::Time plan_start_time_; // Starting time of plan
  ros::Time time_last_; // Time since last sampling
  double time_forward_{1.0}; //Timestep forward

  double last_mission_yaw_{0.0};
  double last_mission_yaw_dot_{0.0};

  u_int16_t type_mask_{2048}; // Default to ignore yaw rate

  // Params
  double YAW_DOT_MAX_PER_SEC{ 2 * M_PI};
  double YAW_DOT_DOT_MAX_PER_SEC{ 5 * M_PI};

// protected:
}; // class EgoPlannerAdaptor

#endif // _EGO_PLANNER_ADAPTOR_H_