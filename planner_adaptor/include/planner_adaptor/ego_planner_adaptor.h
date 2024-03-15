#ifndef _EGO_PLANNER_ADAPTOR_H_
#define _EGO_PLANNER_ADAPTOR_H_

#include <planner_adaptor/planner_adaptor.h>

#include <optimizer/poly_traj_utils.hpp>
#include <traj_utils/PolyTraj.h>
class EgoPlannerAdaptor : public PlannerAdaptor
{
public:
  EgoPlannerAdaptor(){
    time_since_last_samp_ = ros::Time::now();
  };

  virtual void init_planner_specific_topics(ros::NodeHandle& nh){
    planner_goals_pub_ = nh.advertise<gestelt_msgs::Goals>("planner/goals", 5); 
    plan_traj_sub_ = nh.subscribe("back_end/trajectory", 10, 
                                  &EgoPlannerAdaptor::planTrajectoryCB, this);
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

    // num_segs: number of trajectory segments 
    size_t num_segs = msg->duration.size();
    std::vector<double> dura(num_segs); // vector of segment time durations
    std::vector<poly_traj::CoefficientMat> cMats(num_segs);
    for (int i = 0; i < num_segs; i++)
    {
      int i6 = i * 6;
      cMats[i].row(0) <<  msg->coef_x[i6 + 0], msg->coef_x[i6 + 1], msg->coef_x[i6 + 2],
                          msg->coef_x[i6 + 3], msg->coef_x[i6 + 4], msg->coef_x[i6 + 5];
      cMats[i].row(1) <<  msg->coef_y[i6 + 0], msg->coef_y[i6 + 1], msg->coef_y[i6 + 2],
                          msg->coef_y[i6 + 3], msg->coef_y[i6 + 4], msg->coef_y[i6 + 5];
      cMats[i].row(2) <<  msg->coef_z[i6 + 0], msg->coef_z[i6 + 1], msg->coef_z[i6 + 2],
                          msg->coef_z[i6 + 3], msg->coef_z[i6 + 4], msg->coef_z[i6 + 5];

      dura[i] = msg->duration[i];
    }

    be_traj_mutex_.lock();
    be_traj_.reset(new poly_traj::Trajectory(dura, cMats));
    be_traj_->setGlobalStartTime(msg->start_time.toSec());
    be_traj_mutex_.unlock();
  }

  virtual void samplePlanTimerCB(const ros::TimerEvent &e){
    // No heartbeat from the planner received
    // or plan has not been received
    if (isPlannerHBTimeout())
    {
      // ROS_INFO_THROTTLE(5.0, "[EGO Planner Adaptor] Planner heartbeat timeout");
      return;
    }
    if (!be_traj_)
    {
      // ROS_INFO_THROTTLE(5.0, "[EGO Planner Adaptor] No trajectory received!");
      return;
    }

    ros::Time time_now = ros::Time::now();

    be_traj_mutex_.lock();
    poly_traj::Trajectory traj = *be_traj_;
    be_traj_mutex_.unlock();

    // t_cur: time elapsed since start of trajectory
    double t_cur = (time_now).toSec() - traj.getGlobalStartTime();

    Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), jer(Eigen::Vector3d::Zero());
    Eigen::Quaterniond quat{0,0,0,1};
    std::pair<double, double> yaw_yawdot(0, 0);

    // IF time elapsed is below duration of trajectory, then continue to send command
    if (t_cur >= 0.0 && t_cur < traj.getTotalDuration())
    {
      pos = traj.getPos(t_cur);
      vel = traj.getVel(t_cur);
      acc = traj.getAcc(t_cur);
      jer = traj.getJer(t_cur);

      /*** calculate yaw ***/
      yaw_yawdot = calculate_yaw(traj, t_cur, pos, (time_now - time_since_last_samp_).toSec());

      quat = RPYToQuaternion(0.0, 0.0, yaw_yawdot.first);

      time_since_last_samp_ = time_now;

      // Send sampled point to Trajectory Server
      forwardExecTrajectory(pos, quat, vel, acc, jer, type_mask_);
    }
    // IF time elapsed is longer then duration of trajectory, then nothing is done
    else if (t_cur >= traj.getTotalDuration()) // Finished trajectory
    {
      return;
    }
    else { // Time is negative
      return;
    }

  }

  /**
   * @brief 
   * 
   * @param t_cur 
   * @param pos current position
   * @param dt 
   * @return std::pair<double, double> 
   */
  std::pair<double, double> calculate_yaw(const poly_traj::Trajectory& traj, const double& t_cur, const Eigen::Vector3d& pos, const double& dt)
  {
    std::pair<double, double> yaw_yawdot(0, 0);

    // get direction vector
    Eigen::Vector3d dir = t_cur + time_forward_ <= traj.getTotalDuration()
                              ? traj.getPos(t_cur + time_forward_) - pos
                              : traj.getPos(traj.getTotalDuration()) - pos;
    
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

    if (fabs(last_mission_yaw_dot_ + dt * YDDM) <= fabs(YDM)) // Within yaw_dot limits
    {
      // yawdot = last_mission_yaw_dot_ + dt * YDDM;
      d_yaw_max = (last_mission_yaw_dot_ * dt) + (0.5 * YDDM * dt * dt);
    }
    else // exceed yaw_dot limits
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

  Eigen::Quaterniond RPYToQuaternion(const double& roll, const double& pitch, const double& yaw){
    Eigen::Quaterniond q;
    q = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
    
    return q;
  }

private:

  // Plan trajectory data
  boost::shared_ptr<poly_traj::Trajectory> be_traj_; // back end trajectory
  ros::Time time_since_last_samp_; // Time since last sampling
  double time_forward_{1.0}; //Timestep forward

  double last_mission_yaw_{0.0};
  double last_mission_yaw_dot_{0.0};

  u_int16_t type_mask_{2048}; // Default to ignore yaw rate

  // Params
  const double YAW_DOT_MAX_PER_SEC{ 2 * M_PI};
  const double YAW_DOT_DOT_MAX_PER_SEC{ 5 * M_PI};

  std::mutex be_traj_mutex_; // Mutex lock for back end trajectory

// protected:
}; // class EgoPlannerAdaptor

#endif // _EGO_PLANNER_ADAPTOR_H_