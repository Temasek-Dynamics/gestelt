/****************************************************************************
 *
 *   Copyright (c) 2018-2021 Jaeyoung Lim. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @brief Shape Trajectory Library
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#include "trajectory_publisher/shapetrajectory.h"

shapetrajectory::shapetrajectory(int type, double acc_dec_time, double stop_time, double max_angular_vel) : trajectory(), N(0), dt_(0.1), T_(10.0), type_(type) {
  traj_omega_ = 2.0;
  traj_axis_ << 0.0, 0.0, 1.0;
  traj_radial_ << 1.0, 0.0, 0.0;
  traj_origin_ << 0.0, 0.0, 1.0;
  last_theta_ = 0.0;
  acc_dec_time_=acc_dec_time;
  stop_time_=stop_time;
  max_angular_vel_=max_angular_vel;
  angular_acc_=max_angular_vel_/acc_dec_time_;
};

shapetrajectory::~shapetrajectory(){

};

void shapetrajectory::initPrimitives(Eigen::Vector3d pos, Eigen::Vector3d axis, double omega) {
  // Generate primitives based on current state for smooth trajectory
  traj_origin_ = pos;
  traj_omega_ = omega;
  T_ = 2 * 3.14 / traj_omega_;
  traj_axis_ = axis;
  traj_radial_ << 0.0, -1.5, 0.0;
}

void shapetrajectory::generatePrimitives(Eigen::Vector3d pos) {}

void shapetrajectory::generatePrimitives(Eigen::Vector3d pos, Eigen::Vector3d vel) {}

void shapetrajectory::generatePrimitives(Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector3d jerk) {}

void shapetrajectory::generatePrimitives(Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector3d acc,
                                         Eigen::Vector3d jerk) {}

Eigen::Vector3d shapetrajectory::getPosition(double time) {
  Eigen::Vector3d position;
  double theta;

  switch (type_) {
    case TRAJ_ZERO:

      position << 0.0, 0.0, 0.0;
      break;

    case TRAJ_CIRCLE:
      
      // reset theta if time is reset
      if (time < 0.1) {
        last_theta_=0.0;
        // std::cout<<"theta reset!!!: "<<time<<std::endl;

      }
      
      // gently increase the velocity
      // time to decrease the velocity
      decrease_time_=stop_time_-acc_dec_time_;  
      

      //accelerate stage
      if (time<=acc_dec_time_ && time>=0)
      {
       varying_omega_=angular_acc_*time;
       
      }

      // constant velocity stage
      else if (time>acc_dec_time_ && time<=decrease_time_)
      {
        varying_omega_=max_angular_vel_;
      }
      // decelerate stage
      else if (time>decrease_time_ && time <=stop_time_)
      {
       varying_omega_=max_angular_vel_-angular_acc_*(time-decrease_time_);
      }

      // stop stage
      else if(time>stop_time_)
      {
        varying_omega_=0.0;
      }
      
      dt_=0.01;
      last_theta_ += varying_omega_ * dt_;

      // last_theta_ += traj_omega_ * dt_;
      theta = last_theta_;
      
      // std::cout<<"time: "<<time<<std::endl;
      // std::cout<<"traj_omega_: "<<traj_omega_<<std::endl;
      // std::cout<<"varying_omega_: "<<varying_omega_<<std::endl;
      // std::cout<<"theta: "<<theta<<std::endl;


      position = std::cos(theta) * traj_radial_ + std::sin(theta) * traj_axis_.cross(traj_radial_) +
                 (1 - std::cos(theta)) * traj_axis_.dot(traj_radial_) * traj_axis_ + traj_origin_;
      break;

    case TRAJ_LAMNISCATE:  // Lemniscate of Genero

      theta = traj_omega_ * time;
      position = std::cos(theta) * traj_radial_ + std::sin(theta) * std::cos(theta) * traj_axis_.cross(traj_radial_) +
                 (1 - std::cos(theta)) * traj_axis_.dot(traj_radial_) * traj_axis_ + traj_origin_;
      break;
    case TRAJ_STATIONARY:  // Lemniscate of Genero

      position = traj_origin_;
      break;
  }
  return position;
}

Eigen::Vector3d shapetrajectory::getVelocity(double time) {
  Eigen::Vector3d velocity;
  double theta;

  switch (type_) {
    case TRAJ_CIRCLE:
    // gently increase the velocity
      // time to decrease the velocity
      decrease_time_=stop_time_-acc_dec_time_;  
      

      //accelerate stage
      if (time<=acc_dec_time_ && time>=0)
      {
       varying_omega_=angular_acc_*time;
       
      }

      // constant velocity stage
      else if (time>acc_dec_time_ && time<=decrease_time_)
      {
        varying_omega_=max_angular_vel_;
      }
      // decelerate stage
      else if (time>decrease_time_ && time <=stop_time_)
      {
       varying_omega_=max_angular_vel_-angular_acc_*(time-decrease_time_);
      }

      // stop stage
      else if(time>stop_time_)
      {
        varying_omega_=0.0;
      }

      velocity = varying_omega_ * traj_axis_.cross((getPosition(time)-traj_origin_));
      break;
    case TRAJ_STATIONARY:

      velocity << 0.0, 0.0, 0.0;
      break;

    case TRAJ_LAMNISCATE:  // Lemniscate of Genero

     theta = traj_omega_ * time;
      velocity = traj_omega_ *
                 (-std::sin(theta) * traj_radial_ +
                  (std::pow(std::cos(theta), 2) - std::pow(std::sin(theta), 2)) * traj_axis_.cross(traj_radial_) +
                  (std::sin(theta)) * traj_axis_.dot(traj_radial_) * traj_axis_);
      break;

    default:
      velocity << 0.0, 0.0, 0.0;
      break;
  }
  return velocity;
}

Eigen::Vector3d shapetrajectory::getAcceleration(double time) {
  Eigen::Vector3d acceleration;
  double curr_ang_acc;
  switch (type_) {
    case TRAJ_CIRCLE:
    // gently increase the velocity
      // time to decrease the velocity
      decrease_time_=stop_time_-acc_dec_time_;  
      

      //accelerate stage
      if (time<=acc_dec_time_ && time>=0)
      {
       varying_omega_=angular_acc_*time;
       curr_ang_acc=angular_acc_;
      }

      // constant velocity stage
      else if (time>acc_dec_time_ && time<=decrease_time_)
      {
        varying_omega_=max_angular_vel_;
        curr_ang_acc=0.0;
      }
      // decelerate stage
      else if (time>decrease_time_ && time <=stop_time_)
      {
       varying_omega_=max_angular_vel_-angular_acc_*(time-decrease_time_);
       curr_ang_acc=-angular_acc_;
      }

      // stop stage
      else if(time>stop_time_)
      {
        varying_omega_=0.0;
        curr_ang_acc=0.0;
      }

      acceleration = curr_ang_acc * traj_axis_.cross((getPosition(time)-traj_origin_)) + varying_omega_* traj_axis_.cross(getVelocity(time));
      break;
    case TRAJ_STATIONARY:

      acceleration << 0.0, 0.0, 0.0;
      break;
    default:
      acceleration << 0.0, 0.0, 0.0;
      break;
  }
  return acceleration;
}

nav_msgs::Path shapetrajectory::getSegment() {
  Eigen::Vector3d targetPosition;
  Eigen::Vector4d targetOrientation;
  nav_msgs::Path segment;

  targetOrientation << 1.0, 0.0, 0.0, 0.0;
  geometry_msgs::PoseStamped targetPoseStamped;

  for (double t = 0; t < this->getDuration(); t += this->getsamplingTime()) {
    targetPosition = this->getPosition(t);
    targetPoseStamped = vector3d2PoseStampedMsg(targetPosition, targetOrientation);
    segment.poses.push_back(targetPoseStamped);
  }
  return segment;
}

geometry_msgs::PoseStamped shapetrajectory::vector3d2PoseStampedMsg(Eigen::Vector3d position,
                                                                    Eigen::Vector4d orientation) {
  geometry_msgs::PoseStamped encode_msg;
  encode_msg.header.stamp = ros::Time::now();
  encode_msg.header.frame_id = "map";
  encode_msg.pose.orientation.w = orientation(0);
  encode_msg.pose.orientation.x = orientation(1);
  encode_msg.pose.orientation.y = orientation(2);
  encode_msg.pose.orientation.z = orientation(3);
  encode_msg.pose.position.x = position(0);
  encode_msg.pose.position.y = position(1);
  encode_msg.pose.position.z = position(2);
  return encode_msg;
}
