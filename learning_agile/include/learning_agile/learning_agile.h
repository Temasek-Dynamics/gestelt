#ifndef _LEARNING_AGILE_H_
#define _LEARNING_AGILE_H_

#include <numeric>
#include <deque>

#include <Eigen/Geometry> 

#include <ros/ros.h>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TwistStamped.h>


#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>


#include <std_msgs/Empty.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>

#include <gestelt_msgs/Goals.h>

// acados
#include "acados/utils/math.h"
#include "acados_c/ocp_nlp_interface.h"

#include "acados_sim_solver_ACADOS_model.h"
#include "acados_solver_ACADOS_model.h"
#include <casadi/casadi.hpp>
#endif 
class LearningAgile{
    public:
        LearningAgent(ros::NodeHandle& nh);
        
        void mission_start_cb(const gestelt_msgs::GoalsPtr &msg);
        void gate_state_estimation_cb(const ros::TimerEvent &e);
        void setpoint_timer_cb(const ros::TimerEvent &e);
        void drone_state_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void drone_state_twist_cb(const geometry_msgs::TwistStamped::ConstPtr& msg);


        //------------------acados solver-------------------
        void solver_loading();
        void solver_request();

    private:
    ros::Subscriber drone_pose_sub_;
    ros::Subscriber drone_twist_sub_;
    ros::Subscriber waypoint_cb_;

    ros::Publisher next_attitude_setpoint_pub_;
    ros::Publisher gate_centroid_pub_;
    ros::Publisher traverse_time_pub_;

    ros::Publisher mpc_runtime_pub_;
    ros::Publisher current_pred_traj_pub_;

    //environment setting
    Eigen::Vector3d des_trav_point_={0,0,0};
    Eigen::Vector4d des_trav_quat_={0,0,0,0};
    double t_tra_=0.1;

    Eigen::Vector3d start_point_={0,0,0};
    Eigen::Vector3d end_point_={0,0,0};
    Eigen::Vector4d des_goal_quat_={0,0,0,0};
    Eigen::Vector3d des_goal_vel_={0,0,0};
    Eigen::VectorXd des_goal_state_= Eigen::VectorXd::Zero(10);


    //current drone state
    Eigen::VectorXd drone_state_= Eigen::VectorXd::Zero(10);
    Eigen::Vector3d drone_pos_= {0,0,0};
    Eigen::Vector3d drone_vel_= {0,0,0};
    Eigen::Vector4d drone_quat_= {1,0,0,0};
    Eigen::Vector3d drone_ang_vel_= {0,0,0};

    //MPC parameters
    double max_tra_w_=0;
    Eigen::Vector4d current_input_={0,0,0,0};
    Eigen::Vector4d last_input_={0,0,0,0};

    //MPC output
    Eigen::MatrixXd state_traj_opt_;
    Eigen::MatrixXd control_traj_opt_;
    bool NO_SOLUTION_FLAG_=false;
    
    //------------------- acados solver -------------------
    int n_nodes_;
    int n_x_;
    int n_u_;
    int dt_=0.1;

    

};