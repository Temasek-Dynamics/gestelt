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

# include <gestelt_msgs/Goals.h>
#include <Python.h>

#endif 
class LearningAgile{
    public:
        LearningAgent(ros::NodeHandle& nh);
        void mission_start_cb(const gestelt_msgs::GoalsPtr &msg);
        void gate_state_estimation_cb(const ros::TimerEvent &e);
        void setpoint_timer_cb(const ros::TimerEvent &e);
        void drone_state_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void drone_state_twist_cb(const geometry_msgs::TwistStamped::ConstPtr& msg);


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
    Eigen::Vector3d gate_point_={0,0,0};
    Eigen::Vector3d start_point_={0,0,0};
    Eigen::Vector3d end_point_={0,0,0};

    //current drone state
    Eigen::VectorXd drone_state_= Eigen::VectorXd::Zero(10);
    Eigen::Vector3d drone_pos_= {0,0,0};
    Eigen::Vector3d drone_vel_= {0,0,0};
    Eigen::Vector3d drone_quat_= {1,0,0,0};
    Eigen::Vector3d drone_ang_vel_= {0,0,0};


    // learning agile agent python interface
    Py_Initialize();

    PyObject *sys = PyImport_ImportModule("sys");
    PyObject *path = PyObject_GetAttrString(sys, "path");
    PyList_Append(path, PyUnicode_FromString("~/gestelt_ws/src/gestelt/gestelt_bringup/src/LearningAgile"));

    //load the python module
    PyObject *pModule = PyImport_ImportModule("learning_agile_agent");

    //load the class
    PyObject *pClass = PyObject_GetAttrString(pModule, "LearningAgileAgent");

    //create an instance of the class
    PyObject *learing_agile_agent_ = PyObject_CallObject(pClass, NULL);

};