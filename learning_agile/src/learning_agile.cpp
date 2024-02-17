#include <learning_agile/learning_agile.h>

void LearningAgile::init(ros::NodeHandle& nh)
{
    drone_pose_sub_= nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, &LearningAgent::drone_state_pose_cb, this);
    drone_twist_sub_= nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local", 10, &LearningAgent::drone_state_twist_cb, this);
    waypoint_cb_ = nh.subscribe<gestelt_msgs::Goals>("planner/goals_learning_agile", 10, &LearningAgent::mission_start_cb, this);

    next_attitude_setpoint_pub_ = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);
    gate_centroid_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/learning_agile_agent/gate_centroid", 10);

    traverse_time_pub_ = nh.advertise<std_msgs::Int8>("/learning_agile_agent/traverse_time", 10);
    mpc_runtime_pub_ = nh.advertise<std_msgs::Int8>("/learning_agile_agent/callback_runtime", 10);
    current_pred_traj_pub_ = nh.advertise<geometry_msgs::PoseArray>("/learning_agile_agent/current_pred_traj", 10);
    
}   

void LearningAgile::mission_start_cb(const gestelt_msgs::GoalsPtr &msg)
{
    start_point_=drone_pos_;

    gate_point_={msg->goal[0].x, msg->goal[0].y, msg->goal[0].z};
    end_point_={msg->goal[1].x, msg->goal[1].y, msg->goal[1].z};

    // 准备方法的参数
    PyObject *pArgs = PyTuple_Pack(3,
                                    Py_BuildValue("i", start_point_),
                                    Py_BuildValue("i", final_point_),
                                    Py_BuildValue("i", gate_point_));

    PyObject_CallMethod(learing_agile_agent_,"receive_terminal_states",
}