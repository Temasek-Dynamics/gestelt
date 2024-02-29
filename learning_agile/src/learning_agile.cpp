#include <learning_agile/learning_agile.h>

void LearningAgile::init(ros::NodeHandle& nh)
{   
    /////////////////
    /* Subscribers */
    /////////////////
    drone_pose_sub_= nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, &LearningAgent::drone_state_pose_cb, this);
    drone_twist_sub_= nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local", 1, &LearningAgent::drone_state_twist_cb, this);
    waypoint_cb_ = nh.subscribe<gestelt_msgs::Goals>("planner/goals_learning_agile", 1, &LearningAgent::mission_start_cb, this);


    /////////////////
    /* Publishers */
    /////////////////
    next_attitude_setpoint_pub_ = nh.advertise<mavros_msgs::AttitudeTarget>("/learning_agile_agent/soft_RT_mpc_attitude", 1);
    gate_centroid_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/learning_agile_agent/gate_centroid", 1);

    traverse_time_pub_ = nh.advertise<std_msgs::Int8>("/learning_agile_agent/traverse_time", 10);
    mpc_runtime_pub_ = nh.advertise<std_msgs::Int8>("/learning_agile_agent/callback_runtime", 10);
    current_pred_traj_pub_ = nh.advertise<geometry_msgs::PoseArray>("/learning_agile_agent/current_pred_traj", 10);

    /////////////////
    /* Timers */
    /////////////////
    pub_freq=100;
    soft_RT_mpc_timer_ = nh.createTimer(ros::Duration(1/pub_freq), &LearningAgent::setpoint_timer_cb, this);
    

    IGNORE_ATTITUDE = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
}   

void LearningAgile::solver_loading()
{
    // Load the solver from the python interface generated code
    nlp_solver_capsule *acados_ocp_capsule = ACADOS_model_acados_create_capsule();


    status=ACADOS_model_acados_create(acados_ocp_capsule);

    if (status)
    {
        printf("ACADOS_model_acados_create() returned status %d. Exiting.\n", status);
        exit(1);
    }

    ocp_nlp_config *nlp_config = ACADOS_model_acados_get_nlp_config(acados_ocp_capsule);
    ocp_nlp_dims *nlp_dims = ACADOS_model_acados_get_nlp_dims(acados_ocp_capsule);
    ocp_nlp_in *nlp_in = ACADOS_model_acados_get_nlp_in(acados_ocp_capsule);
    ocp_nlp_out *nlp_out = ACADOS_model_acados_get_nlp_out(acados_ocp_capsule);

    // solver settings
    n_nodes_ = nlp_dims->N;
    n_x_ = *nlp_dims->nx;
    n_u_ = *nlp_dims->nu;
    printf("time horizion is %d, with state %d and input %d \n", n_nodes_, n_x_, n_u_);


}

void LearingAgile::solver_request(){
    des_goal_state_ << goal_point_, goal_vel_, goal_quat_;

    for (int i = 0; i < n_nodes_; i++)
    {
        current_input_=last_input_;
        varying_trav_weight=max_tra_w_*casadi::exp(-10*(dt_*i-t_tra_)**2)

        // set the external parameters for the solver
        // desired goal state, current input, desired traverse pose, varying traverse weight
        Eigen::VectorXd solver_extern_param = des_goal_state_<< current_input_, des_trav_point_, des_trav_quat_, varying_trav_weight;
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "p",solver_extern_param);
    }
    //TODO
    // set the initial GUESS

    // set the end desired state
    double end_weight=0;
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, n_nodes_, "p",solver_extern_param);

    //set initial condition aligned with the current state
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx",drone_state_);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx",drone_state_);


    // solve the problem
    status = ACADOS_model_acados_solve(acados_ocp_capsule);
    if (status != 0):
        NO_SOLUTION_FLAG=true;

    // get the solution
    for (int i = 0; i < n_nodes_; i++)
    {
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, i, "x", state_traj_opt_);
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, i, "u", control_traj_opt_);
    }
    // get the last state
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, n_nodes_, "x", state_traj_opt_);
}

void LearningAgile:: setpoint_timer_cb(const ros::TimerEvent &e);
{
    if (NO_SOLUTION_FLAG_)
    {
        ROS_WARN("No solution found for the current MPC problem");
        return;
    }
    std::lock_guard<std::mutex> cmd_guard(cmd_mutex_);
    mavros_msgs::AttitudeTarget mpc_cmd;
    mpc_cmd.header.stamp = ros::Time::now();
    mpc_cmd.header.frame_id = origin_frame_;
    mpc_cmd.type_mask = IGNORE_ATTITUDE; // Ignore orientation
    mpc_cmd.thrust = control_traj_opt_(0,0);
    mpc_cmd.body_rate.x = control_traj_opt_(0,1);
    mpc_cmd.body_rate.y = control_traj_opt_(0,2);
    mpc_cmd.body_rate.z = control_traj_opt_(0,3);
    next_attitude_setpoint_pub_.publish(mpc_cmd);
        
}

void LearningAgile::drone_state_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    drone_pos_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    drone_quat_ << msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z;
    drone_state_.head(3) = drone_pos_;
    drone_state_.segment(6,4) = drone_quat_;
}

void LearingAgile::drone_state_twist_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    drone_vel_ << msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z;
    drone_ang_vel_ << msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z;
    drone_state_.segment(3,3) = drone_vel_;
    drone_state_.tail(3) = drone_ang_vel_;
}

void LearningAgile::mission_start_cb(const gestelt_msgs::GoalsPtr &msg)
{
    des_trav_point_ << msg->gate_point.x, msg->gate_point.y, msg->gate_point.z;
    des_trav_quat_ << msg->gate_quat.w, msg->gate_quat.x, msg->gate_quat.y, msg->gate_quat.z;

    start_point_ << msg->start_point.x, msg->start_point.y, msg->start_point.z;
    end_point_ << msg->end_point.x, msg->end_point.y, msg->end_point.z;
}