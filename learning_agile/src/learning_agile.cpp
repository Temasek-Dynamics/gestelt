#include <learning_agile/learning_agile.h>

void LearningAgile::init(ros::NodeHandle& nh)
{   
    /////////////////
    /* Subscribers */
    /////////////////
    drone_pose_sub_= nh.subscribe("/mavros/local_position/pose", 1, &LearningAgile::drone_state_pose_cb, this);
    drone_twist_sub_= nh.subscribe("/mavros/local_position/velocity_local", 1, &LearningAgile::drone_state_twist_cb, this);
    waypoint_sub_ = nh.subscribe("/planner/goals_learning_agile", 1, &LearningAgile::mission_start_cb, this);


    /////////////////
    /* Publishers */
    /////////////////
    next_attitude_setpoint_pub_ = nh.advertise<mavros_msgs::AttitudeTarget>("/learning_agile_agent/soft_RT_mpc_attitude", 1);
    // gate_centroid_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/learning_agile_agent/gate_centroid", 1);

    // traverse_time_pub_ = nh.advertise<std_msgs::Float32>>("/learning_agile_agent/traverse_time", 10);
    mpc_runtime_pub_ = nh.advertise<std_msgs::Float64>("/learning_agile_agent/callback_runtime", 10);
    current_pred_traj_pub_ = nh.advertise<geometry_msgs::PoseArray>("/learning_agile_agent/current_pred_traj", 10);

    /////////////////
    /* Timers */
    /////////////////
    double pub_freq=100;
    soft_RT_mpc_timer_ = nh.createTimer(ros::Duration(1/pub_freq), &LearningAgile::setpoint_timer_cb, this);
    
    solver_loading();
    
}   

void LearningAgile::solver_loading()
{
    // Load the solver from the python interface generated code
    acados_ocp_capsule = ACADOS_model_acados_create_capsule();


    status=ACADOS_model_acados_create(acados_ocp_capsule);

    if (status)
    {
        ROS_INFO("ACADOS_model_acados_create() returned status %d. Exiting.\n", status);
        exit(1);
    }

    nlp_config = ACADOS_model_acados_get_nlp_config(acados_ocp_capsule);
    nlp_dims = ACADOS_model_acados_get_nlp_dims(acados_ocp_capsule);
    nlp_in = ACADOS_model_acados_get_nlp_in(acados_ocp_capsule);
    nlp_out = ACADOS_model_acados_get_nlp_out(acados_ocp_capsule);

    // solver settings
    n_nodes_ = nlp_dims->N;
    n_x_ = *nlp_dims->nx;
    n_u_ = *nlp_dims->nu;
    ROS_INFO("time horizion is %d, with state %d and input %d \n", n_nodes_, n_x_, n_u_);
    state_traj_opt_=Eigen::MatrixXd::Zero(n_nodes_,4);
    control_traj_opt_=Eigen::MatrixXd::Zero(n_nodes_,4);

}

void LearningAgile::solver_request(){
    
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, n_nodes_,"x", des_goal_state_.data());

    for (int i = 0; i < n_nodes_; i++)
    {
        current_input_=last_input_;
        double varying_trav_weight = max_tra_w_ * std::exp(-10 * std::pow(dt_ * i - t_tra_, 2));

        // set the external parameters for the solver
        // desired goal state, current input, desired traverse pose, varying traverse weight
        Eigen::VectorXd solver_extern_param(22);
        solver_extern_param.segment(0,10) = des_goal_state_;
        solver_extern_param.segment(10,4) = current_input_;
        solver_extern_param.segment(14,3) = des_trav_point_;
        solver_extern_param.segment(17,4) = des_trav_quat_;
        solver_extern_param(21) = varying_trav_weight;

        
        double *solver_extern_param_ptr = solver_extern_param.data();
      
        int NP=22;
        ACADOS_model_acados_update_params(acados_ocp_capsule, i,solver_extern_param_ptr,NP);
    }
    //TODO
    // set the initial GUESS

    // set the end desired state
    Eigen::VectorXd solver_extern_param(22);
    double end_weight=0;
    solver_extern_param.segment(0,10) = des_goal_state_;
    solver_extern_param.segment(10,4) = current_input_;
    solver_extern_param.segment(14,3) = des_trav_point_;
    solver_extern_param.segment(17,4) = des_trav_quat_;
    solver_extern_param(21) = end_weight;

    
    double *solver_extern_param_ptr = solver_extern_param.data();
   
    int NP=22;
    ACADOS_model_acados_update_params(acados_ocp_capsule, n_nodes_, solver_extern_param_ptr, NP);
    //set initial condition aligned with the current state
    double *drone_state_ptr = drone_state_.data();
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx",drone_state_ptr);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx",drone_state_ptr);


    // solve the problem
    status = ACADOS_model_acados_solve(acados_ocp_capsule);
    if (status != 0){
        NO_SOLUTION_FLAG_=true;
        ROS_INFO("acados no solution");
    }
    else{
        // get the solution
        // for (int i = 0; i < n_nodes_; ++i)
        // {   
        //     double state_i_opt[n_x_];
        //     double control_i_opt[n_u_];
            
        //     ROS_INFO("getting the solution");
        //     ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, i, "x", &state_i_opt);
        //     ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, i, "u", &control_i_opt);


        //     // ROS_INFO("control %d is %f, %f, %f", i, control_i_opt_(0), control_i_opt_(1), control_i_opt_(2));
        //     // state_traj_opt_.row(i) = state_i_opt_;
        //     // control_traj_opt_.row(i) = control_i_opt_;
        // }


        // get the last state
        // ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, n_nodes_, "x", &state_i_opt_);
        // state_traj_opt_.row(n_nodes_) = state_i_opt_;


        // double state_i_opt[n_x_];
        
        
       
        // ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "x", &state_i_opt);
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "u", &control_opt_);
       
        // ROS_INFO("HAVE THE SOLUTION");

    }
}

void LearningAgile::setpoint_timer_cb(const ros::TimerEvent &e)
{   

    std::lock_guard<std::mutex> cmd_guard(cmd_mutex_);
    auto start = std::chrono::high_resolution_clock::now();

    if (start_soft_RT_mpc_timer_==true)
    {
        if (NO_SOLUTION_FLAG_)
        {
            // ROS_WARN("No solution found for the current MPC problem");
            return;
        }
        else
        {
        solver_request();
        mavros_msgs::AttitudeTarget mpc_cmd;
        mpc_cmd.header.stamp = ros::Time::now();
        mpc_cmd.header.frame_id = origin_frame_;
        mpc_cmd.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE; // Ignore orientation
        mpc_cmd.thrust = control_opt_[0]/((0.8706)*4);
        mpc_cmd.body_rate.x = control_opt_[1];
        mpc_cmd.body_rate.y = control_opt_[2];
        mpc_cmd.body_rate.z = control_opt_[3];
        next_attitude_setpoint_pub_.publish(mpc_cmd);

        auto end = std::chrono::high_resolution_clock::now();

        double preloop_dur = std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count();
        
        std_msgs::Float64 mpc_runtime;
        mpc_runtime.data = preloop_dur;
        mpc_runtime_pub_.publish(mpc_runtime);
        }  
    }  
}

void LearningAgile::drone_state_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    drone_pos_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    drone_quat_ << msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z;

    // drone state P, V, Q
    drone_state_.segment(0,3) = drone_pos_;
    drone_state_.segment(6,4) = drone_quat_;
}

void LearningAgile::drone_state_twist_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    drone_vel_ << msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z;
    drone_ang_vel_ << msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z;
    drone_state_.segment(3,3) = drone_vel_;
    // drone_state_.tail(3) = drone_ang_vel_;
}

void LearningAgile::mission_start_cb(const gestelt_msgs::GoalsPtr &msg)
{
    des_trav_point_ << msg->waypoints[0].position.x, msg->waypoints[0].position.y, msg->waypoints[0].position.z;
    des_trav_quat_ << msg->waypoints[0].orientation.w, msg->waypoints[0].orientation.x, msg->waypoints[0].orientation.y, msg->waypoints[0].orientation.z;
    // des_trav_quat_=drone_quat_;
    des_goal_point_ << msg->waypoints[1].position.x, msg->waypoints[1].position.y, msg->waypoints[1].position.z;
    des_goal_quat_ << msg->waypoints[1].orientation.w, msg->waypoints[1].orientation.x, msg->waypoints[1].orientation.y, msg->waypoints[1].orientation.z;
    // des_goal_quat_=drone_quat_;
    ROS_INFO("des_trav_point_ is %f, %f, %f", des_trav_point_(0), des_trav_point_(1), des_trav_point_(2));
    ROS_INFO("des_trav_quat_ is %f, %f, %f, %f", des_trav_quat_(0), des_trav_quat_(1), des_trav_quat_(2), des_trav_quat_(3));

    ROS_INFO("des_goal_point_ is %f, %f, %f", des_goal_point_(0), des_goal_point_(1), des_goal_point_(2));
    ROS_INFO("des_goal_quat_ is %f, %f, %f, %f", des_goal_quat_(0), des_goal_quat_(1), des_goal_quat_(2), des_goal_quat_(3));
    //set the goal state
    des_goal_state_.segment(0,3) = des_goal_point_;
    des_goal_state_.segment(3,3) = des_goal_vel_;
    des_goal_state_.segment(6,4) = des_goal_quat_;

    start_soft_RT_mpc_timer_=true;
    
}