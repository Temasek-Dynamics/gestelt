#include <mpc_ros_wrapper/mpc_ros_wrapper.h>

void mpcRosWrapper::init(ros::NodeHandle& nh)
{   
    /////////////////
    /*parameters*/
    /////////////////
    nh.param("learning_agile/max_traverse_weight", max_tra_w_, 0.0);
    nh.param("learning_agile/traverse_weight_span", tra_w_span_, 0.0);
    nh.param("learning_agile/traverse_time", t_tra_abs_, 10.0);
    nh.param("learning_agile/no_solution_flag_t_thresh", no_solution_flag_t_thresh_, 0.02);
    nh.param("learning_agile/single_motor_max_thrust", single_motor_max_thrust_, 2.1334185);
    nh.param("learning_agile/pred_traj_vis", PRED_TRAJ_VIS_FLAG_, false);
    
    /////////////////
    /* Subscribers */
    /////////////////
    drone_pose_sub_= nh.subscribe("/mavros/local_position/pose", 1, &mpcRosWrapper::drone_state_pose_cb, this);
    drone_twist_sub_= nh.subscribe("/mavros/local_position/velocity_body", 1, &mpcRosWrapper::drone_state_twist_cb, this);
    waypoint_sub_ = nh.subscribe("/planner/goals_learning_agile", 1, &mpcRosWrapper::mission_start_cb, this);


    /////////////////
    /* Publishers */
    /////////////////
    next_attitude_setpoint_pub_ = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 1);
    raw_solver_output_pub_ = nh.advertise<mavros_msgs::AttitudeTarget>("/learning_agile_agent/raw_solver_output", 1);
    
    // gate_centroid_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/learning_agile_agent/gate_centroid", 1);

    // traverse_time_pub_ = nh.advertise<std_msgs::Float32>>("/learning_agile_agent/traverse_time", 10);
    mpc_runtime_pub_ = nh.advertise<std_msgs::Float64>("/learning_agile_agent/callback_runtime", 10);
    current_pred_traj_pub_ = nh.advertise<geometry_msgs::PoseArray>("/learning_agile_agent/current_pred_traj", 10);
    

    // solver loading 
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
    

    state_traj_opt_=new double[n_nodes_*n_x_];
    

}

void mpcRosWrapper::solver_request(){
    
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, n_nodes_,"x", des_goal_state_.data());
    
    
    auto current_time = std::chrono::high_resolution_clock::now();
    double request_gap = std::chrono::duration_cast<std::chrono::duration<double>>(current_time - last_request_time_).count();
    // ROS_INFO("request gap is %f", request_gap);
    
    // if two requests gap is too long, emergency stop
    if (MISSION_LOADED_FLAG_ && request_gap > no_solution_flag_t_thresh_)
    {
        NO_SOLUTION_FLAG_=true;
        ROS_INFO("the request period is too long, emergency stop");
    }
    else
    {
        //t_tra: time to the traverse point relative to the current time
        //t_tra_abs_: absolute time to the traverse point, w.r.t the mission start time
        double mission_t_progress= std::chrono::duration_cast<std::chrono::duration<double>>(current_time - mission_start_time_).count();
        double t_tra=t_tra_abs_-mission_t_progress;   
        // ROS_INFO("t_tra is %f", t_tra);
  
        for (int i = 0; i < n_nodes_; i++)
        {
            current_input_=last_input_;
            double varying_trav_weight = max_tra_w_ * std::exp(-tra_w_span_ * std::pow(dt_ * i - t_tra, 2));

            // set the external parameters for the solver
            // desired goal state, current input, desired traverse pose, varying traverse weight
            Eigen::VectorXd solver_extern_param(22);
            solver_extern_param.segment(0,10) = des_goal_state_;
            solver_extern_param.segment(10,4) = current_input_;
            solver_extern_param.segment(14,3) = des_trav_point_;
            solver_extern_param.segment(17,4) = des_trav_quat_;
            solver_extern_param(21) = varying_trav_weight;

            int NP=22;
            double *solver_extern_param_ptr = solver_extern_param.data();
        
            ACADOS_model_acados_update_params(acados_ocp_capsule, i,solver_extern_param_ptr,NP);
        }
        //TODO
        // set the initial GUESS
        // ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, n_nodes_ , "x", &state_traj_opt_[n_nodes_*n_x_]);

        // set the end desired state
        Eigen::VectorXd solver_extern_param(22);
        double end_tra_weight=0;
        solver_extern_param.segment(0,10) = des_goal_state_;
        solver_extern_param.segment(10,4) = current_input_;
        solver_extern_param.segment(14,3) = des_trav_point_;
        solver_extern_param.segment(17,4) = des_trav_quat_;
        solver_extern_param(21) = end_tra_weight;

        
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
        else
        {
            // // get the state solution for visualization

            if (PRED_TRAJ_VIS_FLAG_){
                
                
                
                for (int i = 0; i < n_nodes_; i++)
                {   
                    
                ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, i, "x", &state_traj_opt_[i*n_x_]);
                    
                    // ROS_INFO("state_i_opt_ is %f, %f, %f", state_i_opt_[0], state_i_opt_[1], state_i_opt_[2]);
                    // for (int j = 0; j < n_x_; ++j)
                    // {
                    //     state_traj_opt_.push_back(state_i_opt_);
                    // }
        
                }


                // get the last state
                ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, n_nodes_, "x",  &state_traj_opt_[n_nodes_*n_x_]);
                
            pred_traj_vis();

                
            }
            
        
            // get the control input
            ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "u", &control_opt_);
        
            // ROS_INFO("HAVE THE SOLUTION");

        }
    }
    last_request_time_=current_time;
}


void mpcRosWrapper::drone_state_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    drone_pos_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    drone_quat_ << msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z;

    // drone state P, V, Q
    drone_state_.segment(0,3) = drone_pos_;
    drone_state_.segment(6,4) = drone_quat_;
}

void mpcRosWrapper::drone_state_twist_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    drone_vel_ << msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z;
    drone_ang_vel_ << msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z;
    drone_state_.segment(3,3) = drone_vel_;
    // drone_state_.tail(3) = drone_ang_vel_;
}

void mpcRosWrapper::mission_start_cb(const gestelt_msgs::GoalsPtr &msg)
{
    des_trav_point_ << msg->waypoints[0].position.x, msg->waypoints[0].position.y, msg->waypoints[0].position.z;
    des_trav_quat_ << msg->waypoints[0].orientation.w, msg->waypoints[0].orientation.x, msg->waypoints[0].orientation.y, msg->waypoints[0].orientation.z;
    ROS_INFO("des_trav_quat_ is %f, %f, %f, %f", des_trav_quat_(0), des_trav_quat_(1), des_trav_quat_(2), des_trav_quat_(3));
    // des_trav_quat_=drone_quat_;
    des_goal_point_ << msg->waypoints[1].position.x, msg->waypoints[1].position.y, msg->waypoints[1].position.z;
    des_goal_quat_ << msg->waypoints[1].orientation.w, msg->waypoints[1].orientation.x, msg->waypoints[1].orientation.y, msg->waypoints[1].orientation.z;
    // des_goal_quat_=drone_quat_;
    // ROS_INFO("des_trav_point_ is %f, %f, %f", des_trav_point_(0), des_trav_point_(1), des_trav_point_(2));
    // ROS_INFO("des_trav_quat_ is %f, %f, %f, %f", des_trav_quat_(0), des_trav_quat_(1), des_trav_quat_(2), des_trav_quat_(3));

    // ROS_INFO("des_goal_point_ is %f, %f, %f", des_goal_point_(0), des_goal_point_(1), des_goal_point_(2));
    // ROS_INFO("des_goal_quat_ is %f, %f, %f, %f", des_goal_quat_(0), des_goal_quat_(1), des_goal_quat_(2), des_goal_quat_(3));
    //set the goal state
    des_goal_state_.segment(0,3) = des_goal_point_;
    des_goal_state_.segment(3,3) = des_goal_vel_;
    des_goal_state_.segment(6,4) = des_goal_quat_;

    MISSION_LOADED_FLAG_=true;
    mission_start_time_= std::chrono::high_resolution_clock::now();
    last_request_time_=mission_start_time_;
}


// This is the function where the traj_server called, by calling the Update
// inside the traj_server, there will be no individual learning_agile node
// input: current state, desired goal state, desired traverse point, desired traverse quaternion
// output: control input
void mpcRosWrapper::Update()
{   
    std::lock_guard<std::mutex> cmd_guard(cmd_mutex_);
    auto start = std::chrono::high_resolution_clock::now();
    

    if (MISSION_LOADED_FLAG_==true)
    {   
        solver_request();
        if (NO_SOLUTION_FLAG_)
        {   
            // traj server will send the current position as the setpoint
            ROS_WARN("No solution found for the current MPC problem,will send the current position as the setpoint");
        }
        else
        {
            mavros_msgs::AttitudeTarget mpc_cmd;
            mpc_cmd.header.stamp = ros::Time::now();
            mpc_cmd.header.frame_id = origin_frame_;
            mpc_cmd.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE; // Ignore orientation
            mpc_cmd.thrust = control_opt_[0]/(single_motor_max_thrust_*4);
            mpc_cmd.body_rate.x = control_opt_[1];
            mpc_cmd.body_rate.y = control_opt_[2];
            mpc_cmd.body_rate.z = control_opt_[3];
            next_attitude_setpoint_pub_.publish(mpc_cmd);

            auto end = std::chrono::high_resolution_clock::now();

            double preloop_dur = std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count();
            
            std_msgs::Float64 mpc_runtime;
            mpc_runtime.data = preloop_dur;
            mpc_runtime_pub_.publish(mpc_runtime);

            // raw solver output
            raw_solver_output_pub_.publish(mpc_cmd);
        }  
    }  
    else
    {
        ROS_WARN("No mission loaded, will send the hover setpoint");
        NO_SOLUTION_FLAG_=true;
    }
}

void mpcRosWrapper::pred_traj_vis()
{
    geometry_msgs::PoseArray pred_traj;
    pred_traj.header.stamp = ros::Time::now();
    pred_traj.header.frame_id = origin_frame_;
    for (int i = 0; i < n_nodes_; i++)
    {
        geometry_msgs::Pose pose;
        pose.position.x = state_traj_opt_[i*n_x_];
        pose.position.y = state_traj_opt_[i*n_x_+1];
        pose.position.z = state_traj_opt_[i*n_x_+2];
        pose.orientation.w = state_traj_opt_[i*n_x_+6];
        pose.orientation.x = state_traj_opt_[i*n_x_+7];
        pose.orientation.y = state_traj_opt_[i*n_x_+8];
        pose.orientation.z = state_traj_opt_[i*n_x_+9];
        pred_traj.poses.push_back(pose);
    }
    current_pred_traj_pub_.publish(pred_traj);
}
