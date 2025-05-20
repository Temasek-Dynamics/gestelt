#include "model_inference/inference_node.h"
#include "model_inference/inference_engine.h"
#include "model_inference/torch_engine.h"

void InferenceNode::init(ros::NodeHandle& nh)
{
    std::string model_type, model_path,model_name, device,model_dir;
    bool is_simulation;
    nh.param("model_type", model_type, std::string("torch"));
    nh.param("model_dir",model_dir, std::string(""));
    nh.param("NN_deploy_model_name", model_name, std::string(""));
    nh.param("device",device, std::string("cpu"));
    nh.param("NN_freq", NN_freq_, 50.0);
    nh.param("vel_norm_factor", vel_norm_factor_, 5.0);
    nh.param("pos_norm_factor", pos_norm_factor_, 2.0);
    nh.param("is_simulation", is_simulation, false);
    nh.param("t_tra_abs", t_tra_abs_, 1.0);
    nh.param("mission_period", mission_period_, 5.0);
    model_path = model_dir + model_name;
    ROS_INFO("model path is %s", model_path.c_str());
    engine_ = std::make_unique<TorchEngine>(model_path, device);
    

    //subscribers
    drone_pose_sub_= nh.subscribe("/mavros/local_position/pose", 1, &InferenceNode::drone_state_pose_cb, this);
    drone_twist_sub_= nh.subscribe("/mavros/local_position/velocity_local", 1, &InferenceNode::drone_state_twist_cb, this);
    waypoint_sub_ = nh.subscribe("/planner/goals_learning_agile", 1, &InferenceNode::mission_start_cb, this);
    gate_points_sub_ = nh.subscribe("/visual/gate_points", 1, &InferenceNode::gate_points_cb, this);

    // publishers
    inference_timer_ = nh.createTimer(ros::Duration(1/NN_freq_), &InferenceNode::inference_cb, this);
    NN_output_pub_ = nh.advertise<gestelt_msgs::close_loop_NN_output>("/learning_agile_sim/NN_output", 10);

    // initialize variables
    drone_state_ = Eigen::VectorXd::Zero(10);
    drone_state_(6) = 1.0; // set the quaternion w to 1
    model_input_= Eigen::VectorXd::Zero(37);
    gate_points_ = Eigen::VectorXd::Zero(12);
    last_gate_points_ = Eigen::VectorXd::Zero(12);
    des_goal_state_ = Eigen::VectorXd::Zero(10);
    des_goal_state_(6) = 1.0; // set the quaternion w to 1
    i_ = 0;

    // obtain the static tf2 transform between the map and the world
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    
    if (is_simulation){
        while (nh.ok()){
            try{
                if (tfBuffer.canTransform("world", "map", ros::Time(0), ros::Duration(0.1))){
                    transformStamped_ = tfBuffer.lookupTransform("world", "map", ros::Time(0), ros::Duration(0.1));
                    break;
                }
                else{
                    ROS_WARN("Waiting for transform from world to map");
                    ros::Duration(0.1).sleep();
                }
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("%s", ex.what());
            }
        }
        ROS_INFO("transform from world to map is %f, %f, %f", transformStamped_.transform.translation.x, transformStamped_.transform.translation.y, transformStamped_.transform.translation.z);
    }
    else{
        transformStamped_.header.frame_id = "world";
        transformStamped_.child_frame_id = "map";
        transformStamped_.transform.translation.x = 0;
        transformStamped_.transform.translation.y = 0;
        transformStamped_.transform.translation.z = 0;
        transformStamped_.transform.rotation.w = 1;
        transformStamped_.transform.rotation.x = 0;
        transformStamped_.transform.rotation.y = 0;
        transformStamped_.transform.rotation.z = 0;
    }

}

void InferenceNode::get_obs(){
    model_input_.segment(0,10) = drone_state_;
    model_input_.segment(0,3) /= pos_norm_factor_;
    model_input_.segment(3,3) /= vel_norm_factor_;

    model_input_.segment(10,3) = des_goal_point_/pos_norm_factor_;
    model_input_.segment(13,12) = gate_points_/pos_norm_factor_;
    model_input_.segment(25,12) = last_gate_points_/pos_norm_factor_;

    last_gate_points_ = gate_points_;
}
void InferenceNode::inference_cb(const ros::TimerEvent& e)
{   

    if(MISSION_START_ && RECEIVED_DRONE_TWIST_ && RECEIVED_DRONE_POSE_ && i_<mission_period_*NN_freq_){
        get_obs();
        std_model_input_ = std::vector<float>(model_input_.data(), model_input_.data() + model_input_.size());
        auto res = engine_->predict(std_model_input_);

        gestelt_msgs::close_loop_NN_output out;
        out.header.stamp = ros::Time::now();
        out.header.frame_id = "world";
        std::copy(res.begin(), res.begin()+3, out.position.begin());
        std::copy(res.begin()+3, res.begin()+3+9, out.vector_9D_orientation.begin());
        std::copy(res.begin()+12, res.begin()+12+8, out.weight_vector.begin());
        out.position[0] += transformStamped_.transform.translation.x;
        out.position[1] += transformStamped_.transform.translation.y;
        out.position[2] += transformStamped_.transform.translation.z;
        out.tra_time = t_tra_abs_-(i_/NN_freq_);
        NN_output_pub_.publish(out);
        i_++;
    }
}

void InferenceNode::drone_state_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    drone_pos_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    drone_quat_ << msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z;

    // drone state P, V, Q
    drone_state_.segment(0,3) = drone_pos_;
    drone_state_.segment(6,4) = drone_quat_;
    drone_state_(0)-=transformStamped_.transform.translation.x;
    drone_state_(1)-=transformStamped_.transform.translation.y;
    drone_state_(2)-=transformStamped_.transform.translation.z;
    RECEIVED_DRONE_POSE_ = true;
    // ROS_INFO("drone state is %f, %f, %f", drone_state_(0), drone_state_(1), drone_state_(2));
}

void InferenceNode::drone_state_twist_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    drone_vel_ << msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z;
    drone_state_.segment(3,3) = drone_vel_;
    RECEIVED_DRONE_TWIST_ = true;
}

void InferenceNode::gate_points_cb(const geometry_msgs::PoseArray::ConstPtr& msg)
{
    if (msg->poses.size() != 4)
    {
        ROS_WARN("gate points size is not 4, please check the input");
        return;
    }
    for (int i = 0; i < 4; i++)
    {
        gate_points_.segment(i*3,3) << msg->poses[i].position.x, msg->poses[i].position.y, msg->poses[i].position.z;
        gate_points_.segment(i*3,3) -=drone_state_.segment(0,3);
        gate_points_(i*3) -= transformStamped_.transform.translation.x;
        gate_points_(i*3+1) -= transformStamped_.transform.translation.y;
        gate_points_(i*3+2) -= transformStamped_.transform.translation.z;
    }
    // ROS_INFO("gate points are %f, %f, %f", gate_points_(0), gate_points_(1), gate_points_(2));
}

void InferenceNode::mission_start_cb(const gestelt_msgs::GoalsPtr &msg)
{   
    
    
    des_goal_point_ << msg->waypoints[msg->waypoints.size()-1].position.x, msg ->waypoints[msg->waypoints.size()-1].position.y, msg ->waypoints[msg->waypoints.size()-1].position.z;
    des_goal_quat_ << msg->waypoints[msg->waypoints.size()-1].orientation.w, msg->waypoints[msg->waypoints.size()-1].orientation.x, msg->waypoints[msg->waypoints.size()-1].orientation.y, msg->waypoints[msg->waypoints.size()-1].orientation.z;

    //set the goal state
    des_goal_state_.segment(0,3) = des_goal_point_;
    des_goal_state_.segment(3,3) = des_goal_vel_;
    des_goal_state_.segment(6,4) = des_goal_quat_;
    des_goal_state_(0)-=transformStamped_.transform.translation.x;
    des_goal_state_(1)-=transformStamped_.transform.translation.y;
    des_goal_state_(2)-=transformStamped_.transform.translation.z;
    ROS_INFO("goal state is %f, %f, %f", des_goal_state_(0), des_goal_state_(1), des_goal_state_(2));


    last_gate_points_ = gate_points_;
    MISSION_START_ = true;
}