#!/usr/bin/env python3

## this file is for traversing moving narrow window
import sys
import os
# acquire the current directory
current_dir = os.path.dirname(os.path.abspath(__file__))

# build the path to the subdirectory
subdirectory_path = os.path.join(current_dir, 'Learning_Agile')

# add to sys.path
sys.path.append("../")
sys.path.append(subdirectory_path)

from typing import Any
from Learning_Agile.quad_model import *
from Learning_Agile.quad_policy import *
from Learning_Agile.quad_nn import *
from Learning_Agile.quad_moving import *
from Learning_Agile.learning_agile_agent import LearningAgileAgent
# ros
import numpy as np
import rospy
from gestelt_msgs.msg import CommanderState, Goals, CommanderCommand
from geometry_msgs.msg import Pose, Accel,PoseArray,AccelStamped, TwistStamped, PoseStamped,Quaternion,Vector3
from mavros_msgs.msg import PositionTarget, AttitudeTarget
from std_msgs.msg import Int8, Bool,Float32
import math
import time
import tf


class LearningAgileAgentNode():

    def __init__(self):
        self.NMPC_pub_freq = 100 # hz
        self.tra_NN_pub_freq = 100 # hz

        # drone state subscribers [position, velocity, orientation, angular velocity]
        self.drone_pose_sub = rospy.Subscriber('/mavros/local_position/pose',PoseStamped, self.drone_state_pose_callback)
        self.drone_twist_sub = rospy.Subscriber('/mavros/local_position/velocity_local',TwistStamped, self.drone_state_twist_callback)
        
        # waypoints subscriber [start, end, gate]
        self.waypoints_sub = rospy.Subscriber('/planner/goals_learning_agile',Goals, self.mission_start_callback)
        
        # pos_vel_att_cmd command publisher
        self.next_setpoint_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        self.next_attitude_setpoint_pub = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)

        self.gate_centroid_pub = rospy.Publisher('/learning_agile_agent/gate_centroid', PoseStamped, queue_size=10)
        
        ##--------learning agile agent data----------------##
        # traverse time publisher
        self.traverse_time_pub = rospy.Publisher('/learning_agile_agent/traverse_time', Float32, queue_size=10)
        self.callback_runtime_pub = rospy.Publisher('/learning_agile_agent/callback_runtime', Float32, queue_size=10)
        self.solver_input_state_pub = rospy.Publisher('/learning_agile_agent/solver_input_state', PoseStamped, queue_size=10)
        self.current_pred_traj_pub = rospy.Publisher('/learning_agile_agent/current_pred_traj', PoseArray, queue_size=10)
        # timer for publishing setpoints
        self.terminal_waypoints_received = True
        

        # environment setting
        self.gate_point=np.zeros(3)
        self.start_point=np.zeros(3)
        self.final_point=np.zeros(3)

        # current drone state
        self.drone_state=np.zeros(13)
        self.drone_pos=np.zeros(3)
        self.drone_vel=np.zeros(3)
        self.drone_quat=np.zeros(4)
        self.drone_ang_vel=np.zeros(3)


        ## learning agile agent initialization
        # create the learning agile agent
        self.learing_agile_agent=LearningAgileAgent()

        # time statics
        self.index_t = []

    def mission_start_callback(self,msg):
        """
        when mission starts,
        receive the start and end point, and the initial gate point, from ROS side
        this waypoints callback will only be called once, 
        since the waypoints are only published once when the mission starts
        
        """
        
        # mission start point is the learning agile agent start point
        self.start_point = self.drone_pos

        # self.gate_point = np.array([msg.waypoints[0].position.x,msg.waypoints[0].position.y,msg.waypoints[0].position.z])
        self.final_point = np.array([msg.waypoints[1].position.x,msg.waypoints[1].position.y,msg.waypoints[1].position.z])
        # print("gate point: ",self.gate_point)
        ## receive the start and end point, and the initial gate point, from ROS side
        # rewrite the inputs
        self.learing_agile_agent.receive_terminal_states(start=self.start_point,
                                                         end=self.final_point)
                                                        #  gate_center=self.gate_point)

        # problem definition
        # gazebo_sim=False, means the solver will compile to c code first, then solve the problem
        self.learing_agile_agent.problem_definition(self.drone_quat,gazebo_sim=True)

        # after receiving the waypoints, start the timer to run the learning agile agent
        
        # the traverse time is estimated in 100 hz
        rospy.Timer(rospy.Duration(1/self.tra_NN_pub_freq), self.gate_state_estimation_timer_callback) 
        
        # # the MPC problem is solved in 100 hz
        rospy.Timer(rospy.Duration(1/self.NMPC_pub_freq), self.setpoint_timer_callback)
          
      
    def gate_state_estimation_timer_callback(self, event):
        """
        this function estimate the gate future state, is called in 100 hz
        """
        
        self.drone_state=np.concatenate((self.drone_pos,self.drone_vel,self.drone_quat,self.drone_ang_vel),axis=0).tolist()
        traverse_time, gate_centroid=self.learing_agile_agent.gate_state_estimation(self.drone_state)
        
        # publish the traverse time w.r.t current timestep
        traverse_time_msg=Float32()
        traverse_time_msg.data=traverse_time

        # publish the gate centroid w.r.t the world frame
        gate_centroid_msg=PoseStamped()
        gate_centroid_msg.header.stamp = rospy.Time.now()
        gate_centroid_msg.header.frame_id = "world"
        gate_centroid_msg.pose.position.x = gate_centroid[0]
        gate_centroid_msg.pose.position.y = gate_centroid[1]
        gate_centroid_msg.pose.position.z = gate_centroid[2]
        
        self.gate_centroid_pub.publish(gate_centroid_msg)
        self.traverse_time_pub.publish(traverse_time_msg)
    
    def setpoint_timer_callback(self, event):
        """
        this function solves the MPC problem and output drone PV, is called in 10 hz
        """

        # concatenate the drone state into a list, give it to the learning agile agent
        self.drone_state=np.concatenate((self.drone_pos,self.drone_vel,self.drone_quat,self.drone_ang_vel),axis=0).tolist()
        pos_vel_att_cmd,thrust_each,callback_runtime,current_pred_traj,accelerations=self.learing_agile_agent.solve_problem_gazebo(self.drone_state)
        
        #################################################
        ##---------pub pos_vel_att_cmd setpoint--------##
        #################################################
        
        # publish the pos_vel_att_cmd setpoint
        pos_vel_setpoint=PositionTarget()
        pos_vel_setpoint.header.stamp = rospy.Time.now()
        pos_vel_setpoint.header.frame_id = "world"
        pos_vel_setpoint.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        pos_vel_setpoint.type_mask =  PositionTarget.IGNORE_YAW_RATE+PositionTarget.IGNORE_YAW\
                                    # +PositionTarget.IGNORE_AFX+PositionTarget.IGNORE_AFY+PositionTarget.IGNORE_AFZ\
                                    # +PositionTarget.IGNORE_VX+PositionTarget.IGNORE_VY+PositionTarget.IGNORE_VZ
        pos_vel_setpoint.position.x = pos_vel_att_cmd[0]
        
        # TODO ONLY FOR TEST
        pos_vel_setpoint.position.y = pos_vel_att_cmd[1]-1.8


        pos_vel_setpoint.position.z = pos_vel_att_cmd[2]
        pos_vel_setpoint.velocity.x = pos_vel_att_cmd[3]
        pos_vel_setpoint.velocity.y = pos_vel_att_cmd[4]
        pos_vel_setpoint.velocity.z = pos_vel_att_cmd[5]

        pos_vel_setpoint.acceleration_or_force.x = accelerations[0]
        pos_vel_setpoint.acceleration_or_force.y = accelerations[1]
        pos_vel_setpoint.acceleration_or_force.z = accelerations[2]

        # attitude setpoint
        attitude_setpoint=Quaternion()
        attitude_setpoint.w=pos_vel_att_cmd[6]
        attitude_setpoint.x=pos_vel_att_cmd[7]
        attitude_setpoint.y=pos_vel_att_cmd[8]
        attitude_setpoint.z=pos_vel_att_cmd[9]


        # body rate setpoint
        body_rate_setpoint=Vector3()
        body_rate_setpoint.x=pos_vel_att_cmd[10]
        body_rate_setpoint.y=pos_vel_att_cmd[11]
        body_rate_setpoint.z=pos_vel_att_cmd[12]

        # assemble the body rate and thrust setpoint
        mavros_attitude_setpoint=AttitudeTarget()
        mavros_attitude_setpoint.header.stamp = rospy.Time.now()
        mavros_attitude_setpoint.header.frame_id = "world"
        # mavros_attitude_setpoint.type_mask = AttitudeTarget.IGNORE_PITCH_RATE+AttitudeTarget.IGNORE_THRUST+AttitudeTarget.IGNORE_PITCH_RATE+AttitudeTarget.IGNORE_YAW_RATE+AttitudeTarget.IGNORE_ROLL_RATE
        mavros_attitude_setpoint.type_mask = AttitudeTarget.IGNORE_ATTITUDE

        # each propeller thrust max is 2.466207 N
        # thrust_each: each propeller thrust, in N
        # offset for compensate the gravity
        g_compe=0.4
        mavros_attitude_setpoint.thrust = sum(thrust_each)/(2.466207*4)+g_compe # normalize to [0,1]
        mavros_attitude_setpoint.body_rate=body_rate_setpoint
        # mavros_attitude_setpoint.orientation=attitude_setpoint
        
        #---------- publish the setpoint（PV or attitude)-----------------#
        # self.next_setpoint_pub.publish(pos_vel_setpoint)

        self.next_attitude_setpoint_pub.publish(mavros_attitude_setpoint)


        #TODO ONLY FOR TEST
        #################################################
        ##-pub the solver input state/solver comp time-##
        #################################################
  
        solver_input_state_msg=PoseStamped()
        solver_input_state_msg.header.stamp = rospy.Time.now()
        solver_input_state_msg.header.frame_id = "world"
        solver_input_state_msg.pose.position.x = self.learing_agile_agent.state[0]
        solver_input_state_msg.pose.position.y = self.learing_agile_agent.state[1]
        solver_input_state_msg.pose.position.z = self.learing_agile_agent.state[2]
        solver_input_state_msg.pose.orientation.w = self.learing_agile_agent.state[6]
        solver_input_state_msg.pose.orientation.x = self.learing_agile_agent.state[7]
        solver_input_state_msg.pose.orientation.y = self.learing_agile_agent.state[8]
        solver_input_state_msg.pose.orientation.z = self.learing_agile_agent.state[9]

        self.solver_input_state_pub.publish(solver_input_state_msg)
        
        # publish the solver input and solver performance
        self.callback_runtime_pub.publish(callback_runtime)
        
        
        #################################################
        ##---------pub the current pred traj-----------##
        #################################################

        # publish the current pred traj
        current_pred_traj_msg=PoseArray()
        current_pred_traj_msg.header.stamp = rospy.Time.now()
        current_pred_traj_msg.header.frame_id = "world"
        for i in range(len(current_pred_traj)):
            pose=Pose()
            pose.position.x=current_pred_traj[i][0]
            pose.position.y=current_pred_traj[i][1]-1.8
            pose.position.z=current_pred_traj[i][2]
            pose.orientation.w=current_pred_traj[i][6]
            pose.orientation.x=current_pred_traj[i][7]
            pose.orientation.y=current_pred_traj[i][8]
            pose.orientation.z=current_pred_traj[i][9]
            current_pred_traj_msg.poses.append(pose)
        self.current_pred_traj_pub.publish(current_pred_traj_msg)


    def drone_state_pose_callback(self,msg):
        """
        receive the drone state from PX4 side
        """
        drone_frame_id = msg.header.frame_id

        self.drone_pos = np.array([msg.pose.position.x,msg.pose.position.y,msg.pose.position.z])
        self.drone_quat = np.array([msg.pose.orientation.w,msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z])

        # TODO ONLY FOR TEST
        self.drone_pos[1]=self.drone_pos[1]+1.8 
        

    def drone_state_twist_callback(self,msg):
        """
        receive the drone state from ROS side
        """
       
        self.drone_vel = np.array([msg.twist.linear.x,msg.twist.linear.y,msg.twist.linear.z])
        self.drone_ang_vel = np.array([msg.twist.angular.x,msg.twist.angular.y,msg.twist.angular.z])
        
        

def main():
    
  
    #---------------------- for ros node integration ----------------------------#

    # # # ros node initialization
    rospy.init_node('learing_agile_agent', anonymous=True)
    
    # create the learning agile agent ROS node object
    learing_agile_agent_node = LearningAgileAgentNode()
    
    rospy.spin()

    

    
if __name__ == '__main__':
    main()
    

