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
        
        ## receive the start and end point, and the initial gate point, from ROS side
        # rewrite the inputs
        self.learing_agile_agent.receive_terminal_states(start=self.start_point,end=self.final_point)

        # problem definition
        self.learing_agile_agent.problem_definition(self.drone_quat)

        # after receiving the waypoints, start the timer to run the learning agile agent
        pub_freq = 40 # hz

        # the traverse time is estimated in 100 hz
        rospy.Timer(rospy.Duration(1/50), self.gate_state_estimation_timer_callback) 
        
        # # the MPC problem is solved in 40 hz
        rospy.Timer(rospy.Duration(1/pub_freq), self.setpoint_timer_callback)
          
      
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
        pos_vel_att_cmd,thrust_vector,callback_runtime=self.learing_agile_agent.solve_problem_gazebo(self.drone_state)
        
        # publish the pos_vel_att_cmd setpoint
        pos_vel_setpoint=PositionTarget()
        pos_vel_setpoint.header.stamp = rospy.Time.now()
        pos_vel_setpoint.header.frame_id = "world"
        pos_vel_setpoint.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        pos_vel_setpoint.type_mask =  PositionTarget.IGNORE_YAW_RATE+PositionTarget.IGNORE_YAW+PositionTarget.IGNORE_AFX+PositionTarget.IGNORE_AFY+PositionTarget.IGNORE_AFZ
        pos_vel_setpoint.position.x = pos_vel_att_cmd[0]
        
        # TODO ONLY FOR TEST
        pos_vel_setpoint.position.y = pos_vel_att_cmd[1]-1.8


        pos_vel_setpoint.position.z = pos_vel_att_cmd[2]
        pos_vel_setpoint.velocity.x = pos_vel_att_cmd[3]
        pos_vel_setpoint.velocity.y = pos_vel_att_cmd[4]
        pos_vel_setpoint.velocity.z = pos_vel_att_cmd[5]

         # attitude setpoint
        # attitude_setpoint=Quaternion()
        # attitude_setpoint.w=pos_vel_att_cmd[6]
        # attitude_setpoint.x=pos_vel_att_cmd[7]
        # attitude_setpoint.y=pos_vel_att_cmd[8]
        # attitude_setpoint.z=pos_vel_att_cmd[9]

        # body rate setpoint
        body_rate_setpoint=Vector3()
        body_rate_setpoint.x=pos_vel_att_cmd[10]
        body_rate_setpoint.y=pos_vel_att_cmd[11]
        body_rate_setpoint.z=pos_vel_att_cmd[12]

        # assemble the setpoint
        mavros_attitude_setpoint=AttitudeTarget()
        mavros_attitude_setpoint.header.stamp = rospy.Time.now()
        mavros_attitude_setpoint.header.frame_id = "world"
        mavros_attitude_setpoint.type_mask = AttitudeTarget.IGNORE_ATTITUDE
        
    
        mavros_attitude_setpoint.thrust = sum(thrust_vector)
        mavros_attitude_setpoint.body_rate=body_rate_setpoint
        # mavros_attitude_setpoint.orientation=attitude_setpoint
        

        # publish the setpoint
        self.next_setpoint_pub.publish(pos_vel_setpoint)
        
        # self.next_attitude_setpoint_pub.publish(mavros_attitude_setpoint)


        # publish the solver input and solver performance
        self.callback_runtime_pub.publish(callback_runtime)

        # publish the solver input state
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

    def drone_state_pose_callback(self,msg):
        """
        receive the drone state from PX4 side
        """
        drone_frame_id = msg.header.frame_id

        self.drone_pos = np.array([msg.pose.position.x,msg.pose.position.y,msg.pose.position.z])

        drone_quat_origin = np.array([msg.pose.orientation.w,msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z])
        drone_quat_origin_euler = tf.transformations.euler_from_quaternion(drone_quat_origin.tolist())
        drone_quat_origin_yaw = drone_quat_origin_euler[2]
        # print('drone_quat_origin_yaw=',drone_quat_origin_yaw)


        # TODO ONLY FOR TEST
        drone_quat_current_yaw = drone_quat_origin_yaw+pi/2
        # print('drone_quat_current_yaw=',drone_quat_current_yaw)
        # self.drone_quat = tf.transformations.quaternion_from_euler(drone_quat_origin_euler[0],drone_quat_origin_euler[1],drone_quat_current_yaw)
        self.drone_quat=np.array([1,0,0,0])
        # TODO ONLY FOR TEST
        self.drone_pos[1]=self.drone_pos[1]+1.8 
        

    def drone_state_twist_callback(self,msg):
        """
        receive the drone state from ROS side
        """
       
        self.drone_vel = np.array([msg.twist.linear.x,msg.twist.linear.y,msg.twist.linear.z])
        self.drone_ang_vel = np.array([msg.twist.angular.x,msg.twist.angular.y,msg.twist.angular.z])
        # print('drone_vel=',self.drone_vel)
        

def main():
    
  
    #---------------------- for ros node integration ----------------------------#

    # # # ros node initialization
    rospy.init_node('learing_agile_agent', anonymous=True)
    
    # create the learning agile agent ROS node object
    learing_agile_agent_node = LearningAgileAgentNode()
    
    rospy.spin()

    

    
if __name__ == '__main__':
    main()
    

   



        #  # attitude setpoint
        # attitude_setpoint=Quaternion()
        # attitude_setpoint.w=pos_vel_att_cmd[6]
        # attitude_setpoint.x=pos_vel_att_cmd[7]
        # attitude_setpoint.y=pos_vel_att_cmd[8]
        # attitude_setpoint.z=pos_vel_att_cmd[9]

        # # body rate setpoint
        # body_rate_setpoint=Vector3()
        # body_rate_setpoint.x=pos_vel_att_cmd[10]
        # body_rate_setpoint.y=pos_vel_att_cmd[11]
        # body_rate_setpoint.z=pos_vel_att_cmd[12]

        # # assemble the setpoint
        # mavros_attitude_setpoint=AttitudeTarget()
        # mavros_attitude_setpoint.header.stamp = rospy.Time.now()
        # mavros_attitude_setpoint.header.frame_id = "world"
        # mavros_attitude_setpoint.type_mask = AttitudeTarget.IGNORE_THRUST
        
        # mavros_attitude_setpoint.orientation=attitude_setpoint
        # mavros_attitude_setpoint.body_rate=body_rate_setpoint