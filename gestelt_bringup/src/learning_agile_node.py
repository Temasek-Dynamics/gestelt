#!/usr/bin/env python3

## this file is for traversing moving narrow window
import sys
import os
from multiprocessing import Process

# acquire the current directory
current_dir = os.path.dirname(os.path.abspath(__file__))

# build the path to the subdirectory
subdirectory_path = os.path.join(current_dir, 'Learning_Agile')

# add to sys.path
sys.path.append("../")
sys.path.append(subdirectory_path)

from typing import Any

from Learning_Agile.learning_agile_agent import LearningAgileAgent
# ros
import numpy as np
import rospy
from gestelt_msgs.msg import Goals,CommanderCommand
from geometry_msgs.msg import Pose,PoseArray,TwistStamped, PoseStamped,Quaternion,Vector3
from mavros_msgs.msg import PositionTarget, AttitudeTarget
from std_msgs.msg import Int8, Bool,Float32
import math
import time
import tf

# running time statistics
import cProfile
# get ros params from rosparam server
hardware_platform=rospy.get_param('mission/hardware_platform', 'laptop')
class LearningAgileAgentNode():

    def __init__(self):
        self.NMPC_pub_freq = 100 # hz
        self.tra_NN_pub_freq = 100 # hz

        # drone state subscribers [position, velocity, orientation, angular velocity]
        self.drone_pose_sub = rospy.Subscriber('/mavros/local_position/pose',PoseStamped, self.drone_state_pose_callback, queue_size=1)
        self.drone_twist_sub = rospy.Subscriber('/mavros/local_position/velocity_local',TwistStamped, self.drone_state_twist_callback, queue_size=1)
        
        # waypoints subscriber [start, end, gate]
        self.waypoints_sub = rospy.Subscriber('/planner/goals_learning_agile',Goals, self.mission_start_callback)
        
        # pos_vel_att_cmd command publisher
        # self.next_setpoint_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        self.next_attitude_setpoint_pub = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)

        self.gate_centroid_pub = rospy.Publisher('/learning_agile_agent/gate_centroid', PoseStamped, queue_size=1)
        
        # Publisher of server events to trigger change of states for trajectory server 
        self.server_event_pub = rospy.Publisher('/traj_server/command', CommanderCommand, queue_size=1)
        
        
        ##--------learning agile agent data----------------##
        # traverse time publisher
        # self.traverse_time_pub = rospy.Publisher('/learning_agile_agent/traverse_time', Float32, queue_size=1)
        self.mpc_runtime_pub_ = rospy.Publisher('/learning_agile_agent/callback_runtime', Float32, queue_size=1)
        # self.solver_input_state_pub = rospy.Publisher('/learning_agile_agent/solver_input_state', PoseStamped, queue_size=10)
        # self.current_pred_traj_pub = rospy.Publisher('/learning_agile_agent/current_pred_traj', PoseArray, queue_size=1)
        # timer for publishing setpoints
        self.terminal_waypoints_received = True
        

        # environment setting
        self.gate_point=np.zeros(3)
        self.start_point=np.zeros(3)
        self.final_point=np.zeros(3)

        # current drone state
        self.drone_state=np.zeros(10)
        self.drone_pos=np.zeros(3)
        self.drone_vel=np.zeros(3)
        self.drone_quat=np.zeros(4)
        self.drone_ang_vel=np.zeros(3)


        ## learning agile agent initialization
        # create the learning agile agent
        self.learning_agile_agent=LearningAgileAgent()

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

        self.gate_point = np.array([msg.waypoints[0].position.x,msg.waypoints[0].position.y,msg.waypoints[0].position.z])
        self.final_point = np.array([msg.waypoints[1].position.x,msg.waypoints[1].position.y,msg.waypoints[1].position.z])
        # print("gate point: ",self.gate_point)
        ## receive the start and end point, and the initial gate point, from ROS side
        # rewrite the inputs

        # self.learning_agile_agent.receive_mission_states(start=self.start_point,
        #                                                  end=self.final_point,
        #                                                  gate_center=self.gate_point,
        #                                                  gate_pose=np.array([0,-0.707/2,0]),
        #                                                 t_tra_abs=1,
        #                                                 max_tra_w=60)


        #------------------------------gazebo hover test--------------------------------------#
        self.learning_agile_agent.receive_mission_states(start=self.start_point,
                                                    end=self.final_point,
                                                    gate_center=self.gate_point,
                                                    gate_pose=np.array([0,-0.0,0]),
                                                t_tra_abs=1,
                                                max_tra_w=0)

        
        # problem definition
        # gazebo_sim=False, means the solver will compile to c code first, then solve the problem
        self.learning_agile_agent.problem_definition(self.drone_quat,\
                                                    gazebo_sim=True,\
                                                    dyn_step=0.002)

        # after receiving the waypoints, start the timer to run the learning agile agent
        
        # the traverse time is estimated in 100 hz
        # rospy.Timer(rospy.Duration(1/self.tra_NN_pub_freq), self.gate_state_estimation_timer_callback) 
        
        # # the MPC problem is solved in 100 hz
        # rospy.Timer(rospy.Duration(1/self.NMPC_pub_freq), self.setpoint_timer_callback)
        rate = rospy.Rate(self.NMPC_pub_freq)
        while not rospy.is_shutdown():
            self.setpoint_timer_callback()
            rate.sleep()
            

          
      
    def gate_state_estimation_timer_callback(self, event):
        """
        this function estimate the gate future state, is called in 100 hz
        """
        #,self.drone_ang_vel
        self.drone_state=np.concatenate((self.drone_pos,self.drone_vel,self.drone_quat),axis=0).tolist()
        self.learning_agile_agent.gate_state_estimation(self.drone_state)
        # traverse_time, gate_centroid=
        
        # publish the traverse time w.r.t current timestep
        # traverse_time_msg=Float32()
        # traverse_time_msg.data=traverse_time

        # publish the gate centroid w.r.t the world frame
        # gate_centroid_msg=PoseStamped()
        # gate_centroid_msg.header.stamp = rospy.Time.now()
        # gate_centroid_msg.header.frame_id = "world"
        # gate_centroid_msg.pose.position.x = gate_centroid[0]
        # gate_centroid_msg.pose.position.y = gate_centroid[1]
        # gate_centroid_msg.pose.position.z = gate_centroid[2]
        
        # self.gate_centroid_pub.publish(gate_centroid_msg)
        # self.traverse_time_pub.publish(traverse_time_msg)
    
    def setpoint_timer_callback(self):
        """
        this function solves the MPC problem and output drone PV, is called in 10 hz
        """

        # concatenate the drone state into a list, give it to the learning agile agent 
        #,self.drone_ang_vel
        self.drone_state=np.concatenate((self.drone_pos,self.drone_vel,self.drone_quat),axis=0).tolist()
        input,callback_runtime,current_pred_traj,NO_SOLUTION_FLAG=self.learning_agile_agent.solve_problem_gazebo(self.drone_state)
        
        #################################################
        ##-pub the solver input state/solver comp time-##
        #################################################        
        # publish the solver input and solver performance
        self.mpc_runtime_pub_.publish(callback_runtime)

        # stop the mission if no solution is found
        if NO_SOLUTION_FLAG:
            print("No solution is found, stop the mission!")
            commander_cmd = CommanderCommand()
            commander_cmd.command = CommanderCommand.HOVER

            self.server_event_pub.publish(commander_cmd)
            rospy.signal_shutdown("MPC no solution, turn the node down")
        else:    
            #################################################
            ##---------pub pos_vel_cmd setpoint------------##
            #################################################


            #################################################
            ##---------pub att_body_rate setpoint----------##
            #################################################
            # # attitude setpoint
            # attitude_setpoint=Quaternion()
            # attitude_setpoint.w=pos_vel_att_cmd[6]
            # attitude_setpoint.x=pos_vel_att_cmd[7]
            # attitude_setpoint.y=pos_vel_att_cmd[8]
            # attitude_setpoint.z=pos_vel_att_cmd[9]


            # body rate setpoint
            body_rate_setpoint=Vector3()
            body_rate_setpoint.x=input[1]
            body_rate_setpoint.y=input[2]
            body_rate_setpoint.z=input[3]

            # assemble the body rate and thrust setpoint
            mavros_attitude_setpoint=AttitudeTarget()
            mavros_attitude_setpoint.header.stamp = rospy.Time.now()
            mavros_attitude_setpoint.header.frame_id = "world"
            # mavros_attitude_setpoint.type_mask = AttitudeTarget.IGNORE_PITCH_RATE+\
                                        # AttitudeTarget.IGNORE_THRUST+AttitudeTarget.IGNORE_PITCH_RATE+\
                                        # AttitudeTarget.IGNORE_YAW_RATE+AttitudeTarget.IGNORE_ROLL_RATE
            
            mavros_attitude_setpoint.type_mask = AttitudeTarget.IGNORE_ATTITUDE
            # mavros_attitude_setpoint.orientation=attitude_setpoint


            # thrust_each: each propeller thrust, in N
            # hover thrust is 0.5775 (normalized to [0,1])
            # drone weight is 0.205kg
            # thrust max is 3.48234 N
            # each propeller thrust max is 0.870585 N+0.15
            # mavros_attitude_setpoint.thrust = sum(thrust_each)/((0.8706+0.25)*4) # normalize to [0,1]
            mavros_attitude_setpoint.thrust = input[0]/((0.8706)*4) # normalize to [0,1] # 

            mavros_attitude_setpoint.body_rate=body_rate_setpoint
            
            
            #---------- publish the setpoint（att or body_rate)-----------------#
            self.next_attitude_setpoint_pub.publish(mavros_attitude_setpoint)


            

            
            
            #################################################
            ##---------pub the current pred traj-----------##
            #################################################

            # # publish the current pred traj
            # current_pred_traj_msg=PoseArray()
            # current_pred_traj_msg.header.stamp = rospy.Time.now()
            # current_pred_traj_msg.header.frame_id = "world"
            # for i in range(len(current_pred_traj)):
            #     pose=Pose()
            #     pose.position.x=current_pred_traj[i][0]
            #     pose.position.y=current_pred_traj[i][1]
            #     pose.position.z=current_pred_traj[i][2]
            #     pose.orientation.w=current_pred_traj[i][6]
            #     pose.orientation.x=current_pred_traj[i][7]
            #     pose.orientation.y=current_pred_traj[i][8]
            #     pose.orientation.z=current_pred_traj[i][9]
            #     current_pred_traj_msg.poses.append(pose)
            # self.current_pred_traj_pub.publish(current_pred_traj_msg)


    def drone_state_pose_callback(self,msg):
        """
        receive the drone state from PX4 side
        """
        drone_frame_id = msg.header.frame_id

        self.drone_pos = np.array([msg.pose.position.x,msg.pose.position.y,msg.pose.position.z])
        self.drone_quat = np.array([msg.pose.orientation.w,msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z])
 
        

    def drone_state_twist_callback(self,msg):
        """
        receive the drone state from ROS side
        """
       
        self.drone_vel = np.array([msg.twist.linear.x,msg.twist.linear.y,msg.twist.linear.z])
        self.drone_ang_vel = np.array([msg.twist.angular.x,msg.twist.angular.y,msg.twist.angular.z])
        
        

def main():
    
  
    #---------------------- for ros node integration ----------------------------#

    # # # ros node initialization
    rospy.init_node('learning_agile_agent', anonymous=True)
    
    # create the learning agile agent ROS node object
    learning_agile_agent_node = LearningAgileAgentNode()
    
    rospy.spin()

    

    
if __name__ == '__main__':
    process = Process(target=main())
    process.start()
    process.join()
    # parent_dir=current_dir+'/..'
    # output_dir=parent_dir+'/data/'
    # file_name=hardware_platform+'_running_time_statistics.prof'
    # rel_path=output_dir+file_name
    # cProfile.run('main()',filename=rel_path)
    


#################################################
##---------pub pos_vel_cmd setpoint------------##
#################################################

# publish the pos_vel_att_cmd setpoint
# pos_vel_setpoint=PositionTarget()
# pos_vel_setpoint.header.stamp = rospy.Time.now()
# pos_vel_setpoint.header.frame_id = "world"
# pos_vel_setpoint.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
# pos_vel_setpoint.type_mask =  PositionTarget.IGNORE_YAW_RATE+PositionTarget.IGNORE_YAW\
#                             # +PositionTarget.IGNORE_AFX+PositionTarget.IGNORE_AFY+PositionTarget.IGNORE_AFZ\
#                             # +PositionTarget.IGNORE_VX+PositionTarget.IGNORE_VY+PositionTarget.IGNORE_VZ

# pos_vel_setpoint.position.x = pos_vel_att_cmd[0]
# pos_vel_setpoint.position.y = pos_vel_att_cmd[1]
# pos_vel_setpoint.position.z = pos_vel_att_cmd[2]
# pos_vel_setpoint.velocity.x = pos_vel_att_cmd[3]
# pos_vel_setpoint.velocity.y = pos_vel_att_cmd[4]
# pos_vel_setpoint.velocity.z = pos_vel_att_cmd[5]

# pos_vel_setpoint.acceleration_or_force.x = accelerations[0]
# pos_vel_setpoint.acceleration_or_force.y = accelerations[1]
# pos_vel_setpoint.acceleration_or_force.z = accelerations[2]

#---------- publish the setpoint（PV or attitude)-----------------#
# self.next_setpoint_pub.publish(pos_vel_setpoint)