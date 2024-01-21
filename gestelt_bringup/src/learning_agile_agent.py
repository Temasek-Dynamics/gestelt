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

print(sys.path)
from typing import Any
from Learning_Agile.quad_model import *
from Learning_Agile.quad_policy import *
from Learning_Agile.quad_nn import *
from Learning_Agile.quad_moving import *

# ros
import numpy as np
import rospy
from gestelt_msgs.msg import CommanderState, Goals, CommanderCommand
from geometry_msgs.msg import Pose, Accel,PoseArray,AccelStamped, TwistStamped, PoseStamped
from mavros_msgs.msg import PositionTarget
from std_msgs.msg import Int8, Bool
import math
import time
import tf



# load the DNN2 model
abs_dir=os.path.dirname(os.path.abspath(__file__))
print(abs_dir)
FILE = os.path.join(abs_dir, 'Learning_Agile/nn3_1.pth')
# FILE ="nn3_1.pth"



class MovingGate():
    def __init__(self, inputs):
        # gate status
        self.inputs=inputs
        self.gate_point0 = np.array([[-self.inputs[7]/2,-0,1],[self.inputs[7]/2,-0,1],[self.inputs[7]/2,-0,-1],[-self.inputs[7]/2,-0,-1]])
        self.gate1 = gate(self.gate_point0)
        self.gate_init_p = self.inputs[8]

        ## define the kinematics of the narrow window
        self.v =np.array([0,0.0,0.0])
        self.w = 0 #pi/2

        self.gate_move, self.V = self.gate1.move(v = self.v ,w = self.w)
    def let_gate_move(self):
        self.gate1.rotate_y(self.inputs[8])
        gate_point = self.gate1.gate_point
        self.gate1 = gate(gate_point)
        return gate_point
    
class LearningAgileAgent():
    def __init__(self) -> None:

        # inputs [start, end]
        self.inputs = nn_sample()
       
        # drone state
        self.state = np.zeros(13)

        # moving gate
        self.moving_gate = MovingGate(self.inputs)
        self.gate_point = self.moving_gate.let_gate_move()

        # problem definition
        self.model = torch.load(FILE)
        self.horizon = 50


        # planning variables
        self.u = [0,0,0,0]
        self.tm = [0,0,0,0]
        self.state_n = []
        self.control_n = [self.u]
        self.control_tm = [self.tm]
        self.hl_para = [0,0,0,0,0,0,0]
        self.hl_variable = [self.hl_para]
        
        self.gate_move = self.moving_gate.gate_move
        ## let assume the gate is static
        self.gate_n = gate(self.gate_move[0])
        self.t_guess = magni(self.gate_n.centroid-self.state[0:3])/3
        self.Ttra    = []
        self.T       = []
        self.Time    = []
        self.Pitch   = []
        self.i       = 0

        # trajectory pos_vel_cmd
        pos_vel_cmd=np.zeros(9)

    def receive_terminal_states(self,start,end):
        """receive the start and end point, and the initial gate point, from ROS side

        """
        
        self.inputs[0:3]=start
        self.inputs[3:6]=end
        self.start_point = start
        self.final_point = end
        # self.inputs[7:10]=gate

    def problem_definition(self):
        """initial traversal problem

        """
        self.quad1 = run_quad(goal_pos=self.inputs[3:6],ini_r=self.inputs[0:3].tolist(),ini_q=toQuaternion(self.inputs[6],[0,0,1]))
        print('start_point=',self.inputs[0:3])
        print('final_point=',self.inputs[3:6])
        self.quad1.init_obstacle(self.gate_point.reshape(12))
        self.quad1.uav1.setDyn(0.01)
    
    def gate_state_estimation(self,gazebo_model_state=None):
        # run in 100 hz
        if self.i <= 500:
    
            self.gate_n = gate(self.gate_move[self.i])
            
            self.state=gazebo_model_state
        
            self.state_n = [self.state]

            # binary search for the traversal time
            t = solver(self.model,self.state,self.final_point,self.gate_n,self.moving_gate.V[self.i],self.moving_gate.w)
            t_tra = t+self.i*0.01
            gap_pitch = self.moving_gate.gate_init_p + self.moving_gate.w*self.i*0.01
            
            # print('step',self.i,'tranversal time=',t,'gap_pitch=',gap_pitch*180/pi)
            # print('step',i,'abs_tranversal time=',t_tra)
            
            self.Ttra = np.concatenate((self.Ttra,[t_tra]),axis = 0)
            self.T = np.concatenate((self.T,[t]),axis = 0)
            self.Time = np.concatenate((self.Time,[self.i*0.01]),axis = 0)
            self.Pitch = np.concatenate((self.Pitch,[gap_pitch]),axis = 0)
            
            
            # if (self.i%10)==0: # control frequency = 10 hz

            ## obtain the future traversal window state
            self.gate_n.translate(t*self.moving_gate.V[self.i])
            self.gate_n.rotate_y(t*self.moving_gate.w)
            # print('rotation matrix I_G=',gate_n.I_G)
        
           
        self.i += 1

    def solve_problem_gazebo(self,gazebo_model_state=None):
        
        # run in 10 hz
        ## obtain the state in window frame 
        inputs = np.zeros(18)
        inputs[16] = 2 #magni(self.gate_n.gate_point[0,:]-self.gate_n.gate_point[1,:])
        inputs[17] = atan((self.gate_n.gate_point[0,2]-self.gate_n.gate_point[1,2])/(self.gate_n.gate_point[0,0]-self.gate_n.gate_point[1,0])) # compute the actual gate pitch ange in real-time
        inputs[0:13] = self.gate_n.transform(self.state)
        inputs[13:16] = self.gate_n.t_final(self.final_point)
        print('input_UNDER_GATE=',inputs)
        out = self.model(inputs).data.numpy()
        # print('tra_position=',out[0:3],'tra_time_dnn2=',out[6])
        #print(out)
        # if (horizon-1*i/10) <= 30:
        #     Horizon =30
        # else:
        #     Horizon = int(horizon-1*i/10)

        ## solve the mpc problem and get the control command
        self.quad2 = run_quad(goal_pos=inputs[13:16],horizon =50)
        cmd_solution = self.quad2.get_input(inputs[0:13],self.u,out[0:3],out[3:6],out[6])
        self.pos_vel_cmd=cmd_solution['state_traj_opt'][0,:]
        self.u=cmd_solution['control_traj_opt'][0,:]
        
        
        
        # self.state = np.array(self.quad1.uav1.dyn_fn(self.state, self.u)).reshape(13) # Yixiao's simulation environment ('uav1.dyn_fn'), replaced by pybullet
        self.state_n = np.concatenate((self.state_n,[self.state]),axis = 0)
        self.control_n = np.concatenate((self.control_n,[self.u]),axis = 0)
        # u_m = self.quad1.uav1.u_m
        u1 = np.reshape(self.u,(4,1))
        # tm = np.matmul(u_m,u1)
        # tm = np.reshape(tm,4)
        # control_tm = np.concatenate((control_tm,[tm]),axis = 0)
        # self.hl_variable = np.concatenate((self.hl_variable,[out]),axis=0)       
        
        
       
        return self.pos_vel_cmd


    def solve_problem_comparison(self):
        
        
        self.state = self.quad1.ini_state # state= feedback from pybullet, 13-by-1, 3 position, 3 velocity (world frame), 4 quaternion, 3 angular rate
        self.state_n = [self.state]
        for self.i in range(500):
            # decision variable is updated in 100 hz
            self.gate_n = gate(self.gate_move[self.i])
            t = solver(self.model,self.state,self.final_point,self.gate_n,self.moving_gate.V[self.i],self.moving_gate.w)
            t_tra = t+self.i*0.01
            gap_pitch = self.moving_gate.gate_init_p + self.moving_gate.w*self.i*0.01
            
            
            
            # print('step',self.i,'tranversal time=',t,'gap_pitch=',gap_pitch*180/pi)
            # print('step',i,'abs_tranversal time=',t_tra)
            
            self.Ttra = np.concatenate((self.Ttra,[t_tra]),axis = 0)
            self.T = np.concatenate((self.T,[t]),axis = 0)
            self.Time = np.concatenate((self.Time,[self.i*0.01]),axis = 0)
            self.Pitch = np.concatenate((self.Pitch,[gap_pitch]),axis = 0)
            
            
            if (self.i%10)==0: # control frequency = 10 hz

                ## obtain the future traversal window state
                    self.gate_n.translate(t*self.moving_gate.V[self.i])
                    self.gate_n.rotate_y(t*self.moving_gate.w)
                    # print('rotation matrix I_G=',gate_n.I_G)
                
                ## obtain the state in window frame 
                    inputs = np.zeros(18)
                    inputs[16] = 2 # magni(self.gate_n.gate_point[0,:]-self.gate_n.gate_point[1,:])
                    inputs[17] = atan((self.gate_n.gate_point[0,2]-self.gate_n.gate_point[1,2])/(self.gate_n.gate_point[0,0]-self.gate_n.gate_point[1,0])) # compute the actual gate pitch ange in real-time
                    inputs[0:13] = self.gate_n.transform(self.state)
                    inputs[13:16] = self.gate_n.t_final(self.final_point)
                    print('input_UNDER_GATE=',inputs)
                    out = self.model(inputs).data.numpy()
                    print('tra_position=',out[0:3],'tra_time_dnn2=',out[6])
                #print(out)
                    # if (horizon-1*i/10) <= 30:
                    #     Horizon =30
                    # else:
                    #     Horizon = int(horizon-1*i/10)

                ## solve the mpc problem and get the control command
                    self.quad2 = run_quad(goal_pos=inputs[13:16],horizon =50)
                    cmd_solution = self.quad2.get_input(inputs[0:13],self.u,out[0:3],out[3:6],out[6]) # control input 4-by-1 thrusts to pybullet
                    self.u=cmd_solution['control_traj_opt'][0,:]
            
            
            self.state = np.array(self.quad1.uav1.dyn_fn(self.state, self.u)).reshape(13) # Yixiao's simulation environment ('uav1.dyn_fn'), replaced by pybullet
            self.state_n = np.concatenate((self.state_n,[self.state]),axis = 0)
            self.control_n = np.concatenate((self.control_n,[self.u]),axis = 0)
            u_m = self.quad1.uav1.u_m
            u1 = np.reshape(self.u,(4,1))
            tm = np.matmul(u_m,u1)
            tm = np.reshape(tm,4)
            # control_tm = np.concatenate((control_tm,[tm]),axis = 0)
            # self.hl_variable = np.concatenate((self.hl_variable,[out]),axis=0)       
            
           
        np.save('gate_move_traj',self.gate_move)
        np.save('uav_traj',self.state_n)
        np.save('uav_ctrl',self.control_n)
        np.save('abs_tra_time',self.Ttra)
        np.save('tra_time',self.T)
        np.save('Time',self.Time)
        np.save('Pitch',self.Pitch)
        np.save('HL_Variable',self.hl_variable)
        self.quad1.uav1.play_animation(wing_len=1.5,gate_traj1=self.gate_move ,state_traj=self.state_n)
   
    def solve_problem(self):
        ini_state = np.array(self.quad1.ini_state)
        # initializing lists for saving data

        
        state = self.quad1.ini_state # state= feedback from pybullet, 13-by-1, 3 position, 3 velocity (world frame), 4 quaternion, 3 angular rate
        u = [0,0,0,0]
        tm = [0,0,0,0]
        state_n = [state]
        control_n = [u]
        control_tm = [tm]
        hl_para = [0,0,0,0,0,0,0]
        hl_variable = [hl_para]
        gate_move = self.moving_gate.gate_move
        gate_n = gate(gate_move[0])
        t_guess = magni(gate_n.centroid-state[0:3])/3
        Ttra    = []
        T       = []
        Time    = []
        Pitch   = []
        j = 0

        for i in range(500):
            gate_n = gate(gate_move[i])
            t = solver(self.model,state,self.final_point,gate_n,self.moving_gate.V[i],self.moving_gate.w)
            t_tra = t+i*0.01
            gap_pitch = self.moving_gate.gate_init_p + self.moving_gate.w*i*0.01
            
            # print('step',i,'tranversal time=',t,'gap_pitch=',gap_pitch*180/pi)
            # print('step',i,'abs_tranversal time=',t_tra)
            Ttra = np.concatenate((Ttra,[t_tra]),axis = 0)
            T = np.concatenate((T,[t]),axis = 0)
            Time = np.concatenate((Time,[i*0.01]),axis = 0)
            Pitch = np.concatenate((Pitch,[gap_pitch]),axis = 0)
            if (i%10)==0: # control frequency = 10 hz
            ## obtain the current gate state
            ## solve for the traversal time
                # t = solver(model,state,final_point,gate_n,V[i],w)
                # t_tra = t+i*0.01
                # print('step',i,'tranversal time=',t)
                # print('step',i,'abs_tranversal time=',t_tra)
                # Ttra = np.concatenate((Ttra,[t_tra]),axis = 0)
            #print(t,' ',i)
            ## obtain the future traversal window state
                gate_n.translate(t*self.moving_gate.V[i])
                gate_n.rotate_y(t*self.moving_gate.w)
                # print('rotation matrix I_G=',gate_n.I_G)
            ## obtain the state in window frame 
                inputs = np.zeros(18)
                inputs[16] = magni(gate_n.gate_point[0,:]-gate_n.gate_point[1,:])
                inputs[17] = atan((gate_n.gate_point[0,2]-gate_n.gate_point[1,2])/(gate_n.gate_point[0,0]-gate_n.gate_point[1,0])) # compute the actual gate pitch ange in real-time
                inputs[0:13] = gate_n.transform(state)
                inputs[13:16] = gate_n.t_final(self.final_point)
            
                out = self.model(inputs).data.numpy()
                print('tra_position=',out[0:3],'tra_time_dnn2=',out[6])
            #print(out)
                # if (horizon-1*i/10) <= 30:
                #     Horizon =30
                # else:
                #     Horizon = int(horizon-1*i/10)

            ## solve the mpc problem and get the control command
                self.quad2 = run_quad(goal_pos=inputs[13:16],horizon =50)
                cmd_solution = self.quad2.get_input(inputs[0:13],self.u,out[0:3],out[3:6],out[6]) # control input 4-by-1 thrusts to pybullet
                u=cmd_solution['control_traj_opt'][0,:]
                j += 1
            state = np.array(self.quad1.uav1.dyn_fn(state, u)).reshape(13) # Yixiao's simulation environment ('uav1.dyn_fn'), replaced by pybullet
            state_n = np.concatenate((state_n,[state]),axis = 0)
            control_n = np.concatenate((control_n,[u]),axis = 0)
            u_m = self.quad1.uav1.u_m
            u1 = np.reshape(u,(4,1))
            tm = np.matmul(u_m,u1)
            tm = np.reshape(tm,4)
            control_tm = np.concatenate((control_tm,[tm]),axis = 0)
            hl_variable = np.concatenate((hl_variable,[out]),axis=0)
        np.save('gate_move_traj',gate_move)
        np.save('uav_traj',state_n)
        np.save('uav_ctrl',control_n)
        np.save('abs_tra_time',Ttra)
        np.save('tra_time',T)
        np.save('Time',Time)
        np.save('Pitch',Pitch)
        np.save('HL_Variable',hl_variable)
        self.quad1.uav1.play_animation(wing_len=1.5,gate_traj1=gate_move ,state_traj=state_n)
        
        # quad1.uav1.plot_input(control_n)
        # quad1.uav1.plot_angularrate(state_n)
        # quad1.uav1.plot_T(control_tm)
        # quad1.uav1.plot_M(control_tm)

# waypoint subscriber: start, end, gate
class LearningAgileAgentNode():

    def __init__(self):

        # drone state subscribers [position, velocity, orientation, angular velocity]
        self.drone_pose_sub = rospy.Subscriber('/mavros/local_position/pose',PoseStamped, self.drone_state_pose_callback)
        self.drone_twist_sub = rospy.Subscriber('/mavros/local_position/velocity_local',TwistStamped, self.drone_state_twist_callback)
        
        # waypoints subscriber [start, end, gate]
        self.waypoints_sub = rospy.Subscriber('/planner/goals_learning_agile',Goals, self.waypoints_callback)
        
        # pos_vel_cmd command publisher
        self.next_setpoint_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        
        # timer for publishing setpoints
        self.terminal_waypoints_received = False
        

        self.drone_state=np.zeros(13)
        self.gate_point=np.zeros(3)
        self.start_point=np.zeros(3)
        self.final_point=np.zeros(3)
        self.drone_pos=np.zeros(3)
        self.drone_vel=np.zeros(3)
        self.drone_quat=np.zeros(4)
        self.drone_ang_vel=np.zeros(3)


        ## learning agile agent initialization
        # create the learning agile agent
        self.learing_agile_agent=LearningAgileAgent()

    def waypoints_callback(self,msg):
        """
        receive the start and end point, and the initial gate point, from ROS side
        this waypoints callback will only be called once, 
        since the waypoints are only published once when the mission starts
        
        """
         
        self.start_point = self.drone_pos
        self.gate_point = np.array([msg.waypoints[0].position.x,msg.waypoints[0].position.y,msg.waypoints[0].position.z])
        self.final_point = np.array([msg.waypoints[1].position.x,msg.waypoints[1].position.y,msg.waypoints[1].position.z])
        
        ## receive the start and end point, and the initial gate point, from ROS side
        # rewrite the inputs
        self.learing_agile_agent.receive_terminal_states(start=self.start_point,end=self.final_point)

        # problem definition
        self.learing_agile_agent.problem_definition()

        # after receiving the waypoints, start the timer to run the learning agile agent
        pub_freq = 10 # hz
        rospy.Timer(rospy.Duration(1/pub_freq), self.setpoint_timer_callback)
        rospy.Timer(rospy.Duration(1/100), self.gate_state_estimation_timer_callback)    
      
    def gate_state_estimation_timer_callback(self, event):
        """
        this function estimate the gate future state, is called in 100 hz
        """
        self.drone_state=np.concatenate((self.drone_pos,self.drone_vel,self.drone_quat,self.drone_ang_vel),axis=0).tolist()
        self.learing_agile_agent.gate_state_estimation(self.drone_state)
       
    
    def setpoint_timer_callback(self, event):
        
        # solve the problem

        # concatenate the drone state into a list, give it to the learning agile agent
        self.drone_state=np.concatenate((self.drone_pos,self.drone_vel,self.drone_quat,self.drone_ang_vel),axis=0).tolist()
        pos_vel_cmd=self.learing_agile_agent.solve_problem_gazebo(self.drone_state)
        
        # publish the pos_vel_cmd setpoint
        setpoint=PositionTarget()
        setpoint.header.stamp = rospy.Time.now()
        setpoint.header.frame_id = "world"
        setpoint.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        setpoint.type_mask =  PositionTarget.IGNORE_YAW_RATE+PositionTarget.IGNORE_YAW+PositionTarget.IGNORE_AFX+PositionTarget.IGNORE_AFY+PositionTarget.IGNORE_AFZ
        setpoint.position.x = pos_vel_cmd[0]
        
        # TODO ONLY FOR TEST
        setpoint.position.y = pos_vel_cmd[1]+5


        setpoint.position.z = pos_vel_cmd[2]
        setpoint.velocity.x = pos_vel_cmd[3]
        setpoint.velocity.y = pos_vel_cmd[4]
        setpoint.velocity.z = pos_vel_cmd[5]


        self.next_setpoint_pub.publish(setpoint)
            



    def drone_state_pose_callback(self,msg):
        """
        receive the drone state from ROS side
        """
        drone_frame_id = msg.header.frame_id

        self.drone_pos = np.array([msg.pose.position.x,msg.pose.position.y,msg.pose.position.z])
        self.drone_quat = np.array([msg.pose.orientation.w,msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z])
        
        # TODO ONLY FOR TEST
        self.drone_pos[1]=self.drone_pos[1]-5 
        # print('drone_pos=',self.drone_pos)

    def drone_state_twist_callback(self,msg):
        """
        receive the drone state from ROS side
        """
       
        self.drone_vel = np.array([msg.twist.linear.x,msg.twist.linear.y,msg.twist.linear.z])
        self.drone_ang_vel = np.array([msg.twist.angular.x,msg.twist.angular.y,msg.twist.angular.z])
        # print('drone_vel=',self.drone_vel)
        

def main():
    
    #---------------------- for ros node integration ----------------------------#
    # # ros node initialization
    # rospy.init_node('learing_agile_agent', anonymous=True)
    
    # # create the learning agile agent ROS node object
    # learing_agile_agent_node = LearningAgileAgentNode()
    
    # rospy.spin()


    ## --------------for single planning part test-------------------------------##.
    # create the learning agile agent
    learing_agile_agent=LearningAgileAgent()
    ## receive the start and end point, and the initial gate point, from ROS side
    # rewrite the inputs
    learing_agile_agent.receive_terminal_states(start=np.array([0,-5,1]),end=np.array([0,5,1]))

    # problem definition
    learing_agile_agent.problem_definition()

    # solve the problem
    learing_agile_agent.solve_problem_comparison()

    
if __name__ == '__main__':
    main()
    

   