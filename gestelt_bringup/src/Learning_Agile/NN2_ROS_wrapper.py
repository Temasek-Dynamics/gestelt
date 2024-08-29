#!/usr/bin/env python3

## this file is for traversing moving narrow window
import sys
import os
import subprocess
import yaml
# acquire the current directory
current_dir = os.path.dirname(os.path.abspath(__file__))

# build the path to the subdirectory
subdirectory_path = os.path.join(current_dir, 'Learning_Agile')

# add to sys.path
sys.path.append("../")
sys.path.append(subdirectory_path)


from quad_model import *
from quad_policy import *
from quad_nn import *
from quad_moving import *

import rospy
from gestelt_msgs.msg import Goals, NNOutputDecisionVariables
from geometry_msgs.msg import Pose, Accel,PoseArray,AccelStamped, Twist
from mavros_msgs.msg import PositionTarget
from std_msgs.msg import Int8, Bool
import math
import time
import tf
from tf.transformations import quaternion_from_euler
from quad_policy import Rd2Rp
from quad_model import toQuaternion
from learning_agile_agent import MovingGate
from learning_agile_mission import transform_map_to_world

device=torch.device('cuda' if torch.cuda.is_available() else 'cpu')
##=========================load the configuration file=======================##
is_simulation=rospy.get_param('mission/is_simulation', False)
goal_position=rospy.get_param('mission/goal_position', [0.0,0.0,1.2])
goal_ori_euler=rospy.get_param('mission/goal_ori_euler', [0,0,0])

class NN2_ROS_wrapper:
    def __init__(self,
                NN2_freq,
                model_file,
                gate_v=np.array([0,0,0]),
                gate_w=0,
                
                STATIC_GATE_TEST=True):
        
        ## ==========================initialize ==========================-##
        self.state = np.zeros(10)
        self.gate_step = 1/NN2_freq # consistent with the python simulation
        self.MISSION_START = False
        self.RECEIVED_DRONE_POSE = False
        self.RECEIVED_DRONE_TWIST = False
        self.mission_start_time = rospy.Time.now().to_sec() # will be updated once the mission starts

        ##======== declare the subscriber and the receiver================-##
        rospy.Subscriber("/mavros/local_position/pose", Pose, self.drone_pose_cb)
        rospy.Subscriber("/mavros/local_position/velocity_body", Twist, self.drone_twist_cb)
        rospy.Subscriber("/planner/goals_learning_agile", Goals, self.mission_start_cb)
        self.NN2_output_pub = rospy.Publisher("/learning_agile/NN2_ouput", NNOutputDecisionVariables, queue_size=10)
        self.NN2_output_timer = rospy.Timer(rospy.Duration(1/NN2_freq), self.NN2_forward)
        
        ##================- load trained DNN2 model ======================-##
        self.model = torch.load(model_file)
        
        
        ##====================-gate initialization ========================##

        ## random gate initialization
        self.env_init_set = nn_sample()
        self.moving_gate = MovingGate(self.env_init_set)
        self.gate_point = self.moving_gate.gate_point
        self.moving_gate.let_gate_move(dt=self.gate_step,gate_v=gate_v,gate_w=gate_w)
        self.gate_points_list = self.moving_gate.gate_points_list
        self.gate_n = gate(self.gate_points_list[0])

        ##============== initial guess of the traversal time==============-##
        self.t_guess = magni(self.gate_n.centroid-self.state[0:3])/1
        
        ##====================test option================================##
        self.STATIC_GATE_TEST = STATIC_GATE_TEST
        
        ##=======================misc ====================================##
        """
        callback function for the drone pose, under the world frame,
        in real flight, world frame is the origin.
        but in the simulation:
        map frame is the origin, world frame is the initial position of the drone,
        the fixed waypoint needs to be transformed from the world frame to the map frame
        """
        ## only works for the simulation
        self.trans,self.rot = transform_map_to_world()
    def gate_state_estimation(self):

        """
        estimate the gate pose, using binary search
        t_tra_abs: the absolute traversal time w.r.t the mission start time
        t_tra_rel: the relative traversal time w.r.t the current time

        """


        
        if self.STATIC_GATE_TEST:
            self.gate_n = gate(self.gate_points_list[0])

            # self.t_tra_abs is manually set
            self.t_tra_rel=self.t_tra_abs-self.i*self.gate_step

        else:

            self.gate_n = gate(self.gate_points_list[self.i])
   
            ## binary search for the traversal time
            ## to set the drone state under the gate frame, for the NN2 input
            self.t_tra_rel = binary_search_solver(self.model,device,self.state,self.final_point,self.gate_n,self.moving_gate.V[self.i],self.moving_gate.w)
            self.t_tra_abs = self.t_tra_rel+self.i*self.gate_step

            ## obtain the future traversal window state
            self.gate_n.translate(self.t_tra_rel*self.moving_gate.V[self.i])
            self.gate_n.rotate_y(self.t_tra_rel*self.moving_gate.w)
            # print('rotation matrix I_G=',gate_n.I_G)
            
       
    def NN2_forward(self,event):
        """
        forward the neural network 2
        drone state input is under the world frame
        """
   
        if self.MISSION_START and self.RECEIVED_DRONE_TWIST and self.RECEIVED_DRONE_POSE:
            print("NN2 forward Start")
            ##================= call the gate state estimation function ================##
            ##====frequency of the gate state estimation is the same as the NN2 ========##
            curr_time = rospy.Time.now().to_sec()
            self.i = int((curr_time-self.mission_start_time)/self.gate_step)
            self.gate_state_estimation()

            ##============================ NN2 input ===================================##
            nn2_inputs = np.zeros(15)

            if self.STATIC_GATE_TEST:
                nn2_inputs[0:10] = self.state 
                nn2_inputs[10:13] = self.final_point
                
                # manually set the traversal time and pose
                out=np.zeros(7)
                out[0:3]=self.gate_center
                out[3:6]=self.gate_ori_RP # Rodrigues parameters

                # relative traversal time
                out[6]=self.t_tra_rel
            else:
                # drone state under the predicted gate frame(based on the binary search)
                nn2_inputs[0:10] = self.gate_n.transform(self.state)
                nn2_inputs[10:13] = self.gate_n.t_final(self.final_point)

                # width of the gate
                nn2_inputs[13] = magni(self.gate_n.gate_point[0,:]-self.gate_n.gate_point[1,:]) # gate width
                # pitch angle of the gate
                nn2_inputs[14] = atan((self.gate_n.gate_point[0,2]-self.gate_n.gate_point[1,2])/(self.gate_n.gate_point[0,0]-self.gate_n.gate_point[1,0])) # compute the actual gate pitch angle in real-time

                # NN2 OUTPUT the traversal time and pose
                out = self.model(nn2_inputs,device).to('cpu')
                out = out.data.numpy()
        

                ## TODO: publish the goal position w.r.t the gate frame to the MPC wrapper as well
                # transfer the gate position to the gate frame
                self.quad1.update_goal_pos(nn2_inputs[10:13])
                
                # wrap the NN output as the message
                NN2_output_msg = NNOutputDecisionVariables()
                NN2_output_msg.header.stamp = rospy.Time.now()
                NN2_output_msg.header.frame_id = "world"
                NN2_output_msg.traverse_position_x = out[0]+self.trans[0]
                NN2_output_msg.traverse_position_x = out[1]+self.trans[1]
                NN2_output_msg.traverse_position_x = out[2]+self.trans[2]
                NN2_output_msg.traverse_orientation_a = out[3]
                NN2_output_msg.traverse_orientation_b = out[4]
                NN2_output_msg.traverse_orientation_c = out[5]
                NN2_output_msg.relative_traverse_time = out[6]
            
            self.NN2_output_pub.publish(NN2_output_msg)

    def mission_start_cb(self,msg):
        """
        once this message is received, the mission starts
        """
        print("Detect Mission Start")
        self.final_point = np.array([msg.waypoints[1].position.x,msg.waypoints[1].position.y,msg.waypoints[1].position.z])
        self.mission_start_time = rospy.Time.now().to_sec()
        self.MISSION_START = True

    def drone_pose_cb(self,msg):
        """
        callback function for the drone pose, under the world frame,
        in real flight, world frame is the origin.
        but in the simulation:
        map frame is the origin, world frame is the initial position of the drone,
        the fixed waypoint needs to be transformed from the world frame to the map frame
        """
        self.state[0] = msg.pose.position.x-self.trans[0]
        self.state[1] = msg.pose.position.y-self.trans[1]
        self.state[2] = msg.pose.position.z-self.trans[2]
        self.state[6:10] = msg.pose.orientation.w,msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z
        self.RECEIVED_DRONE_POSE = True

    def drone_twist_cb(self,msg):
        """
        callback function for the drone twist, under the world frame
        """
        self.state[3] = msg.twist.linear.x
        self.state[4] = msg.twist.linear.y
        self.state[5] = msg.twist.linear.z
        self.RECEIVED_DRONE_TWIST = True
if __name__ == '__main__':
    ##========================= Test Options ================================##
    NN2_model_name = 'NN2_imitate_1.pth'
    STATIC_GATE_TEST = False

    ##=================Load the model and configuration file=================##
    # yaml file dir#
    conf_folder=os.path.abspath(os.path.join(current_dir, '..', '..','config'))
    yaml_file = os.path.join(conf_folder, 'learning_agile_mission.yaml')
    python_sim_data_folder = os.path.join(current_dir, 'python_sim_result')
    model_file=os.path.join(current_dir, 'training_data/NN_model',NN2_model_name)


    ##=================initialize the node====================================##
    rospy.init_node('NN2_ROS_wrapper', anonymous=True)
    nn2_node=NN2_ROS_wrapper(NN2_freq=100,
                              model_file=model_file,
                              gate_v=np.array([0,0,0]),
                              gate_w=0,
                              STATIC_GATE_TEST=STATIC_GATE_TEST)    
    
    rospy.spin()