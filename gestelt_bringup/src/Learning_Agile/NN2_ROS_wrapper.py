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
from gestelt_msgs.msg import Goals,  CommanderState
from geometry_msgs.msg import  PoseStamped, TwistStamped, Point
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker
import time
from quad_policy import Rd2Rp
from quad_model import toQuaternion
from learning_agile_agent import MovingGate
from learning_agile_mission import transform_map_to_world

device=torch.device('cuda' if torch.cuda.is_available() else 'cpu')
##=========================load the configuration file=======================##
STATIC_GATE_TEST = rospy.get_param('STATIC_GATE_TEST', True)
is_simulation=rospy.get_param('mission/is_simulation', False)
goal_position=rospy.get_param('mission/goal_position', [0.0,0.0,1.2])
goal_ori_euler=rospy.get_param('mission/goal_ori_euler', [0,0,0])
gate_v = rospy.get_param('gate/linear_vel', [0,0,0])
gate_w = rospy.get_param('gate/angular_vel', 0)
###============================== Dictionary of UAV states =================================##
server_states = {}

# Check if UAV has achived desired traj_server_state
def check_traj_server_states(des_traj_server_state):
    if len(server_states.items()) == 0:
        print("No Server states received!")
        return False
    
    for server_state in server_states.items():
        # print(f"{server_state[0]}: {des_traj_server_state}")
        if server_state[1].traj_server_state != des_traj_server_state:
            return False
    return True

def get_server_state_callback():
    msg = rospy.wait_for_message(f"/traj_server/state", CommanderState, timeout=5.0)
    server_states[str(msg.drone_id)] = msg
    # print("==================")
    # print(msg)
    # print("==================")
class NN2_ROS_wrapper:
    def __init__(self,
                mission_period,
                NN2_freq,
                model_file,
                gate_v=np.array([0,0,0]),
                gate_w=0):
        
        ## ==========================initialize ==========================-##
        self.mission_period = mission_period
        self.NN2_freq = NN2_freq
        self.state = np.zeros(10)
        self.gate_step = 1/NN2_freq 
        self.MISSION_START = False
        self.RECEIVED_DRONE_POSE = False
        self.RECEIVED_DRONE_TWIST = False
        self.mission_start_time = rospy.Time.now().to_sec() # will be updated once the mission starts

        ##======== declare the subscriber and the receiver================-##
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.drone_pose_cb)
        rospy.Subscriber("/mavros/local_position/velocity_local", TwistStamped, self.drone_twist_cb)
        rospy.Subscriber("/planner/goals_learning_agile", Goals, self.mission_start_cb)
        self.NN_trav_pose_pub = rospy.Publisher("/learning_agile_agent/NN_trav_pose", PoseStamped, queue_size=1)
        self.NN_trav_time_pub = rospy.Publisher("/learning_agile_agent/NN_trav_time", Float32, queue_size=1)

        self.B_S_time_pub = rospy.Publisher("/learning_agile_agent/B_S_time", Float32, queue_size=1)
        self.NN_forward_time_pub = rospy.Publisher("/learning_agile_agent/NN_forward_time", Float32, queue_size=1)
        self.gate_vis_pub = rospy.Publisher("/learning_agile_agent/gate_vis", Marker, queue_size=1)
        
        self.gate_vis_timer = rospy.Timer(rospy.Duration(1/NN2_freq), self.gate_vis)
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
        
        
        ##=======================misc ====================================##
        """
        callback function for the drone pose, under the world frame,
        in real flight, world frame is the origin.
        but in the simulation:
        map frame is the origin, world frame is the initial position of the drone,
        the fixed waypoint needs to be transformed from the world frame to the map frame
        """
        ## only works for the simulation
        self.trans,self.rot = transform_map_to_world(is_simulation)
        print("translation",self.trans)
        print("rotation",self.rot)
    def gate_state_estimation(self):

        """
        estimate the gate pose, using binary search
        t_tra_abs: the absolute traversal time w.r.t the mission start time
        t_tra_rel: the relative traversal time w.r.t the current time

        """
        
        ## binary search for the traversal time
        ## to set the drone state under the gate frame, for the NN2 input
        self.t_tra_rel = binary_search_solver(self.model,device,self.state,self.final_point,self.gate_n,self.moving_gate.V[self.i],self.moving_gate.w)
        self.t_tra_abs = self.t_tra_rel+self.i*self.gate_step

        ## obtain the future traversal window state
        self.gate_n.translate(self.t_tra_rel*self.moving_gate.V[self.i])
        self.gate_n.rotate_y(self.t_tra_rel*self.moving_gate.w)
        # print('rotation matrix I_G=',gate_n.I_G)
    
    def gate_vis(self,event):
        ##====frequency of the gate state estimation is the same as the NN2 ========##
        curr_time = rospy.Time.now().to_sec()

        if self.MISSION_START:

            self.i = int((curr_time-(self.mission_start_time))*self.NN2_freq)
            # print("i",self.i)
        
            self.gate_n = gate(self.gate_points_list[self.i])
            ##============================ gate visualization =========================##
            gate_vis_msg = Marker()
            gate_vis_msg.header.frame_id = "world"
            gate_vis_msg.header.stamp = rospy.Time.now()
            gate_vis_msg.ns = "gate"
            gate_vis_msg.id = 0
            gate_vis_msg.type = Marker.LINE_STRIP
            gate_vis_msg.action = Marker.ADD
            for k in range(len(self.gate_n.gate_point)):
                p = self.gate_points_list[self.i,k,:]
                gate_vis_msg.points.append(Point(x=p[0],y=p[1],z=p[2]))
            gate_vis_msg.points.append(Point(x=self.gate_points_list[self.i,0,0],y=self.gate_points_list[self.i,0,1],z=self.gate_points_list[self.i,0,2]))
            gate_vis_msg.color.a = 1.0
            gate_vis_msg.color.r = 1.0
            gate_vis_msg.color.g = 0.0
            gate_vis_msg.color.b = 0.0
            gate_vis_msg.scale.x = 0.05   # control the width of the line
            gate_vis_msg.pose.orientation.w = 1.0
            gate_vis_msg.pose.orientation.x = 0.0
            gate_vis_msg.pose.orientation.y = 0.0
            gate_vis_msg.pose.orientation.z = 0.0
            gate_vis_msg.pose.position.x = 0 + self.trans[0]
            gate_vis_msg.pose.position.y = 0 + self.trans[1]
            gate_vis_msg.pose.position.z = 0 + self.trans[2]
            self.gate_vis_pub.publish(gate_vis_msg)
        else:
            self.i = 0

    def NN2_forward(self,event):
        """
        forward the neural network 2
        drone state input is under the world frame
        """
       
        if self.MISSION_START and self.RECEIVED_DRONE_TWIST and self.RECEIVED_DRONE_POSE:
            
            ##================= call the gate state estimation function ================##
            
            
            if self.i>self.mission_period*self.NN2_freq:
                self.NN2_output_timer.shutdown()
                print("Reach Maximum Time, NN2 output timer shutdown")
                return
            
            t_comp = time.time()
            self.gate_state_estimation()
            B_S_time=time.time()-t_comp
            ##============================ NN2 input ===================================##
            nn2_inputs = np.zeros(15)

           
            # drone state under the predicted gate frame(based on the binary search)
            nn2_inputs[0:10] = self.gate_n.transform(self.state)
            nn2_inputs[10:13] = self.gate_n.t_final(self.final_point)

            # width of the gate
            nn2_inputs[13] = magni(self.gate_n.gate_point[0,:]-self.gate_n.gate_point[1,:]) # gate width
            # pitch angle of the gate
            nn2_inputs[14] = atan((self.gate_n.gate_point[0,2]-self.gate_n.gate_point[1,2])/(self.gate_n.gate_point[0,0]-self.gate_n.gate_point[1,0])) # compute the actual gate pitch angle in real-time

            # NN2 OUTPUT the traversal time and pose
            t_comp = time.time()
            out = self.model(nn2_inputs,device).to('cpu')
            NN_forward_time=time.time()-t_comp
            out = out.data.numpy()
    
            ## transfer from Rodrigues parameters to quaternion
            atti = Rd2Rp(out[3:6])   
            quat=toQuaternion(atti[0],atti[1])

            
            # wrap the NN output as the message
            NN_trav_pose_msg = PoseStamped()
            NN_trav_pose_msg.header.stamp = rospy.Time.now()
            NN_trav_pose_msg.header.frame_id = "world"
            NN_trav_pose_msg.pose.position.x = out[0]+self.trans[0]+self.gate_n.centroid[0]
            NN_trav_pose_msg.pose.position.y = out[1]+self.trans[1]+self.gate_n.centroid[1]
            NN_trav_pose_msg.pose.position.z = out[2]+self.trans[2]+self.gate_n.centroid[2]
            NN_trav_pose_msg.pose.orientation.w = quat[0]
            NN_trav_pose_msg.pose.orientation.x = quat[1]
            NN_trav_pose_msg.pose.orientation.y = quat[2]
            NN_trav_pose_msg.pose.orientation.z = quat[3]
            
            NN_trav_time_msg = Float32()
            NN_forward_time_msg = Float32()
            B_S_time_msg = Float32()

            NN_trav_time_msg.data = self.t_tra_rel
            NN_forward_time_msg.data = NN_forward_time
            B_S_time_msg.data = B_S_time

            self.NN_trav_pose_pub.publish(NN_trav_pose_msg)
            self.NN_trav_time_pub.publish(NN_trav_time_msg)
            self.NN_forward_time_pub.publish(NN_forward_time_msg)
            self.B_S_time_pub.publish(B_S_time_msg)
            
    def mission_start_cb(self,msg):
        """
        once this message is received, the mission starts
        """
        print("Detect Mission Start")
        self.final_point = np.array([msg.waypoints[1].position.x-self.trans[0],
                                     msg.waypoints[1].position.y-self.trans[1],
                                     msg.waypoints[1].position.z-self.trans[2]])
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

    ##=================Load the model and configuration file=================##
    # yaml file dir#
    conf_folder=os.path.abspath(os.path.join(current_dir, '..', '..','config'))
    yaml_file = os.path.join(conf_folder, 'learning_agile_mission.yaml')
    python_sim_data_folder = os.path.join(current_dir, 'python_sim_result')
    model_file=os.path.join(current_dir, 'training_data/NN_model',NN2_model_name)


    ##=================initialize the node====================================##
    rospy.init_node('NN2_ROS_wrapper', anonymous=True)
    nn2_node=NN2_ROS_wrapper(mission_period=5,
                            NN2_freq=100,
                            model_file=model_file,
                            gate_v=np.array(gate_v),
                            gate_w=gate_w)    
    rospy.spin()
