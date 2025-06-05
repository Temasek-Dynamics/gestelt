#!/usr/bin/env python3

## this file is for traversing moving narrow window
import os
import time
import rospy
from scipy.spatial.transform import Rotation as R
from collections import deque
import torch
import numpy as np
from multiprocessing import Process, Queue

from config import current_dir,train_cfg, mission_cfg
from quad_model import Gate,get_gate_points
from quad_nn import nn_sample
from quad_moving import binary_search_solver
from geometry.solid_geometry import pitch_from_gate,magni,verify_SVD_ca

from gestelt_msgs.msg import Goals,  CommanderState, close_loop_NN_output
from geometry_msgs.msg import  PoseStamped, TwistStamped, Point, PoseArray, Pose
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker
from learning_agile_sim import MovingGate
from learning_agile_ROS_mission import transform_map_to_world
from quad_policy import get_obs, manual_set_z_forward

##=================Load the model and configuration file=================##
# acquire the current directory


# device=torch.device('cuda' if torch.cuda.is_available() else 'cpu')
device=torch.device('cpu')

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
    
def inference_worker(model, queue_in, queue_out):
    obs = queue_in.get()
    full_input=np.array(obs).reshape([1,-1])
    NN_forward_time=0
    # NN output the traversal time and pose
    t_comp = time.time()
    nn_output = model(torch.tensor(full_input, dtype=torch.float).to(device))[0]
    NN_forward_time=time.time()-t_comp
    out = nn_output.to('cpu').data.numpy()
    verify_tra_R,_=verify_SVD_ca(out[3:12])

    quat=np.roll(R.from_matrix(verify_tra_R).as_quat(),1)
    queue_out.put((out, quat, NN_forward_time))
    
class NN2_ROS_wrapper:
    def __init__(self):
        ## =================load parameters from yaml======================##
        is_simulation=rospy.get_param('mission/is_simulation', False)
        gate_v = rospy.get_param('gate/linear_vel', [0,0,0])
        gate_w = rospy.get_param('gate/angular_vel', 0)
        self.mission_period = rospy.get_param('mission/period', 5)
        NN_model_name=rospy.get_param('NN_deploy_model_name', 'NN2_imitate_1.pth')
        self.NN_freq = rospy.get_param('NN_freq', 100)
        self.MANUAL_SET_POSE_TEST = rospy.get_param('MANUAL_SET_POSE_TEST', False)
        self.PHYSICAL_GATE = rospy.get_param('gate/PHYSICAL_GATE', False)

        ## ==========================initialize ==========================-##
        
        self.state = np.zeros(10)
        self.input_size = train_cfg['model']['input_size']
        self.gate_step = 1/self.NN_freq 
        self.MISSION_START = False
        self.RECEIVED_DRONE_POSE = False
        self.RECEIVED_DRONE_TWIST = False
        self.mission_start_time = rospy.Time.now().to_sec() # will be updated once the mission starts
        self.trans = [0.0, 0.0, 0.0] # initialization to avoid "not found" error
        self.rot = [0.0, 0.0, 0.0, 1.0] # initialization to avoid "not found" error

        ##======== declare the subscriber and the receiver================-##
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.drone_pose_cb)
        rospy.Subscriber("/mavros/local_position/velocity_local", TwistStamped, self.drone_twist_cb)
        rospy.Subscriber("/planner/goals_learning_agile", Goals, self.mission_start_cb)
        rospy.Subscriber("//learning_agile_sim/NN_output", close_loop_NN_output, self.NN_output_cb)
        if self.PHYSICAL_GATE:
            rospy.Subscriber("/vrpn_client_node/gate_tianchensun/pose", PoseStamped, self.physical_gate_pose_cb)

        if not self.MANUAL_SET_POSE_TEST:
            
            self.NN_output_timer = rospy.Timer(rospy.Duration(1/self.NN_freq), self.close_loop_NN_forward)
        
            ##================- load trained DNN2 model ======================-##
            # model_file=os.path.join(current_dir, 'training_data/NN_model',NN2_model_name)
            model_file = os.path.join(current_dir, NN_model_name)
            self.model = torch.load(model_file,map_location=torch.device('cpu'))
        
        
        ##====================-gate initialization ========================##

        ## random gate initialization
        self.env_init_set = nn_sample(TEST=True)
        gate_length = rospy.get_param('gate/length', 1.2)
        self.gate_center = rospy.get_param('mission/gate_position', [0,0,1.5])
        self.moving_gate = MovingGate(self.env_init_set,
                                      gate_center=self.gate_center,
                                      gate_length=gate_length)
        self.moving_gate.set_vel(dt=self.gate_step,gate_v=gate_v,gate_w=gate_w,python_sim_time=self.mission_period)
        self.gate_points_list = self.moving_gate.gate_points_list
        self.gate_t_i = Gate(self.gate_points_list[0]) 
        self.history_obs = deque(maxlen=5)
        
        self.NN_output = rospy.Publisher("/learning_agile_sim/NN_output", close_loop_NN_output, queue_size=1)
        self.vis_NN_trav_pose_pub = rospy.Publisher("/learning_agile_sim/vis_NN_trav_pose", PoseStamped, queue_size=1)

        self.NN_forward_time_pub = rospy.Publisher("/learning_agile_sim/NN_forward_time", Float32, queue_size=1)
        
        self.gate_points_pub = rospy.Publisher("/visual/gate_points", PoseArray, queue_size=1)
        self.gate_state_acquire_timer = rospy.Timer(rospy.Duration(1/self.NN_freq), self.gate_state_acquire)

        self.physical_gate_points_rotated = get_gate_points(gate_center=[0,0,1.8],
                                    gate_length=rospy.get_param('gate/length', 0.6),
                                    gate_width=rospy.get_param('gate/width', 0.45))
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
        print("map to world translation",self.trans)
        print("map to world rotation",self.rot)
        
        # for multiple processes
        self.queue_in = Queue()
        self.queue_out = Queue()
        self.process = Process(target=inference_worker, args=(self.model, self.queue_in, self.queue_out))
        self.process.start()
    
    def physical_gate_pose_cb(self,msg):
        """
        this callback converts the gate pose msg to the four gate corners position.
        The pose is under the world frame.

        Args:
            msg (pose_stamped): the pose of the gate
        """
        gate_center = [msg.pose.position.x,msg.pose.position.y,msg.pose.position.z]
        gate_points_no_pitch=get_gate_points(gate_center=[0,0,0],
                                    gate_length=rospy.get_param('gate/length', 1.2),
                                    gate_width=rospy.get_param('gate/width', 0.5))
        
        gate_rot = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        gate_rot_mat = R.from_quat(gate_rot).as_matrix()

        # rotate the gate points with the gate rotation
        self.physical_gate_points_rotated = gate_points_no_pitch @ gate_rot_mat + gate_center #V^T @ R^T

    
    def gate_state_acquire(self,event):
        ##====frequency of the gate state estimation is the same as the NN2 ========##
        curr_time = rospy.Time.now().to_sec()

        if self.MISSION_START:
            self.i = int((curr_time-(self.mission_start_time))*self.NN_freq)
        else:
            self.i = 0
        # print("i",self.i)
        if self.PHYSICAL_GATE:
            self.gate_t_i = Gate(self.physical_gate_points_rotated)
            self.last_gate_points =(self.physical_gate_points_rotated-self.state[0:3])
        else:
            self.gate_t_i = Gate(self.gate_points_list[self.i]) 
            self.last_gate_points =(self.gate_points_list[0]-self.state[0:3])
        ##============================ gate points publisher =========================##
        gate_points_msg = PoseArray()
        gate_points_msg.header.frame_id = "world"
        gate_points_msg.header.stamp = rospy.Time.now()
        gate_points_msg.poses = []
        for k in range(len(self.gate_t_i.gate_point)):
            single_gate_point = Pose()
            single_gate_point.position.x=self.gate_t_i.gate_point[k,0] + self.trans[0]
            single_gate_point.position.y=self.gate_t_i.gate_point[k,1] + self.trans[1]
            single_gate_point.position.z=self.gate_t_i.gate_point[k,2] + self.trans[2]
            single_gate_point.orientation.x = 0.0
            single_gate_point.orientation.y = 0.0
            single_gate_point.orientation.z = 0.0
            single_gate_point.orientation.w = 1.0
            gate_points_msg.poses.append(single_gate_point)

        self.gate_points_pub.publish(gate_points_msg)
              

   
    def close_loop_NN_forward(self,event):
        """
        forward the close loop neural network 2
        drone state input is under the world frame
        """
       
        if self.MISSION_START and self.RECEIVED_DRONE_TWIST and self.RECEIVED_DRONE_POSE:
            
            ##================= call the gate state estimation function ================##
            
            
            if self.i>=self.mission_period*self.NN_freq-1:
                self.gate_state_acquire_timer.shutdown()
                
                print("Reach Maximum Time, stop the NN forward, set -5s as the traversing time")
                # NN_trav_time_msg = Float32()
                # NN_trav_time_msg.data = -5 # set a constant minus traversing time to indicate the mission is done
                # self.NN_trav_time_pub.publish(NN_trav_time_msg)
                print("shutdown the NN forward timer")
                self.NN_output_timer.shutdown()

            else: 
                obs,_ ,self.last_gate_points = get_obs(
                    self.last_gate_points,
                    self.i,
                    self.input_size,
                    self.state,
                    self.final_point,
                    self.gate_t_i
                )
                
                
                NN_forward_time=0
                # NN output the traversal time and pose
                if not self.MANUAL_SET_POSE_TEST:
                    # print("obs",obs)
                    self.queue_in.put(obs)
                    out, quat, NN_forward_time = self.queue_out.get()
                    # print("out",out)
                    # print("quat",quat)

                else:
                    gate_ori_euler=np.array(mission_cfg['mission']['gate_ori_euler'])
                    self.gate_ori_9d=R.from_euler('zyx',gate_ori_euler).as_matrix().flatten()
                    gate_pitch,out,verify_tra_R = manual_set_z_forward( cur_pos=self.state[0:3],
                                                                        gate_center=self.gate_center,
                                                                        gate_ori_9d=self.gate_ori_9d)
                    quat=np.roll(R.from_matrix(verify_tra_R).as_quat(),1)
                    
                # wrap the NN output as the message
                NN_output = close_loop_NN_output()
                NN_output.header.stamp = rospy.Time.now()
                NN_output.header.frame_id = "world"
                NN_output.position[0:3] = out[0:3]+self.trans
                NN_output.vector_9D_orientation[0:9] = out[3:12]
                NN_output.weight_vector[:]=out[12:12+train_cfg["model"]["weights_vector_length"]]
                NN_output.tra_time = mission_cfg["t_tra_abs"] - self.i*(1/self.NN_freq)

               
                NN_forward_time_msg = Float32()
                NN_forward_time_msg.data = NN_forward_time

                ##= visualize the traversing pose
                vis_NN_trav_pose_msg = PoseStamped()
                vis_NN_trav_pose_msg.header.stamp = rospy.Time.now()
                vis_NN_trav_pose_msg.header.frame_id = "world"
                vis_NN_trav_pose_msg.pose.position.x = NN_output.position[0]+self.state[0]
                vis_NN_trav_pose_msg.pose.position.y = NN_output.position[1]+self.state[1]
                vis_NN_trav_pose_msg.pose.position.z = NN_output.position[2]+self.state[2]
                vis_NN_trav_pose_msg.pose.orientation.w = quat[0]
                vis_NN_trav_pose_msg.pose.orientation.x = quat[1]
                vis_NN_trav_pose_msg.pose.orientation.y = quat[2]
                vis_NN_trav_pose_msg.pose.orientation.z = quat[3]
               

                self.NN_output.publish(NN_output)
                self.vis_NN_trav_pose_pub.publish(vis_NN_trav_pose_msg)
                self.NN_forward_time_pub.publish(NN_forward_time_msg)

    def NN_output_cb(self,msg):
        """receive the model inference node output, and convert
        it into the vis NN traversing pose

        Args:
            msg (close_loop_NN_output): the output of the NN
        """
        if self.MISSION_START:
            out=np.zeros(train_cfg['model']['output_size'])
            out[0:3] = msg.position
            out[3:12] = msg.vector_9D_orientation
            out[12:12+train_cfg["model"]["weights_vector_length"]] = msg.weight_vector
            verify_tra_R,_=verify_SVD_ca(out[3:12])
            quat=np.roll(R.from_matrix(verify_tra_R).as_quat(),1)
            ##= visualize the traversing pose
            vis_NN_trav_pose_msg = PoseStamped()
            vis_NN_trav_pose_msg.header.stamp = rospy.Time.now()
            vis_NN_trav_pose_msg.header.frame_id = "world"
            vis_NN_trav_pose_msg.pose.position.x = msg.position[0]+self.state[0]
            vis_NN_trav_pose_msg.pose.position.y = msg.position[1]+self.state[1]
            vis_NN_trav_pose_msg.pose.position.z = msg.position[2]+self.state[2]
            vis_NN_trav_pose_msg.pose.orientation.w = quat[0]
            vis_NN_trav_pose_msg.pose.orientation.x = quat[1]
            vis_NN_trav_pose_msg.pose.orientation.y = quat[2]
            vis_NN_trav_pose_msg.pose.orientation.z = quat[3]

            self.vis_NN_trav_pose_pub.publish(vis_NN_trav_pose_msg)



    def mission_start_cb(self,msg):
        """
        once this message is received, the mission starts
        """
        print("Detect Mission Start")
        self.final_point = np.array([msg.waypoints[-1].position.x-self.trans[0],
                                     msg.waypoints[-1].position.y-self.trans[1],
                                     msg.waypoints[-1].position.z-self.trans[2]])
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

    ##=================initialize the node====================================##
    rospy.init_node('NN2_ROS_wrapper', anonymous=True)
    nn2_node=NN2_ROS_wrapper()    
    rospy.spin()


# def NN2_forward(self,event):
#     """
#     forward the neural network 2
#     drone state input is under the world frame
#     """
    
#     if self.MISSION_START and self.RECEIVED_DRONE_TWIST and self.RECEIVED_DRONE_POSE:
        
#         ##================= call the gate state estimation function ================##
        
        
#         if self.i>=self.mission_period*self.NN_freq:
#             self.gate_state_acquire_timer.shutdown()
            
#             print("Reach Maximum Time, stop the NN forward, set -5s as the traversing time")
#             NN_trav_time_msg = Float32()
#             NN_trav_time_msg.data = -5 # set a constant minus traversing time to indicate the mission is done
#             self.NN_trav_time_pub.publish(NN_trav_time_msg)
#             print("shutdown the NN forward timer")
#             self.NN_output_timer.shutdown()

#         else:
#             t_comp = time.time()
#             self.gate_state_search()
#             B_S_time=time.time()-t_comp
#             ##============================ NN2 input ===================================##
#             nn2_inputs = np.zeros(15)

        
#             # drone state under the predicted gate frame(based on the binary search)
#             nn2_inputs[0:10] = self.gate_t_i.transform(self.state)
#             nn2_inputs[10:13] = self.gate_t_i.t_final(self.final_point)

#             # width of the gate
#             nn2_inputs[13] = magni(self.gate_t_i.gate_point[0,:]-self.gate_t_i.gate_point[3,:]) # gate width
#             # pitch angle of the gate
#             nn2_inputs[14] = pitch_from_gate(self.gate_t_i)

#             # NN2 OUTPUT the traversal time and pose
#             t_comp = time.time()
#             out = self.model(torch.tensor(nn2_inputs, dtype=torch.float).to(device)).to('cpu')
#             NN_forward_time=time.time()-t_comp
#             out = out.data.numpy()
    
#             ## transfer from Rodrigues parameters to quaternion
#             atti = Rd2Rp(out[3:6])   
#             quat=toQuaternion(atti[0],atti[1])

#             # wrap the NN output as the message
#             NN_trav_pose_msg = PoseStamped()
#             NN_trav_pose_msg.header.stamp = rospy.Time.now()
#             NN_trav_pose_msg.header.frame_id = "world"
#             NN_trav_pose_msg.pose.position.x = out[0]+self.trans[0]+self.gate_t_i.centroid[0]
#             NN_trav_pose_msg.pose.position.y = out[1]+self.trans[1]+self.gate_t_i.centroid[1]
#             NN_trav_pose_msg.pose.position.z = out[2]+self.trans[2]+self.gate_t_i.centroid[2]
#             NN_trav_pose_msg.pose.orientation.w = quat[0]
#             NN_trav_pose_msg.pose.orientation.x = quat[1]
#             NN_trav_pose_msg.pose.orientation.y = quat[2]
#             NN_trav_pose_msg.pose.orientation.z = quat[3]
            
#             NN_trav_time_msg = Float32()
#             NN_forward_time_msg = Float32()
#             B_S_time_msg = Float32()

#             NN_trav_time_msg.data = self.t_tra_rel
#             NN_forward_time_msg.data = NN_forward_time
#             B_S_time_msg.data = B_S_time

#             self.NN_trav_pose_pub.publish(NN_trav_pose_msg)
#             self.NN_trav_time_pub.publish(NN_trav_time_msg)
#             self.NN_forward_time_pub.publish(NN_forward_time_msg)
#             self.B_S_time_pub.publish(B_S_time_msg)

# def gate_state_search(self):

#         """
#         estimate the gate pose, using binary search
#         t_tra_abs: the absolute traversal time w.r.t the mission start time
#         t_tra_rel: the relative traversal time w.r.t the current time

#         """
        
#         ## binary search for the traversal time
#         ## to set the drone state under the gate frame, for the NN2 input
#         self.t_tra_rel = binary_search_solver(self.model,device,self.state,self.final_point,self.gate_t_i,self.moving_gate.V[self.i],self.moving_gate.w)
#         self.t_tra_abs = self.t_tra_rel+self.i*self.gate_step

#         ## obtain the future traversal window state
#         self.gate_t_i.translate(self.t_tra_rel*self.moving_gate.V[self.i])
#         self.gate_t_i.rotate_y(self.t_tra_rel*self.moving_gate.w)
#         # print('rotation matrix I_G=',gate_t_i.I_G)