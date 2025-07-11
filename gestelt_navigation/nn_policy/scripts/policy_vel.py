#!/usr/bin/env python3
import sys
print("Running with Python:", sys.executable)
import time
import numpy as np
import rospy
import pkg_resources
import yaml
import os
import rospkg
import yaml
from std_msgs.msg import Int8
from gestelt_msgs.msg import CommanderState, ExecTrajectory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from enum import Enum
import roslib.packages
from std_msgs.msg import Bool
import tf2_ros
import threading
from std_msgs.msg import Int8
from mavros_msgs.msg import AttitudeTarget
# import tf2_geometry_msgs
# from geometry_msgs.msg import Vector3Stamped
# from geometry_msgs import Posestamped

from modules.policy_simple import *

from signal import signal, SIGINT
import random

def handler(signal_received, frame):
    # Handle any cleanup here
    print('SIGINT or CTRL-C detected. Exiting gracefully')

    exit(0)

class DRONESTATE(Enum):
    INIT = 0
    IDLE = 1
    TAKEOFF = 2
    LAND = 3
    HOVER = 4
    MISSION = 5
    E_STOP = 6

class ServerEvent(Enum):
    TAKEOFF_E = 0   
    LAND_E = 1           
    MISSION_E = 2        
    HOVER_E = 3          
    E_STOP_E = 4       
    EMPTY_E = 5      


class TEST_RENDER(object):

    def __init__(self, policy_path, pc):
        self.pc = pc
        if self.pc == True:
            self.policy = TrackVel(input_dim=16)
        else:
            self.policy = TrackVel(input_dim = 10)
        self.policy.load_state_dict(torch.load(policy_path))
        self.policy.eval()
        self.init_a = np.zeros((1,4))

        target_vel = np.zeros((1, 3))
        # target_vel[:,0]=1
        # target_vel[:,2]=1
        target_vel_pos = np.zeros((1, 3))
        target_vel_pos[:,0] = -1.0
        target_vel_pos[:,1] = 0.0
        target_vel_pos[:,2] = 1.0
        # target_vel = np.random.randn(1, 3)

        target_or = np.zeros((1, 4))
        target_or[:,-1] = np.ones((1,)) * 1
        target_or[:,-3] = np.ones((1,)) * 0
        

        target_unit_vel_pos = target_vel_pos / np.linalg.norm(target_vel_pos, axis = 1).reshape(-1,1)
        self.target_unit_vel_tensor = torch.tensor(target_unit_vel_pos, dtype=torch.float32)

        target_pos = np.array([0, 1,0.0]).reshape(1,3)
        # target_pos = 2 * target_unit_vel_pos + target_pos
        
        # print(f"Target position is: {target_pos}")

        
        # target_pos = target_unit_vel + target_pos
        self.t_vel = torch.tensor(target_vel, dtype=torch.float32)
        self.t_pos = torch.tensor(target_pos, dtype=torch.float32)
        self.init_pos = torch.tensor(target_pos, dtype=torch.float32)
        self.previous_action = torch.tensor(self.init_a, dtype=torch.float32)
        self.t_or = torch.tensor(target_or, dtype=torch.float32  )
        


    def evaluate_(self, pos, att, qd):
        start_time = time.time()
        # if torch.norm(pos - self.t_pos) < 0.3:
        #     print("switching new target")
        #     new_point = self.vector_to_line(pos, self.init_pos, self.target_unit_vel_tensor)
        #     self.t_pos = new_point + self.target_unit_vel_tensor
        #     print(self.t_pos)
        if self.pc == True:
            # delta_vect = self.vector_to_line(pos, self.init_pos , self.target_unit_vel_tensor)
            diff_pos = self.t_pos - pos
            # _, angular_diff = self.quaternion_loss(self.t_or, att)
            # print(diff_pos)
            x = torch.cat((diff_pos, att, qd, self.t_vel), dim=1)
            # x[:,6] = -x[:,6]
            # x = torch.cat((diff_pos, att, qd, self.t_vel), dim=1)
        else:
            vel = qd[:,3:]
            angvel = qd[:,:3]
            diff_vel = self.t_vel - vel
            x = torch.cat((att, angvel, diff_vel), dim=1)
        a = self.policy(x)
        
        self.previous_action = a
        end_time = time.time()
        # print(f"Time taken: {end_time - start_time:.4f} seconds")
        return a
    
    def vector_to_line(self,P, A, d):
        d_unit = d / torch.norm(d, dim=-1, keepdim=True)  # Normalize direction
        AP = P - A
        proj_len = torch.sum(AP * d_unit, dim=-1, keepdim=True)  # scalar projection
        proj = proj_len * d_unit  # vector projection
        Q = A + proj  # Closest point on the line
        return Q - P  # Vector from P to closest point
    
    def update_target_pos(self,P):
        target_pos = P
        self.t_pos = torch.tensor(target_pos, dtype=torch.float32)

    def quaternion_loss(self,y_true, y_pred):
        """
        Computes the angular difference between two quaternions in radians.
        Args:
            y_true: Tensor of shape (batch_size, 4), ground truth quaternions.
            y_pred: Tensor of shape (batch_size, 4), predicted quaternions.
        Returns:
            Scalar tensor: Mean angular difference across the batch.
        """
        # Normalize quaternions to ensure they are unit quaternions
        y_true = F.normalize(y_true, dim=-1)
        y_pred = F.normalize(y_pred, dim=-1)

        # y_pred = y_pred[:,3:]

        # Compute the dot product
        dot_product = torch.sum(y_true * y_pred, dim=-1)  # Shape: (batch_size,)

        # Clamp the dot product to avoid invalid values due to numerical errors
        dot_product_clamped = torch.clamp(dot_product, -1.0 + 1e-6, 1.0 - 1e-6)

        # Compute angular difference (in radians)
        angular_diff = 2 * torch.acos(torch.abs(dot_product_clamped))

        # Return the mean angular difference
        return angular_diff.mean(), angular_diff
    

class NN_POLICY_PLANNER(object):

    def __init__(self, mission_command_mode, policy, inference_timestep, max_angular_rates):
        #Creating subscribers and Publishers
        self.bullet_sim_mutex = threading.Lock()
        self.tfBuffer =  tf2_ros.Buffer(rospy.Duration(10))
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.warp_pose_msg = PoseStamped()
        rospy.sleep(1)

        self.max_angular_rates = max_angular_rates
        self.policy = policy
        self.last_pos_time = None
        self.last_odom_time = None
        

        self.swarm_mode_pub_ = rospy.Publisher('/traj_server/swarm_command', Int8, queue_size=5)
        self.commander_state_sub_ = rospy.Subscriber("/drone0/traj_server/state",CommanderState, self.commStateCb, queue_size = 10)
        self.drone_pose_sub_ = rospy.Subscriber("/drone0/mavros/local_position/pose",PoseStamped, self.poseCb, queue_size = 10)
        self.drone_pose_sub_ = rospy.Subscriber("/drone0/mavros/local_position/odom",Odometry, self.odomCb, queue_size = 10)
        self.drone_pose_sub_ = rospy.Subscriber("/mode_change", Bool, self.modeChgCb, queue_size = 10)
        self.mission_mode_sub_ = rospy.Subscriber("/traj_server/warp_mission_command", Int8, self.missionModeCb, queue_size = 5)
        self.target_position_sub_ = rospy.Subscriber("/drone0/warp/local_position/target_position", PoseStamped, self.targetPosCb, queue_size = 5)

        self.warp_drone_pose_pub_ = rospy.Subscriber('/drone0/warp/local_position/pose', PoseStamped, self.warpPoseCB, queue_size=5)
        self.warp_drone_odom_sub_ = rospy.Subscriber('/drone0/warp/local_position/odom', Odometry, self.warpOdomCB, queue_size=5)
        self.geom_controller_sub_ = rospy.Subscriber('/drone0/setpoint_raw/attitude', AttitudeTarget, self.geomCB, queue_size=5)
        self.geom_controller_pub_ = rospy.Publisher("/drone0/geom_ctrl", AttitudeTarget, queue_size = 5)
        
        #PVA controller trajectory Publisher
        self.pva_traj_pub_ = rospy.Publisher("/drone0/planner_adaptor/exec_trajectory", ExecTrajectory, queue_size = 5)
        self.mission_mode_pub_ = rospy.Publisher("/traj_server/mission_command", Int8, queue_size = 5, latch=False)
        

        self.rate = rospy.Rate(0.02)
        self.event_manager = rospy.Timer(rospy.Duration(inference_timestep), self.eventCB)

        #DRONE STATE MACHINE
        self.drone_state = 0
        self.servent_event = 0

        # DRONE STATES
        self.drone_pos = np.zeros((3,1))
        self.drone_qd = np.zeros((3,1))
        self.drone_quat = np.zeros((4,1))

        self.warp_pos = np.zeros((3,1))
        self.warp_quat = np.zeros((4,1))
        self.warp_qd = np.zeros((3,1))
        
        self.mission_command_mode = 1
        self.warp_mission_command_mode = mission_command_mode
        self.attitude_mode_toggle = 0
        self.action = np.zeros((1,4))
        time.sleep(1)
        self.policy_evaluation_timer = rospy.Timer(rospy.Duration(0.01), self.nn_evaluation)
        

    def commStateCb(self,msg):
        self.drone_state = DRONESTATE[msg.traj_server_state].value

    def missionModeCb(self,msg):
        self.warp_mission_command_mode = msg.data
        print(f"Mission Mode changed to {self.warp_mission_command_mode}")

    def publish_mission(self, mission_num):
        mission_idx = Int8()
        mission_idx.data = int(mission_num)
        self.swarm_mode_pub_.publish(mission_idx)

    def poseCb(self, msg):
        self.drone_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        self.drone_quat = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        if self.last_pos_time is not None:
           time_diff = (msg.header.stamp- self.last_pos_time).to_sec() 
           if time_diff > 0.03:
               print(f"TIME DIFFERENCE EXCEEDED!!! {time_diff} at {msg.header.stamp}")
        self.last_pos_time = msg.header.stamp

        self._pose_odom_pub_callback()

    def odomCb(self, msg):
        self.drone_qd = np.array([msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z])
        if self.last_odom_time is not None:
           time_diff = (msg.header.stamp- self.last_odom_time).to_sec() 
           if time_diff > 0.03:
               print(f"TIME DIFFERENCE ODOM EXCEEDED!!! {time_diff} at {msg.header.stamp}")
        self.last_odom_time = msg.header.stamp

    def warpOdomCB(self,msg):

        self.warp_qd = np.array([msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z, msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z ])
        #print(msg.header.stamp)
        # if self.last_odom_time is not None:
        #    time_diff = (msg.header.stamp- self.last_odom_time).to_sec() 
        #    if time_diff > 0.03:
        #        print(f"TIME DIFFERENCE ODOM EXCEEDED!!! {time_diff} at {msg.header.stamp}")
        # self.last_odom_time = msg.header.stamp

    def geomCB(self, msg):
        self.geom_body_rate = np.array([msg.body_rate.x, msg.body_rate.y, msg.body_rate.z])
        self.geom_thrust = msg.thrust

    def warpPoseCB(self,msg):
        self.warp_q = np.array([msg.pose.position.x, msg.pose.position.y,msg.pose.position.z, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        
        # if self.last_pos_time is not None:
        #    time_diff = (msg.header.stamp- self.last_pos_time).to_sec() 
        #    if time_diff > 0.03:
        #        print(f"TIME DIFFERENCE EXCEEDED!!! {time_diff} at {msg.header.stamp}")
        # self.last_pos_time = msg.header.stamp

    def targetPosCb(self,msg):
        self.warp_target_pos = np.array([msg.pose.position.x, msg.pose.position.y,msg.pose.position.z])
        self.policy.update_target_pos(self.warp_target_pos)

    def eventCB(self, event):
        if self.drone_state == DRONESTATE["IDLE"].value:
            self.publish_mission(ServerEvent["TAKEOFF_E"].value)
        elif self.drone_state == DRONESTATE["HOVER"].value:
            self.publish_mission(ServerEvent["MISSION_E"].value)
        elif self.drone_state == DRONESTATE["MISSION"].value:
            self.executeMission()

    def publishPVA(self):
        pva_traj_msg = ExecTrajectory()
        pva_traj_msg.transform.translation.x = 0.0
        pva_traj_msg.transform.translation.y = 0.0
        pva_traj_msg.transform.translation.z = 1.0
        pva_traj_msg.transform.rotation.x = 0.0
        pva_traj_msg.transform.rotation.y = 0.0
        pva_traj_msg.transform.rotation.z = 0.0
        pva_traj_msg.transform.rotation.w = 1.0
        pva_traj_msg.type_mask = 2048


        #This part is non essential. Merely for debugging purposes
        pva_traj_msg.type_mask = self.attitude_mode_toggle
        pva_traj_msg.throttle = self.action[0,0]
        pva_traj_msg.angular_rates.angular.x = self.action[0,1]   #body rate x
        pva_traj_msg.angular_rates.angular.y = self.action[0,2]     #body rate y
        pva_traj_msg.angular_rates.angular.z = self.action[0,3]     #body rate z

        #Publish the PVA
        self.pva_traj_pub_.publish(pva_traj_msg)

    def publishVEL(self):
        pva_traj_msg = ExecTrajectory()
        pva_traj_msg.transform.translation.x = 0.0
        pva_traj_msg.transform.translation.y = 0.0
        pva_traj_msg.transform.translation.z = 1.0
        pva_traj_msg.transform.rotation.x = 0.0
        pva_traj_msg.transform.rotation.y = 0.0
        pva_traj_msg.transform.rotation.z = 0.0 #0.707
        pva_traj_msg.transform.rotation.w = 1.0 #0.707
        pva_traj_msg.velocity.linear.x = 1.0
        pva_traj_msg.velocity.linear.y = 0.0
        pva_traj_msg.velocity.linear.z = 0.0
        pva_traj_msg.type_mask = 2048

        #Publish the PVA
        self.pva_traj_pub_.publish(pva_traj_msg)

    def publishGeomCtrl(self):
        pva_traj_msg = AttitudeTarget()
        pva_traj_msg.body_rate.x = self.geom_body_rate[0]
        pva_traj_msg.body_rate.y = self.geom_body_rate[1]
        pva_traj_msg.body_rate.z = self.geom_body_rate[2]
        pva_traj_msg.thrust = self.geom_thrust

        #Publish the PVA
        self.geom_controller_pub_.publish(pva_traj_msg)


    def publishATT(self, type_mask, nn_action):
        pva_traj_msg = ExecTrajectory()

        pva_traj_msg.type_mask = type_mask
        # print(self.action)
        pva_traj_msg.throttle = nn_action[0,0]  #0.321

        ### This part will only be taken in by trajectory server if type_mask == 0
        pva_traj_msg.transform.rotation.x = 0.0
        pva_traj_msg.transform.rotation.y = 0.0
        pva_traj_msg.transform.rotation.z = 0.707 
        pva_traj_msg.transform.rotation.w = 0.707

        ### This part will only be taken in by trajectory server if type_mask == 1
        pva_traj_msg.angular_rates.angular.x = nn_action[0,1] * self.max_angular_rates     #body rate x
        pva_traj_msg.angular_rates.angular.y = nn_action[0,2] * self.max_angular_rates   #body rate y
        pva_traj_msg.angular_rates.angular.z = nn_action[0,3] * self.max_angular_rates

        self.pva_traj_pub_.publish(pva_traj_msg)


    def checkNNReadiness(self):
        return not (self.action == 0).all()
    
    def publishMissionCmdMode(self, mode):
        mission_pub_msg = Int8()
        mission_pub_msg.data = mode
        self.mission_mode_pub_.publish(mission_pub_msg)
        if mode == 2:
            print("switched to mission mode 2: ATTITUDE CONTROL")
        elif mode == 4:
            print("switched to mission mode 2: Geom CONTROL")
        else:
            print("switched to mission mode 1: PVA CONTROL")

    def executeMission(self):
        if self.drone_state == DRONESTATE["MISSION"].value:
            if self.mission_command_mode == 1:
                self.publishPVA()
                if self.warp_mission_command_mode == 2:
                    #Check if ready to switch
                    if self.checkNNReadiness():
                        print("me here")
                        self.publishMissionCmdMode(2)
                        self.mission_command_mode = 2
                if self.warp_mission_command_mode == 3:
                    self.publishMissionCmdMode(3)
                    self.mission_command_mode = 3
                if self.warp_mission_command_mode == 4:
                    self.publishMissionCmdMode(4)
                    self.mission_command_mode = 4

                        
            elif self.mission_command_mode == 2:  #This controls the orientation. Attitude and thrust
                if self.checkNNReadiness():
                    self.publishATT(self.attitude_mode_toggle, self.action)


                if self.warp_mission_command_mode == 1:
                    #Check if ready to switch
                    self.publishMissionCmdMode(1)
                    self.mission_command_mode = 1

            elif self.mission_command_mode == 3:
                self.publishVEL()

            elif self.mission_command_mode == 4:
                self.publishGeomCtrl()
                

    def modeChgCb(self, msg):
        if msg.data == True:
            self.attitude_mode_toggle = 1
        elif msg.data == False:
            self.attitude_mode_toggle = 0

    def _pose_odom_pub_callback(self):
        with self.bullet_sim_mutex:
            timestamp = rospy.Time.now()
            
            try:
                trans = self.tfBuffer.lookup_transform("warp", "body", rospy.Time(0))  #"global" + str(i + 1
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                print("error")
            
            # pose
            self.warp_pose_msg.header.stamp = timestamp
            self.warp_pose_msg.header.frame_id = "warp" #TODO: to change to world add static tf
            self.warp_pose_msg.pose.position.x = trans.transform.translation.x
            self.warp_pose_msg.pose.position.y = trans.transform.translation.y
            self.warp_pose_msg.pose.position.z = trans.transform.translation.z
            self.warp_pos = np.array([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z])
            self.warp_pose_msg.pose.orientation.x = trans.transform.rotation.x
            self.warp_pose_msg.pose.orientation.y = trans.transform.rotation.y
            self.warp_pose_msg.pose.orientation.z = trans.transform.rotation.z
            self.warp_pose_msg.pose.orientation.w = trans.transform.rotation.w
            self.warp_quat = np.array([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])
            # self.warp_drone_pose_pub_.publish(self.warp_pose_msg)
            # print(trans.transform.translation)


    def nn_evaluation(self, event):
        warp_q = self.warp_q[3:]
        warp_pos = torch.Tensor(self.warp_q[:3]).unsqueeze(0)
        warp_q = torch.Tensor(warp_q).unsqueeze(0)
        warp_qd = torch.Tensor(self.warp_qd).unsqueeze(0)
        self.action = self.policy.evaluate_(warp_pos, warp_q, warp_qd)
        # print(self.action)




if __name__=="__main__":
    signal(SIGINT, handler)
    print("STARTING NODE")
    policy_file = "20250710-154609" #0.02 good enough for real drone 20250527-122703   0.05-to test 20250624-181715
    print(f"POLICY PATH IS {policy_file}") 

    rospack = rospkg.RosPack()
    path = rospack.get_path('nn_policy')
    full_path = os.path.join(path, "logs/vel_tracking")
    print(f"The full path is {full_path}")
    actual_full_path = os.path.join(full_path, policy_file)
    config_path = os.path.join(actual_full_path,"training_config.yaml")
    full_policy_path = os.path.join(actual_full_path, "policy.pth")

    rospy.init_node("nn_policy_planner2")
    ros_lib = roslib.packages.get_pkg_dir("gestelt_bringup")
    full_config_path = os.path.join(ros_lib, "config/traj_server_vel.yaml")
    with open(full_config_path, 'r') as file:
        loaded_params = yaml.safe_load(file)

    # with open(config_path, 'r') as file:
    #     config_params = yaml.safe_load(file)

    mission_command_mode = loaded_params["mission_command_mode"]

    position_control = True #config_params["position_control"]
    delta_time = 0.05 #float(config_params["delta_time"])
    max_angular_rate = 3.0 #float(config_params["max_angular_rates"])

    
      #vel 20250424-161234 #position 20250424-131220, 20250424-161345
    nn_policy = TEST_RENDER(full_policy_path, position_control) 

    nn_policy_planner = NN_POLICY_PLANNER(mission_command_mode=int(mission_command_mode), policy=nn_policy,
                                          inference_timestep=delta_time, max_angular_rates = max_angular_rate)

    rospy.spin()

    print("done")

