import rospy
from signal import signal, SIGINT
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Int8
import random
import numpy as np


def handler(signal_received, frame):
    # Handle any cleanup here
    print('SIGINT or CTRL-C detected. Exiting gracefully')

    exit(0)

class State:
    IDLE = "idle"
    COLLECTING_DATA = "data_collecting"


class Data_Collection_Manager(object):
    def __init__(self):

        self.FSM = State()
        self.change_init_pos = rospy.Publisher("/drone0/jax/init_pose", PoseStamped, queue_size = 5)
        # self.start_data_collection_pub_ = rospy.Publisher("/traj_server/warp_mission_recorder", Bool, queue_size=5)
        ### Period means for each cycle. Need to get info that it has been completed to send new initial position point
        self.data_collect_period_sub_ = rospy.Subscriber("/traj_server/warp_mission_period_completed", Bool, self.data_collect_periodCB, queue_size=5)

        ## Global means I want to globally stop data collection. Need to receive info from cmdline to stop.
        self.data_collect_global_sub_ = rospy.Subscriber("/traj_server/warp_mission_global_completed", Bool, self.data_collect_globalCB, queue_size=5)

        ## Glob2local means I am going to publish to the local manager to stop so that it can save the data.
        self.data_collect_glob2loc_pub_ = rospy.Publisher("/traj_server/warp_mission_global2local_completed", Bool, queue_size=5)

        ## Ready to start means that once the local manager detects that the drone is in position. It is ready to start one cycle of collection
        self.ready_to_start_sub_ = rospy.Subscriber("/traj_server/ready_to_start", Bool, self.ready_to_startCB, queue_size=5)

        ## These are the messages need to publish to local manager to start the drone flying and data collection
        self.mode_chg_pub_ = rospy.Publisher("/mode_change", Bool, queue_size=5)
        self.warp_mission_change = rospy.Publisher("/traj_server/warp_mission_command", Int8, queue_size=5)

        ## State machine timer
        self.event_manager = rospy.Timer(rospy.Duration(0.01), self.eventCB)

        self.data_collect_period_bool = False
        self.ready_to_start_bool = False
        self.data_collect_global_interim_bool = False
        self.data_collect_global_bool = False

        self.published_mission_mode_bool = False
        
    def data_collect_globalCB(self, msg):
        self.data_collect_global_interim_bool = msg.data
        # print(self.data_collect_global_interim_bool)

    def data_collect_periodCB(self, msg):
        self.data_collect_period_bool = msg.data

    def ready_to_startCB(self,msg):
        if self.ready_to_start_bool == False and msg.data == True:
            #Count down before starting
            rospy.sleep(1)
        self.ready_to_start_bool = msg.data

    def eventCB(self, event):

        if self.data_collect_global_bool == True:
            if self.ready_to_start_bool == True and self.data_collect_period_bool == False:
                #Start data collection
                print("Ready to Start")
                if self.published_mission_mode_bool == False:
                    mode_chg_msg = Bool()
                    mode_chg_msg.data = True
                    self.mode_chg_pub_.publish(mode_chg_msg)

                    warpmission_mode = Int8()
                    warpmission_mode.data = 2
                    self.warp_mission_change.publish(warpmission_mode)
                    self.published_mission_mode_bool = True

            elif self.ready_to_start_bool == True and self.data_collect_period_bool == True:
                #Reached one period time to restart the next round
                # ready_to_start_bool = Bool()
                # ready_to_start_bool.data = True
                # self.start_data_collection_pub_.publish(ready_to_start_bool)
                rospy.sleep(1)
                #rest all parameters
                self.reset()
            elif self.ready_to_start_bool == False and self.data_collect_period_bool == True:
                raise ValueError("Start bool is false when data collected bool is true")
        

        if self.ready_to_start_bool == False and self.data_collect_period_bool == False:
            if self.data_collect_global_interim_bool == False:
                self.data_collect_global_bool = False
                # Stopping global data collection
                self.reset_global()
                #Publishing to the local manager to stop collecting data
                stop_collecting_global2local = Bool()
                stop_collecting_global2local.data = True
                self.data_collect_glob2loc_pub_.publish(stop_collecting_global2local)

        if self.data_collect_global_interim_bool == True:
            self.data_collect_global_bool = True

    def reset_global(self):
        self.data_collect_global_interim_bool = False
        self.data_collect_global_bool = False
        
    def reset(self):
        self.ready_to_start_bool = False
        self.data_collect_period_bool = False
        self.published_mission_mode_bool = False
        #Reset initial position
        initial_pose = PoseStamped()
        #Generate initial position around a particular region
        init_y = np.random.uniform(2, 0)
        init_z = np.random.uniform(0.5,1)
        init_x = np.random.uniform(-0.5,0.5)
        initial_pose.pose.position.x = init_x
        initial_pose.pose.position.y = init_y
        initial_pose.pose.position.z = init_z 

        self.change_init_pos.publish(initial_pose) 
        print("Published new initial point")      




if __name__=="__main__":
    signal(SIGINT, handler)
    print("STARTING DATA COLLECTION NODE")
    rospy.init_node("Data_collection_node")
    dcm = Data_Collection_Manager()
    rospy.spin()