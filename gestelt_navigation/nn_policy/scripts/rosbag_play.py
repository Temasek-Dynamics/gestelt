#!/usr/bin/env python
import rosbag
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import AttitudeTarget
from gestelt_msgs.msg import CommanderState, ExecTrajectory
from scipy.spatial.transform import Rotation as R

import sys

def extract_pose_data(msg):
    pos = msg.pose.position
    if pos.z < 0:
        z = 0
    else:
        z = pos.z
    return [msg.header.stamp.to_sec(), pos.x, pos.y, z]

def extract_ori_data(msg):
    ori = msg.pose.orientation
    return [msg.header.stamp.to_sec(), ori.x, ori.y, ori.z, ori.w]

def extract_attitude_data(msg):
    q = msg.orientation
    body_rates= msg.body_rate
    return [msg.header.stamp.to_sec(), body_rates.x, body_rates.y, body_rates.z, msg.thrust]

def extract_odom_data(msg):
    odom_rates= msg.twist.twist.angular
    if np.abs(odom_rates.x) > 2:
        x = 2
    else:
        x = odom_rates.x
    if np.abs(odom_rates.y) > 2:
        y = 2
    else:
        y = odom_rates.y
    if np.abs(odom_rates.z) > 2:
        z = 2
    else:
        z = odom_rates.z
    return [msg.header.stamp.to_sec(), x,y,z]

def extract_odom_linear_data(msg):
    odom_rates= msg.twist.twist.linear
    if np.abs(odom_rates.x) > 2:
        x = 2
    else:
        x = odom_rates.x
    if np.abs(odom_rates.y) > 2:
        y = 2
    else:
        y = odom_rates.y
    if np.abs(odom_rates.z) > 2:
        z = 2
    else:
        z = odom_rates.z
    return [msg.header.stamp.to_sec(), x,y,z]

def extract_trajectory_data(msg):
    # Assumes first point is representative
    if msg.throttle == 0:
        return None
    t = msg.header.stamp.to_sec()
    throttle = msg.throttle
    angular_rates = msg.angular_rates.angular
    return [t, angular_rates.x, angular_rates.y, angular_rates.z, throttle]

def main(bag_path):
    pose_times, px, py, pz = [], [], [], []
    quat_times, qx, qy,qz,qw = [],[],[],[],[]
    euler_times, ex,ey,ez = [],[],[],[]
    att_times, wxd, wyd, wzd, td = [], [], [], [], []
    traj_times, tx, ty, tz, tt = [], [], [], [], []
    odom_times, wxr, wyr, wzr, lxr,lyr,lzr = [], [], [], [],[],[],[]
    mission_start = 0
    print(f"Reading bag: {bag_path}")
    bag = rosbag.Bag(bag_path)

    for topic, msg, t in bag.read_messages(topics=[
        "/drone0/mavros/local_position/pose",
        "/drone0/mavros/setpoint_raw/attitude",
        "/drone0/planner_adaptor/exec_trajectory",
        "/traj_server/warp_mission_command",
        "/drone0/mavros/local_position/odom",
        "/drone0/mavros/setpoint_raw/target_attitude"
    ]):
        
        if topic == "/traj_server/warp_mission_command":
            mission_start = msg.data
        elif topic=="/drone0/mavros/setpoint_raw/target_attitude":
            print("here")
            
        elif topic == "/drone0/mavros/local_position/pose":
            if mission_start == 3:
                t, x, y, z = extract_pose_data(msg)
                pose_times.append(t-pose_start_time); px.append(x); py.append(y); pz.append(z)
            else:
                t, x, y, z = extract_pose_data(msg)
                pose_start_time = t

            if mission_start ==3:
                t,x,y,z,w = extract_ori_data(msg)
                r = R.from_quat(np.array([x,y,z,w]))
                euler_deg = r.as_euler('xyz', degrees=True)
                quat_times.append(t-quat_start_time); qx.append(x); qy.append(y); qz.append(z); qw.append(w)
                euler_times.append(t- quat_start_time); ex.append(euler_deg[0]); ey.append(euler_deg[1]); ez.append(euler_deg[2])
            else:
                t,x,y,z,w = extract_ori_data(msg)
                quat_start_time = t

        elif topic == "/drone0/mavros/setpoint_raw/attitude":
            if mission_start == 2:
                t, x, y, z, w = extract_attitude_data(msg)
                print(t)
                att_times.append(t-att_start_time); wxd.append(x); wyd.append(y); wzd.append(z); td.append(w)
            else:
                t, x, y, z, w = extract_attitude_data(msg)
                att_start_time = t
        elif topic == "/drone0/mavros/local_position/odom":
            if mission_start == 2:
                t, x, y, z = extract_odom_data(msg)
                odom_times.append(t-odom_start_time); wxr.append(x); wyr.append(y); wzr.append(z)
                t, x, y, z = extract_odom_linear_data(msg)
                lxr.append(x); lyr.append(y); lzr.append(z)
            else:
                t, x, y, z = extract_odom_data(msg)
                odom_start_time = t
        elif topic == "/drone0/planner_adaptor/exec_trajectory":
            if mission_start == 2:
                traj_data = extract_trajectory_data(msg)
                # print(traj_data)
                if traj_data:
                    t, x, y, z, w = traj_data
                    print(w)
                    traj_times.append(t-traj_start_time); tx.append(x); ty.append(y); tz.append(z); tt.append(w)
            else:
                traj_data = extract_trajectory_data(msg)
                if traj_data:
                    t, x, y, z, w = traj_data
                    traj_start_time = t

    bag.close()

    # Plot
    fig, axs = plt.subplots(6, 1, figsize=(12, 10))

    # axs[0].plot(pose_times, px, label='X')
    # axs[0].plot(pose_times, py, label='Y')
    axs[0].plot(pose_times[:1000], pz[:1000], label='Z')
    axs[0].set_title("Local Position (Pose)")
    axs[0].set_ylabel("Position (m)")
    axs[0].legend()
    axs[0].grid()

    # axs[1].plot(att_times, wxd, label='wx')
    # axs[1].plot(att_times, wyd, label='wy')
    # axs[1].plot(att_times, wzd, label='wz')
    axs[1].plot(att_times[:1000], td[:1000], label='thrust')
    axs[1].set_title("Desired Angular Rates and Thrust (Policy)")
    axs[1].set_ylabel("Magnitude")
    axs[1].legend()
    axs[1].grid()

    # axs[2].plot(traj_times, tx, label='X')
    # axs[2].plot(traj_times, ty, label='Y')
    # axs[2].plot(traj_times, tz, label='Z')
    # axs[2].plot(traj_times, tt, label='thrust')
    # axs[2].set_title("Planned Trajectory")
    # axs[2].set_xlabel("Time (s)")
    # axs[2].set_ylabel("Trajectory Pos (m)")
    # axs[2].legend()
    # axs[2].grid()

    axs[2].plot(odom_times, wxr, label='X')
    axs[2].plot(odom_times, wyr, label='Y')
    axs[2].plot(odom_times, wzr, label='Z')
    axs[2].set_title("Angular Velocity (Odom Data)")
    axs[2].set_xlabel("Time (s)")
    axs[2].set_ylabel("Angular Velocity (rad/s)")
    axs[2].legend()
    axs[2].grid()

    axs[3].plot(odom_times[300:500], lxr[300:500], label='X')
    # axs[3].plot(odom_times, lyr, label='Y')
    # axs[3].plot(odom_times, lzr, label='Z')
    axs[3].set_title("Linear Velocity (Odom Data)")
    axs[3].set_xlabel("Time (s)")
    axs[3].set_ylabel("Velocity (m/s)")
    axs[3].legend()
    axs[3].grid()

    axs[4].plot(odom_times[:200], wxr[:200], label='actual')
    axs[4].plot(att_times[:200], wxd[:200], label='desired')
    # axs[3].plot(odom_times, wzr, label='Z')
    axs[4].set_title("Comparing")
    axs[4].set_xlabel("Time (s)")
    axs[4].set_ylabel("Angular Velocity (m/s)")
    axs[4].legend()
    axs[4].grid()

    axs[5].plot(quat_times, qx, label='x')
    axs[5].plot(quat_times, qy, label='y')
    axs[5].plot(quat_times, qz, label='z')
    axs[5].plot(quat_times, qw, label='w')
    # ax[3].plot(odom_times, wzr, label='Z')
    axs[5].set_title("Comparing")
    axs[5].set_xlabel("Time (s)")
    axs[5].set_ylabel("Angular Velocity (m/s)")
    axs[5].legend()
    axs[5].grid()


    plt.tight_layout()
    plt.show()

    fig2, axs2 = plt.subplots(2, 1, figsize=(12, 10))

    axs2[0].plot(quat_times, qx, label='x')
    axs2[0].plot(quat_times, qy, label='y')
    axs2[0].plot(quat_times, qz, label='z')
    axs2[0].plot(quat_times, qw, label='w')
    # ax[0].plot(odom_times, wzr, label='Z')
    axs2[0].set_title("Comparing")
    axs2[0].set_xlabel("Time (s)")
    axs2[0].set_ylabel("Angular Velocity (m/s)")
    axs2[0].legend()
    axs2[0].grid()

    axs2[1].plot(quat_times, ex, label='x')
    axs2[1].plot(quat_times, ey, label='y')
    axs2[1].plot(quat_times, ez, label='z')

    # ax[1].plot(odom_times, wzr, label='Z')
    axs2[1].set_title("Comparing")
    axs2[1].set_xlabel("Time (s)")
    axs2[1].set_ylabel("Angular Velocity (m/s)")
    axs2[1].legend()
    axs2[1].grid()

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    # if len(sys.argv) < 2:
    #     print("Usage: python plot_rosbag_topics.py <your_bag_file.bag>")
    #     sys.exit(1)
    # path = "/home/yanrui/2025-04-15-10-42-54.bag"
    # path = "/home/yanrui/2025-04-15-10-59-40.bag"
    # path = "/home/yanrui/2025-04-15-11-08-27.bag" #preparams
    # path = "/home/yanrui/2025-04-15-13-29-52.bag" #postparams without delay
    # path = "/home/yanrui/2025-04-15-13-57-08.bag"  #postparams with 1 time step delay
    # path = "/home/yanrui/2025-04-15-14-03-28.bag"   #postparams with 1 time step delay run 2
    # path = "/home/yanrui/2025-04-15-14-09-22.bag"   #postparams with 1 time step delay run 3 (second place run)
    # path = "/home/yanrui/2025-04-15-14-29-13.bag"   #postparams with 1 time step delay run 4 (third place run)
    # path = "/home/yanrui/2025-04-15-14-44-35.bag"   #postparams with 1 time step delay run 5 (best run)
    # path = "/home/yanrui/2025-04-25-16-50-46.bag"   #Velocity tracking. Not good

    # path = "/home/yanrui/2025-04-25-14-37-17.bag"   #Velocity tracking. Not good
    path = "/home/yanrui/2025-04-25-18-14-47.bag" 
    
    main(path)
