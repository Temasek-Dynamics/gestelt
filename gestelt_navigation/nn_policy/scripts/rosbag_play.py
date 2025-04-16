#!/usr/bin/env python
import rosbag
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import AttitudeTarget
from gestelt_msgs.msg import CommanderState, ExecTrajectory

import sys

def extract_pose_data(msg):
    pos = msg.pose.position
    if pos.z < 0:
        z = 0
    else:
        z = pos.z
    return [msg.header.stamp.to_sec(), pos.x, pos.y, z]

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
    att_times, wxd, wyd, wzd, td = [], [], [], [], []
    traj_times, tx, ty, tz, tt = [], [], [], [], []
    odom_times, wxr, wyr, wzr = [], [], [], []
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
            if mission_start == 2:
                t, x, y, z = extract_pose_data(msg)
                pose_times.append(t-pose_start_time); px.append(x); py.append(y); pz.append(z)
            else:
                t, x, y, z = extract_pose_data(msg)
                pose_start_time = t
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
    fig, axs = plt.subplots(4, 1, figsize=(12, 10))

    axs[0].plot(pose_times, px, label='X')
    axs[0].plot(pose_times, py, label='Y')
    axs[0].plot(pose_times, pz, label='Z')
    axs[0].set_title("Local Position (Pose)")
    axs[0].set_ylabel("Position (m)")
    axs[0].legend()
    axs[0].grid()

    axs[1].plot(att_times, wxd, label='Qx')
    axs[1].plot(att_times, wyd, label='Qy')
    axs[1].plot(att_times, wzd, label='Qz')
    axs[1].plot(att_times, td, label='Qw')
    axs[1].set_title("Attitude (Quaternion)")
    axs[1].set_ylabel("Quaternion")
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
    axs[2].set_title("Planned Trajectory")
    axs[2].set_xlabel("Time (s)")
    axs[2].set_ylabel("Trajectory Pos (m)")
    axs[2].legend()
    axs[2].grid()

    axs[3].plot(odom_times[:200], wxr[:200], label='actual')
    axs[3].plot(att_times[:200], wxd[:200], label='desired')
    # axs[3].plot(odom_times, wzr, label='Z')
    axs[3].set_title("Comparing")
    axs[3].set_xlabel("Time (s)")
    axs[3].set_ylabel("Trajectory Pos (m)")
    axs[3].legend()
    axs[3].grid()


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
    path = "/home/yanrui/2025-04-15-14-44-35.bag"   #postparams with 1 time step delay run 5 (best run)

    
    main(path)
