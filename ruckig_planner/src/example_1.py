#!/usr/bin/env python3

import rospy
from gestelt_msgs.msg import Goals
from geometry_msgs.msg import Vector3
import matplotlib.pyplot as plt
import numpy as np

from ruckig import InputParameter, Result, Ruckig, Trajectory

class RuckigPlanner:
    def __init__(self):
        rospy.init_node('RuckigPlanner', anonymous=True)
        rospy.loginfo("[RuckigPlanner] Ruckig is working" )

        rospy.Subscriber('/planner/goals', Goals, self.waypointCB)
        self.goal_waypoints = []
        self.goal_waypoints_vel = []
        self.goal_waypoints_acc = []
        self.position = []
        self.velocity = []
        self.acceleration = []
        rospy.spin()

    def waypointCB(self, msg):
        self.goal_waypoints = [Vector3(wp.position.x, wp.position.y, wp.position.z) for wp in msg.waypoints]
        self.goal_waypoints_vel = [Vector3(vel.linear.x, vel.linear.y, vel.linear.z) for vel in msg.velocities]
        self.goal_waypoints_acc = [Vector3(acc.linear.x, acc.linear.y, acc.linear.z) for acc in msg.accelerations]

        rospy.loginfo("[Trajectory Planner] No. of waypoints: %d", len(msg.waypoints))
        self.planner()

    def planner(self):
        rospy.loginfo("Planner working")
        inp = InputParameter(3)
        
        # Replace with actual values from odom
        inp.current_position = [0.0, 0.0, 1.0]
        inp.current_velocity = [0.0, 0.0, 0.0]
        inp.current_acceleration = [0.0, 0.0, 0.0]

        first_waypoint = self.goal_waypoints[0]
        first_waypoint_vel = self.goal_waypoints_vel[0]
        first_waypoint_acc = self.goal_waypoints_acc[0]
        
        inp.target_position = [first_waypoint.x, first_waypoint.y, first_waypoint.z]
        inp.target_velocity = [first_waypoint_vel.x, first_waypoint_vel.y, first_waypoint_vel.z]
        inp.target_acceleration = [first_waypoint_acc.x, first_waypoint_acc.y, first_waypoint_acc.z]

        inp.max_velocity = [3.0, 3.0, 3.0]
        
        otg = Ruckig(3)
        trajectory = Trajectory(3)

        # Calculate the trajectory in an offline manner
        result = otg.calculate(inp, trajectory)
        if result == Result.ErrorInvalidInput:
            raise Exception('Invalid input!')

        rospy.loginfo(f'Trajectory duration: {trajectory.duration:0.4f} [s]')
        self.generate_trajectory_points(trajectory)

    def generate_trajectory_points(self, trajectory):
        rospy.loginfo("Generating trajectory points")
        self.position, self.velocity, self.acceleration = [], [], []
        flight_duration = trajectory.duration
        time_stamp = 0

        while time_stamp <= flight_duration:
            new_position, new_velocity, new_acceleration = trajectory.at_time(time_stamp)
            self.position.append(new_position)
            self.velocity.append(new_velocity)
            self.acceleration.append(new_acceleration)
            time_stamp += 1/25  # 25Hz for publishing commands

        self.position = np.array(self.position)
        self.velocity = np.array(self.velocity)
        self.acceleration = np.array(self.acceleration)
        rospy.loginfo(f'Position Values are: \n{self.position}')
        self.uavStatePlotter(flight_duration)

    def uavStatePlotter(self, flight_duration):
        x = self.position[:, 0]
        y = self.position[:, 1]
        z = self.position[:, 2]

        # Calculate the number of points
        num_points = len(x)

        # Generate time values based on flight duration
        time = np.linspace(0, flight_duration, num_points)

        # Plot the UAV position
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(time, x, label='X-axis')
        ax.plot(time, y, label='Y-axis')
        ax.plot(time, z, label='Z-axis')

        # Set labels and title
        ax.set_xlabel('Time')
        ax.set_ylabel('Position')
        ax.set_zlabel('Coordinates')
        ax.set_title('UAV Position over Time')

        # Show legend
        ax.legend()

        # Show the plot
        plt.show()

if __name__ == '__main__':
    try:
        ruckig_planner = RuckigPlanner()
    except rospy.ROSInterruptException:
        pass
