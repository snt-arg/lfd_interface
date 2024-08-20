#!/usr/bin/env python3

import rospy

from lfd_interface.msg import DemonstrationMsg
from lfd_interface.srv import GetDemonstration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
import matplotlib.pyplot as plt


class ReverseDemo:
    def __init__(self, demo_name):
        self.demo_name = demo_name
        self.sc_lfd_storage = rospy.ServiceProxy("get_demonstration", GetDemonstration)
        self.pub_save_demo = rospy.Publisher("save_demonstration", DemonstrationMsg , queue_size=1)
        

    def run(self):
        resp = self.sc_lfd_storage(name=self.demo_name)
        demo_traj = resp.Demonstration.joint_trajectory
        reversed_traj = self.reverse_trajectory(demo_traj)
        self.plot_trajectories(demo_traj, reversed_traj)
        reversed_demo_msg = DemonstrationMsg()
        reversed_demo_msg.joint_trajectory = reversed_traj
        reversed_demo_msg.name = self.demo_name[:-1] + "reverse" + self.demo_name[-1]
        self.pub_save_demo.publish(reversed_demo_msg)

    def reverse_trajectory(self, trajectory):
        reversed_trajectory = JointTrajectory()
        reversed_trajectory.header = trajectory.header
        reversed_trajectory.joint_names = trajectory.joint_names

        reversed_points = []
        for point in reversed(trajectory.points):
            reversed_point = JointTrajectoryPoint()

            reversed_point.positions = list(point.positions)
            # reversed_point.velocities = list(point.velocities) if point.velocities else []
            # reversed_point.accelerations = list(point.accelerations) if point.accelerations else []
            # reversed_point.effort = list(point.effort) if point.effort else []

            # The time_from_start should be recalculated to reflect the reversed trajectory
            reversed_point.time_from_start = trajectory.points[-1].time_from_start - point.time_from_start

            # Add this reversed point to the list
            reversed_points.append(reversed_point)

        # Add the reversed points to the reversed trajectory
        reversed_trajectory.points = reversed_points

        return reversed_trajectory

    def plot_trajectories(self, original_traj, reversed_traj):
        """
        Plots the joint trajectories for both the original and reversed trajectories.

        Parameters:
            original_traj (JointTrajectory): Original trajectory
            reversed_traj (JointTrajectory): Reversed trajectory
        """
        # Extract time and joint positions for original and reversed trajectories
        original_times = [point.time_from_start.to_sec() for point in original_traj.points]
        reversed_times = [point.time_from_start.to_sec() for point in reversed_traj.points]

        original_positions = np.array([point.positions for point in original_traj.points])
        reversed_positions = np.array([point.positions for point in reversed_traj.points])

        # Plot joint trajectories for each joint
        num_joints = len(original_traj.joint_names)
        plt.figure(figsize=(10, 8))
        
        for joint_idx in range(num_joints):
            plt.subplot(num_joints, 1, joint_idx + 1)
            plt.plot(original_times, original_positions[:, joint_idx], label="Original", color='blue')
            plt.plot(reversed_times, reversed_positions[:, joint_idx], label="Reversed", color='red', linestyle='--')
            plt.ylabel(f"Joint {joint_idx + 1} Position")
            plt.xlabel("Time [s]")
            plt.legend()
            plt.grid(True)

        plt.tight_layout()
        plt.show()

if __name__ == "__main__":
    rospy.init_node("reverse_demo")
    demo_name = rospy.get_param("~demo_name")
    print(demo_name)
    reverse_demo = ReverseDemo(demo_name)
    reverse_demo.run()
    # rospy.spin()