#!/usr/bin/env python3

import rospy

from lfd_program.core.moveit import MoveitProgram
from lfd_interface.srv import GetDemonstration


class MoveToStart:
    def __init__(self,robot_ns):
        self.moveit_prog = MoveitProgram(robot_ns)
        self.sc_lfd_storage = rospy.ServiceProxy("get_demonstration", GetDemonstration)
        

    def run(self, demo_name):
        resp = self.sc_lfd_storage(name=demo_name)
        demo_traj = resp.Demonstration.joint_trajectory
        self.moveit_prog.plan_joint(demo_traj.points[0])



if __name__ == "__main__":
    rospy.init_node("move_to_start")
    demo_name = rospy.get_param("~demo_name")
    robot_ns = rospy.get_param("~robot_ns")
    print(demo_name)
    move_to_start = MoveToStart(robot_ns)
    move_to_start.run(demo_name)
    # rospy.spin()