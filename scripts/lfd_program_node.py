#!/usr/bin/env python3

import rospy

from lfd_program.dmp import DMPProgram

if __name__ == "__main__":

    rospy.init_node("lfd_program", anonymous=False)

    dmp_prog = DMPProgram("smoothpicknplace")
    dmp_prog.train()
    rospy.loginfo(dmp_prog.demo_goal_joint())
    dmp_prog.visualize()

    rospy.spin() 