#!/usr/bin/env python3

import rospy

from lfd_program.dmp import DMPProgram

from lfd_program.util.fk import FK

if __name__ == "__main__":

    rospy.init_node("lfd_program", anonymous=False)

    # dmp_prog = DMPProgram("smoothpicknplace")
    # dmp_prog.train()
    # rospy.loginfo(dmp_prog.demo_goal_joint())
    # dmp_prog.visualize()

    fk = FK("fr3_hand_tcp", "fr3_link0")
    print(fk.get_current_fk_pose())

    rospy.spin() 