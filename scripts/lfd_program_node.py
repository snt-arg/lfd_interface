#!/usr/bin/env python3

import rospy

from lfd_program.runner import ProgramRunner


if __name__ == "__main__":

    rospy.init_node("lfd_program_node", anonymous=False)

    robot = "yumi_r"
    camera = True

    if robot=="yumi_r":
        runner = ProgramRunner(robot_type="yumi_r", camera=camera)
        runner.move("nomatter", "smoothyrtest")
    elif robot=="yumi_l":
        runner = ProgramRunner(robot_type="yumi_l", camera=camera)
        runner.move(None, "smoothyltest")
    elif robot=="fr3":
        runner = ProgramRunner(camera=camera)
        runner._dmp_train("smoothfrpick")
        runner._dmp_train("smoothfrplace")

        runner.gripper("open")
        runner.move("nomatter", "smoothfrpick")
        runner.gripper("close", True)
        runner.move(None, "smoothfrplace")
        runner.gripper("open")


    # rospy.spin() 