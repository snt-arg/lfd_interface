#!/usr/bin/env python3

import rospy

from lfd_program.runner import ProgramRunner


if __name__ == "__main__":

    rospy.init_node("lfd_program_node", anonymous=False)

    robot = rospy.get_param("~robot", "yumi_l")
    duration_scale = rospy.get_param("~duration_scale", 5)
    camera = rospy.get_param("~camera", False)

    if robot=="yumi_r":
        runner = ProgramRunner(robot_type="yumi_r", camera=camera, duration_scale=duration_scale)
        runner.move("nomatter", "smoothyrtest")
    elif robot=="yumi_l":
        runner = ProgramRunner(robot_type="yumi_l", camera=camera, duration_scale=duration_scale)

        # runner.gripper("grasp", "close")

        # runner.move("screw", "smoothylhometoscrew")
        # runner.move(None, "smoothylscrewmount")
        # runner.move("ring", "smoothylscrewtoring")

        # runner.move("ring", "smoothylringtest")

    elif robot=="fr3":
        runner = ProgramRunner(camera=camera, duration_scale=duration_scale)
        runner._dmp_train("smoothfrpick")
        runner._dmp_train("smoothfrplace")

        runner.gripper("open")
        runner.move("nomatter", "smoothfrpick")
        runner.gripper("close", True)
        runner.move(None, "smoothfrplace")
        runner.gripper("open")


    # rospy.spin() 