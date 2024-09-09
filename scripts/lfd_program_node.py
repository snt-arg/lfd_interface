#!/usr/bin/env python3

import rospy

from lfd_program.runner import ProgramRunner


if __name__ == "__main__":

    rospy.init_node("lfd_program_node", anonymous=False)

    robot = rospy.get_param("~robot", "yumi_l")
    duration_scale = rospy.get_param("~duration_scale", 5)
    camera = rospy.get_param("~camera", False)
    if robot=="yumi":
        runner_l = ProgramRunner(robot_type="yumi_l", camera=camera, duration_scale=duration_scale)
        runner_r = ProgramRunner(robot_type="yumi_r", camera=camera, duration_scale=duration_scale)
        runner_l.move(None, "smoothylhometoscrewreverse")
        runner_r.move(None, "smoothyrtestreverse")
        runner_l.move(None, "smoothylhometoscrew")
        runner_r.move(None, "smoothyrtest")
    elif robot=="yumi_r":
        runner = ProgramRunner(robot="yumi_r")
        
        runner.set_motion_mode("dmp")
        runner.configure_motion(demo_name="smoothyrtest",
                                duration_scale=duration_scale)
        runner.move(debug=False)
        runner.configure_motion(demo_name="smoothyrtestreverse",
                                duration_scale=duration_scale)        
        runner.move(debug=False)

        runner.configure_motion(demo_name="smoothyrtest",
                                duration_scale=duration_scale)
        runner.move(debug=False)
        runner.configure_motion(demo_name="smoothyrtestreverse",
                                duration_scale=duration_scale)        
        runner.move(debug=False)
    elif robot=="yumi_l":
        runner = ProgramRunner(robot="yumi_l")
        
        runner.set_motion_mode("dmp")
        runner.configure_motion(demo_name="smoothylhometoscrew",
                                duration_scale=duration_scale)
        runner.move(debug=False)
        runner.configure_motion(demo_name="smoothylhometoscrewreverse",
                                duration_scale=duration_scale)        
        runner.move(debug=False)
        # runner.robot.sm_runner.run_rapid(r_routine="movecone", nonblocking=False)

        # runner.robot.sm_runner.run_rapid(r_routine="mountcone", nonblocking=False)
        # runner.robot.sm_runner.run_rapid(r_routine="mountring", nonblocking=False)

        # runner.gripper("grasp", "open")

        # runner.move(None, "smoothylrecoveryone")
        # runner.gripper("moveto", "10.0")
        # runner.move("screw", "smoothylrecoverytwo")
        # runner.gripper("grasp", "close")

        # runner.move(None, "smoothylhometoscrew")
        # runner.move(None, "smoothylscrewmount")
        # runner.move(None, "smoothylscrewtoring")
        # runner.move(None, "smoothylringmount")

        # runner.move(None, "smoothylhometoscrew")
        # runner.move(None, "smoothylhometoscrewreverse")
        # runner.move(None, "smoothylscrewmount")
        # runner.move(None, "smoothylscrewtoring")


        # ############################################
        # runner.gripper("moveto", "10.0")
        # runner.move("screw", "smoothylhometoscrew")
        # runner.gripper("grasp", "close")
        # #############################################

        # #############################################
        # runner.move(None, "smoothylscrewmount")
        # runner.gripper("grasp", "open")

        # #############################################
        # runner.gripper("grasp", "close")
        # runner.move("ring", "smoothylscrewtoring")
        # runner.gripper("moveto", "1.5")
        # #############################################

        # #############################################        
        # rospy.sleep(0.5)
        # runner.robot.sm_runner.run_rapid(r_routine="movecone", nonblocking=False)
        # rospy.sleep(0.5)
        # #############################################
      
        # #############################################
        # runner.move(None, "smoothylnewringtocone")
        # runner.gripper("grasp", "close")
        # #############################################

        # #############################################
        # runner.gripper("moveto", "10.0")
        # runner.move("screw", "smoothylnewconetoscrew")
        # runner.gripper("grasp", "close")
        # #############################################

        # #############################################
        # runner.move(None, "smoothylscrewmount")
        # runner.gripper("grasp", "open")
        # runner.gripper("grasp", "close")
        # #############################################

        # #############################################
        # rospy.sleep(0.5)
        # runner.robot.sm_runner.run_rapid(r_routine="mountcone", nonblocking=False)
        # runner.robot.sm_runner.run_rapid(r_routine="mountring", nonblocking=False)
        # #############################################

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