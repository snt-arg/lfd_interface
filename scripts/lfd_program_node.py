#!/usr/bin/env python3

import rospy

from lfd_program.runner import ProgramRunner, YumiRunner


if __name__ == "__main__":

    rospy.init_node("lfd_program_node", anonymous=False)

    robot = rospy.get_param("~robot", "yumi_l")
    duration_scale = rospy.get_param("~duration_scale", 5)
    camera = rospy.get_param("~camera", False)

    if robot=="yumi":
        yumi_runner = YumiRunner()
        yumi_runner.set_motion_mode("dmp")
        yumi_runner.configure_motion(duration_scale=duration_scale)

        yumi_runner.configure_l_motion(demo_name="smoothylhometoscrew")
        yumi_runner.configure_r_motion(demo_name="smoothyrtest")
        yumi_runner.move()

        yumi_runner.configure_l_motion(demo_name="smoothylhometoscrewreverse")
        yumi_runner.configure_r_motion(demo_name="smoothyrtestreverse")
        yumi_runner.move() 

    elif robot=="yumi_r":
        runner = ProgramRunner(robot="yumi_r")
        runner.set_motion_mode("dmp")
        runner.configure_motion(duration_scale=duration_scale)

        runner.configure_motion(demo_name="smoothyrtest")
        runner.move()

        runner.configure_motion(demo_name="smoothyrtestreverse")        
        runner.move()

        # runner.configure_motion(demo_name="smoothyrtest")
        # runner.move()

        # runner.configure_motion(demo_name="smoothyrtestreverse")        
        # runner.move()

    elif robot=="yumi_l":
        runner = ProgramRunner(robot="yumi_l")
        runner.set_motion_mode("dmp")
        runner.configure_motion(duration_scale=duration_scale)
        runner.set_camera() 


        # runner.configure_motion(demo_name="smoothylhometoscrewreverse")        
        # runner.move(debug=True)


        runner.configure_motion(demo_name="smoothylhometoscrew")
        runner.locate_target("screw")
        runner.move(debug=True)


        runner.configure_motion(demo_name="smoothylhometoscrewreverse")        
        runner.move(debug=True)



    elif robot=="fr3":
        pass


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