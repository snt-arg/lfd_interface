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
        yumi_runner.yumi_l_program.set_camera()
        debug=False

        # YUMI LEFT
        ###################
        yumi_runner.gripper_l.moveto("10")
        yumi_runner.configure_l_motion(demo_name="smoothylhometoscrewupdate")
        yumi_runner.locate_l_target("screw")
        yumi_runner.move_l(debug=debug)
        yumi_runner.gripper_l.grasp("close")
        ###################

        while True:
            #############################################
            rospy.sleep(0.3)
            yumi_runner.gripper_r.moveto("20")
            yumi_runner.configure_l_motion(demo_name="smoothylscrewmountupdate")        
            yumi_runner.configure_r_motion(demo_name="smoothyrhometoconeupdate")
            yumi_runner.move(debug=False)
            yumi_runner.gripper_l.grasp("open")
            rospy.sleep(0.3)
            yumi_runner.gripper_l.grasp("close")
            #############################################

            #############################################
            yumi_runner.gripper_r.grasp("close")
            yumi_runner.configure_l_motion(demo_name="smoothylscrewtoringupdate")
            yumi_runner.configure_r_motion(demo_name="smoothyrmoveconeoneupdate", duration_scale=3)
            yumi_runner.locate_l_target("ring")        
            yumi_runner.move(debug=False)
            yumi_runner.gripper_l.moveto("2")
            yumi_runner.configure_r_motion(duration_scale=duration_scale)
            #############################################


            # YUMI RIGHT
            #################
            yumi_runner.configure_r_motion(demo_name="smoothyrmoveconetwoupdate")
            yumi_runner.move_r(debug=debug)
            #################
            
            
            # YUMI LEFT
            ###################
            yumi_runner.configure_l_motion(demo_name="smoothylringmountupdate")        
            yumi_runner.move_l(debug=debug)
            yumi_runner.gripper_l.grasp("close")
            ###################

#     #######################################################################


            # YUMI RIGHT
            #################
            yumi_runner.configure_r_motion(demo_name="smoothyrmountconeupdate")
            yumi_runner.move_r(debug=debug)
            yumi_runner.gripper_r.moveto("10")
            #################

            # YUMI RIGHT
            #################
            yumi_runner.configure_r_motion(demo_name="smoothyrinsertringoneupdate")
            yumi_runner.move_r(debug=debug)
            yumi_runner.gripper_r.moveto("0.5")
            #################

            # YUMI RIGHT
            #################
            yumi_runner.configure_r_motion(demo_name="smoothyrinsertringtwoupdate")
            yumi_runner.move_r(debug=debug, motion_sup=False)
            #################

            # YUMI RIGHT
            #################
            yumi_runner.gripper_r.moveto("5")
            yumi_runner.yumi_r_program.robot_program.execute_motion(r_routine="removecone")
            # yumi_runner.gripper_r.grasp("close")
            #################

            # YUMI RIGHT
            #################
            yumi_runner.configure_r_motion(demo_name="smoothyrputbackconeupdate")
            yumi_runner.move_r(debug=debug)
            yumi_runner.gripper_r.moveto("5")
            #################

            # YUMI RIGHT
            #################
            yumi_runner.configure_r_motion(demo_name="smoothyrpickscrewupdatereverse")
            yumi_runner.move_r(debug=debug)        
            #################

            # YUMI RIGHT
            #################
            yumi_runner.configure_r_motion(demo_name="smoothyrremovescrewoneupdatereverse")
            yumi_runner.move_r(debug=debug)
            #################

            # YUMI RIGHT
            #################
            yumi_runner.gripper_r.grasp("close")
            yumi_runner.configure_r_motion(demo_name="smoothyrremovescrewoneupdate")
            yumi_runner.move_r(debug=debug)        
            #################

            # YUMI RIGHT
            #################
            yumi_runner.configure_r_motion(demo_name="smoothyrremovescrewtwoupdate")
            yumi_runner.move_r(debug=debug)
            yumi_runner.gripper_r.moveto("10")   
            #################
            rospy.sleep(0.3)
            
            # YUMI LEFT
            ###################
            yumi_runner.gripper_l.moveto("10")
            yumi_runner.configure_l_motion(demo_name="smoothylringtoscrewloopupdatereverse")  
            yumi_runner.configure_r_motion(demo_name="smoothyrgobackhomeupdate")
            yumi_runner.locate_l_target("screw")      
            yumi_runner.move(debug=False)
            yumi_runner.gripper_l.grasp("close")
            ###################
            
            # YUMI RIGHT
            #################
            # yumi_runner.move_r(debug=debug) 
            #################

# ########################################################################################








        # yumi_runner.configure_l_motion(demo_name="smoothylhometoscrew")
        # yumi_runner.configure_r_motion(demo_name="smoothyrtest")
        # yumi_runner.move(motion_sup=False)

        # yumi_runner.configure_l_motion(demo_name="smoothylhometoscrewreverse")
        # yumi_runner.configure_r_motion(demo_name="smoothyrtestreverse")
        # yumi_runner.move() 

    elif robot=="yumi_r":
        runner = ProgramRunner(robot="yumi_r")
        runner.set_motion_mode("dmp")
        runner.configure_motion(duration_scale=duration_scale)

        # runner.gripper.moveto("20")
        # runner.configure_motion(demo_name="smoothyrhometoconeupdate")
        # runner.move(debug=True)

        # runner.gripper.grasp("close")
        # runner.configure_motion(demo_name="smoothyrmoveconeoneupdate", duration_scale=3)
        # runner.move(debug=True, motion_sup=False)

        # runner.configure_motion(demo_name="smoothyrmoveconetwoupdate", duration_scale=duration_scale)
        # runner.move(debug=True)


        # runner.configure_motion(demo_name="smoothyrmountconeupdate")
        # runner.move(debug=True)
        # runner.gripper.moveto("10")


        # runner.configure_motion(demo_name="smoothyrinsertringoneupdate")
        # runner.move(debug=True)
        # runner.gripper.moveto("0.5")

        # runner.configure_motion(demo_name="smoothyrinsertringtwoupdate", duration_scale=duration_scale)
        # runner.move(debug=True, motion_sup=False)

        # runner.gripper.moveto("5")
        # runner.robot_program.execute_motion(r_routine="removecone")
        # runner.gripper.grasp("close")

        # runner.configure_motion(demo_name="smoothyrremovescrewoneupdate")
        # runner.move(debug=True) 


        runner.configure_motion(demo_name="smoothyrputbackconeupdate", duration_scale=duration_scale)
        runner.move(debug=True)
        runner.gripper.moveto("4")

        runner.configure_motion(demo_name="smoothyrpickscrewupdatereverse")
        runner.move(debug=True)        


        runner.configure_motion(demo_name="smoothyrremovescrewoneupdatereverse")
        runner.move(debug=True)

        runner.gripper.grasp("close")
        runner.configure_motion(demo_name="smoothyrremovescrewoneupdate")
        runner.move(debug=True)        

        runner.configure_motion(demo_name="smoothyrremovescrewtwoupdate")
        runner.move(debug=True)          

        runner.configure_motion(demo_name="smoothyrgobackhomeupdate")
        runner.move(debug=True)  

        #####################################

        # runner.configure_motion(demo_name="smoothyrmoveconetwoupdatereverse")
        # runner.move(debug=True)

        # runner.configure_motion(demo_name="smoothyrmoveconeoneupdatereverse")
        # runner.move(debug=True)
        # runner.gripper.moveto("20")


        # runner.configure_motion(demo_name="smoothyrhometoconeupdatereverse")
        # runner.move(debug=True)


        # runner.configure_motion(demo_name="smoothyrtestreverse")        
        # runner.move()

        # runner.configure_motion(demo_name="smoothyrtest")
        # runner.move()

        # runner.configure_motion(demo_name="smoothyrtestreverse")        
        # runner.move()

    elif robot=="yumi_l":
        runner = ProgramRunner(robot="yumi_l")
        runner.set_motion_mode("dmp")
        runner.configure_motion(duration_scale=duration_scale)
        runner.set_camera() 


        ###################
        runner.gripper.moveto("10")
        runner.configure_motion(demo_name="smoothylhometoscrewupdate")
        runner.locate_target("screw")
        runner.move(debug=True)
        runner.gripper.grasp("close")
        ###################

        ##################
        runner.configure_motion(demo_name="smoothylscrewmountupdate")        
        runner.move(debug=True)
        runner.gripper.grasp("open")
        rospy.sleep(0.5)
        runner.gripper.grasp("close")
        ###################

        ###################
        runner.configure_motion(demo_name="smoothylscrewtoringupdate")
        runner.locate_target("ring")        
        runner.move(debug=True)
        runner.gripper.moveto("1.5")
        ###################


        # ###################
        # runner.configure_motion(demo_name="smoothylscrewtoringupdatereverse")        
        # runner.move(debug=True)
        # ###################

        ###################
        runner.configure_motion(demo_name="smoothylringmountupdate")        
        runner.move(debug=True)
        ###################


        # ###################
        # runner.gripper.grasp("close")
        # runner.configure_motion(demo_name="smoothylringmountupdatereverse")
        # runner.locate_target("ring")
        # runner.move(debug=True)
        # runner.gripper.moveto("1.5")
        # ###################

        # ###################
        # runner.configure_motion(demo_name="smoothylscrewmountupdatereverse")        
        # runner.move(debug=True)
        # ###################

        # ###################
        # runner.configure_motion(demo_name="smoothylhometoscrewupdatereverse")        
        # runner.move(debug=True)
        # ###################


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