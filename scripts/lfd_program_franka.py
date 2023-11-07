#!/usr/bin/env python3

import rospy

from lfd_program.franka import FrankaProgram
from lfd_program.dmp import DMPProgram
from lfd_program.moveit import MoveitProgram
from lfd_program.camera import CameraProgram

from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState

from lfd_program.util.kinematics import FK, IK

class ProgramRunner:

    def __init__(self, robot_type = "fr3") -> None:
        
        self.camera = CameraProgram()
        self.moveit = MoveitProgram()
        
        if robot_type == "fr3":
            self.fk = FK("fr3_hand_tcp", "fr3_link0")
            self.ik = IK()
            self.robot = FrankaProgram()
        elif robot_type == "yumileft":
            pass
        elif robot_type == "yumiright":
            pass
        
        self.dmps = {}
    
    def move(self, target = None, mode = "moveit"):
        if mode == "moveit":
            self._move_moveit(target)
            # Use moveit to move to the target
            # target is a named preset joint position
            pass
        else:
            self._move_dmp(target, mode)
            # use a dmp routine to move
            # target refers to the camera-acquired position
            pass

    def _move_moveit(self, target : str):
        """
        move robot to a named target's associated pos via moveit
        """
        #TODO Later
        pos = self._get_named_target(target)

        pass
    
    def _get_named_target(self, name : str):
        """
        get the associated pos of a named target
        """
        #TODO Later
        pass
    
    def _move_dmp(self, target : str, demo_name : str):
        """
        move the robot via dmp trained by the requested demo
        """
        if demo_name not in self.dmps:
            self._dmp_train(demo_name)
        
        dmp = self.dmps[demo_name]

        if target is not None:
            pos = dmp.demo_goal_joint()
            pose = self.fk.get_pose(pos)
            
            goal_pos = self._get_camera_pos(target, pose, pos)
            dmp.visualize(goal_joint=goal_pos, duration_scale=10)
        else:
            dmp.visualize(duration_scale=10)

    def _get_camera_pos(self, job_name : str, pose_template : Pose, pos_init: JointState):
        pose = self.camera.trigger(job_name, pose_template)
        pos = self.ik.request_ik(pose, JointTrajectoryPoint(positions=pos_init.position))
        return pos

    def _dmp_train(self, demo_name : str):
        self.dmps[demo_name] = DMPProgram(demo_name)
        self.dmps[demo_name].train()

    def gripper(self, target : str, grasp = False):
        """
        target = open/close
        grasp = true if it needs to grasp sth,
                false if it needs to just move
        """
        if target == "open" and grasp:
            #TODO Code to open the gripper and grasp something
            pass
        elif target == "open" and not grasp:
            self.robot.gripper_open()
        elif target == "close" and grasp:
            self.robot.gripper_grasp()
        elif target == "close" and not grasp:
            #TODO Code to close the gripper without grasping
            pass
    


    

if __name__ == "__main__":

    rospy.init_node("lfd_program_franka", anonymous=False)

    # # initializations
    # fk = FK("fr3_hand_tcp", "fr3_link0")
    # ik = IK()
    # camera = CameraProgram()
    # moveit = MoveitProgram()
    # franka = FrankaProgram()



    # # Assume the robot is in home pos
    # dmp = DMPProgram("smoothpicknplaceee")
    # dmp.train()
    # franka.gripper_open()
    # goal_joint = dmp.demo_goal_joint()
    # goal_pose = fk.get_pose(goal_joint)
    # object_pose = camera.trigger("nomatter",goal_pose)
    # object_joint = ik.request_ik(object_pose, JointTrajectoryPoint(positions=goal_joint.position))
    # # object_joint = ik.request_ik(goal_pose, JointTrajectoryPoint(positions=[0,0,0,0,0,0,0]))
    # # moveit.plan_joint(object_joint)

    # dmp.execute(object_joint, 2)
    # franka.gripper_grasp()

    runner = ProgramRunner()
    runner.gripper("open")
    runner.move("nomatter", "smoothfrpick")
    runner.gripper("close", True)
    runner.move(None, "smoothfrplace")
    runner.gripper("open")


    # rospy.spin() 