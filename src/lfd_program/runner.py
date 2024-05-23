

from lfd_program.robot.franka import FrankaProgram
from lfd_program.robot.abb import YumiProgram
from lfd_program.core.dmp import DMPProgram
from lfd_program.core.moveit import MoveitProgram
from lfd_program.core.camera import CameraProgram

from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState

from lfd_program.util.kinematics import FK, IK


class ProgramRunner:

    def __init__(self, robot_type = "fr3", camera=True, duration_scale = 1) -> None:
        
        if camera:
            self.camera = CameraProgram()
        
        self.duration_scale = duration_scale
        self.moveit = MoveitProgram()
        
        if robot_type == "fr3":
            self.fk = FK("fr3_hand_tcp", "fr3_link0")
            self.ik = IK()
            self.robot = FrankaProgram()

        elif robot_type == "yumi_l":
            self.fk = FK("yumi_link_7_l", "yumi_base_link")
            self.ik = IK()
            self.robot = YumiProgram()
            
        elif robot_type == "yumi_r":
            self.fk = FK("yumi_link_7_r", "yumi_base_link")
            self.ik = IK()
            self.robot = YumiProgram()
        
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
            self.robot.move(dmp.visualize, goal_joint=goal_pos, duration_scale=self.duration_scale)
        else:
            self.robot.move(dmp.visualize, duration_scale=self.duration_scale)

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
    
