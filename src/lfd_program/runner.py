

from lfd_program.robot.franka import FR3
from lfd_program.robot.abb import YumiL, YumiR
from lfd_program.motion.dmp import DMPProgram
from lfd_program.motion.moveit import MoveitProgram
from lfd_program.core.camera import CameraProgram



class ProgramRunner:

    def __init__(self, robot) -> None:
        self.robot_ns = robot
        if robot == "yumi_l":
            self.robot_program = YumiL()
        if robot == "yumi_r":
            self.robot_program = YumiR()
        if robot == "fr3":
            self.robot_program = FR3()
            
        self.gripper = self.robot_program.gripper
        self.available_motion_modes = ["moveit", "dmp"]
    

    def set_camera(self, camera = None):
        self.camera_program = CameraProgram(camera)
    

    def set_motion_mode(self, motion_mode):
        if motion_mode not in self.available_motion_modes:
            raise ValueError("Invalid motion mode") 
        self.motion_mode = motion_mode

        if self.motion_mode == "moveit":
            self.motion_program = MoveitProgram(self.robot_ns)
        elif self.motion_mode == "dmp":
            self.motion_program = DMPProgram(self.robot_ns)


    def configure_motion(self, **kwargs):
            self.motion_program.configure(**kwargs)


    def move(self, debug=False):
        self.robot_program.move(self.motion_program, debug)


    def locate_target(self, alias):
        joint_target = self.camera_program.locate_target(alias, 
                                                   self.motion_program, 
                                                   self.robot_program)
        self.motion_program.set_target(joint_target)
  






    
#     def _move_dmp(self, target : str, demo_name : str):
#         """
#         move the robot via dmp trained by the requested demo
#         """
#         if demo_name not in self.dmps:
#             self._dmp_train(demo_name)
        
#         dmp = self.dmps[demo_name]

#         if target is not None:
#             pos = dmp.demo_goal_joint()
#             pose = self.fk.get_pose(pos)
            
#             goal_pos = self._get_camera_pos(target, pose, pos)
#             self.robot.move(dmp.execute, goal_joint=goal_pos, duration_scale=self.duration_scale)
#         else:
#             self.robot.move(dmp.execute, duration_scale=self.duration_scale)

#     def _get_camera_pos(self, job_name : str, pose_template : Pose, pos_init: JointState):
#         pose = self.camera.trigger(job_name, pose_template)
#         pos = self.ik.request_ik(pose, JointTrajectoryPoint(positions=pos_init.position))
#         return pos
