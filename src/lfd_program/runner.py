

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


class YumiRunner:

    def __init__(self):
        self.yumi_l_program = ProgramRunner("yumi_l")
        self.yumi_r_program = ProgramRunner("yumi_r")
        self.gripper_l = self.yumi_l_program.gripper
        self.gripper_r = self.yumi_r_program.gripper

    def set_motion_mode(self, motion_mode):
        self.yumi_l_program.set_motion_mode(motion_mode)
        self.yumi_r_program.set_motion_mode(motion_mode)
    
    def configure_motion(self, **kwargs):
        self.configure_l_motion(**kwargs)
        self.configure_r_motion(**kwargs)
    
    def configure_l_motion(self, **kwargs):
        self.yumi_l_program.configure_motion(**kwargs)
    
    def configure_r_motion(self, **kwargs):
        self.yumi_r_program.configure_motion(**kwargs)
    
    def locate_l_target(self, alias):
        self.yumi_l_program.locate_target(alias)
    
    def locate_r_target(self, alias):
        self.yumi_r_program.locate_target(alias)
    
    def move_l(self, debug=False):
        self.yumi_l_program.move(debug)
    
    def move_r(self, debug=False):
        self.yumi_r_program.move(debug)
    
    def move(self, debug=False):
        self.yumi_l_program.robot_program.move_generic(self.yumi_l_program.motion_program, 
                                                       debug)
        self.yumi_r_program.robot_program.move_generic(self.yumi_r_program.motion_program, 
                                                       debug)
        self.yumi_l_program.robot_program.execute_motion(r_routine="execute",
                                                        l_routine="execute")

