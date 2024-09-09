
from trajectory_msgs.msg import JointTrajectoryPoint


class MotionProgram:
    def __init__(self, robot_ns) -> None:
        self.robot_ns = robot_ns
        self.joint_target = None

    def configure(self, **kwargs):
        raise NotImplementedError()
    
    def set_target(self, target):
        self.joint_target = target
    
    def joint_state_to_trajectory_point(self, joint_state):
        point = JointTrajectoryPoint()
        point.positions = joint_state.position
        return point
    