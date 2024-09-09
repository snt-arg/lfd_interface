
from trajectory_msgs.msg import JointTrajectoryPoint

class RobotProgram:
    def __init__(self) -> None:
        pass

    def pose_to_joint(self, pose_target, joint_init):
        return self.ik.request_ik(pose_target, JointTrajectoryPoint(positions=joint_init.position))
    
    def joint_to_pose(self, joint_target):
        return self.fk.get_pose(joint_target)