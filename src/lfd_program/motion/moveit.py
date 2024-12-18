
from lfd_program.core.motion import MotionProgram


class MoveitProgram(MotionProgram):
    def __init__(self, robot_ns) -> None:
        super().__init__(robot_ns)
    
    def configure(self, **kwargs):
        self.default_joint_target = kwargs.get("joint_target", None)
        self.default_pose_target = kwargs.get("pose_target", None)
    
    def run(self, debug):
        raise NotImplementedError()


# class MoveitProgram(object):

#     def __init__(self,robot_ns):
#         self.ac_planjoint = actionlib.SimpleActionClient(f"{robot_ns}/plan_joint", PlanJointAction)
#         self.ac_planpose = actionlib.SimpleActionClient(f"{robot_ns}/plan_pose", PlanPoseAction)

#         self.ac_planjoint.wait_for_server()
#         self.ac_planpose.wait_for_server()

#     def plan_joint(self, joint_position : JointTrajectoryPoint):
#         goal_planjoint = PlanJointGoal(joint_position=joint_position)
#         self.ac_planjoint.send_goal(goal_planjoint)
#         self.ac_planjoint.wait_for_result()
#         return self.ac_planjoint.get_result().success

#     def plan_pose(self, pose: Pose, q_init: JointTrajectoryPoint):
#         goal_planpose = PlanPoseGoal(pose=pose, q_init=q_init)
#         self.ac_planpose.send_goal(goal_planpose)
#         self.ac_planpose.wait_for_result()
#         return self.ac_planpose.get_result().success