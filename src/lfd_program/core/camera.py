

import rospy
import actionlib

from lfd_interface.msg import CameraAction, CameraGoal, CameraResult


class CameraProgram:
    def __init__(self, camera_type = None) -> None:
        self.a_client = actionlib.SimpleActionClient('camera_action', CameraAction)
        self.a_client.wait_for_server()
    
    def locate_target(self, alias, motion_program, robot_program):
        joint_init = motion_program.default_joint_target
        pose_template = robot_program.joint_to_pose(joint_init)
        pose_target = self.trigger(alias, pose_template)
        return robot_program.pose_to_joint(pose_target, joint_init)

    def trigger(self, name, pose_template):
        goal = CameraGoal()
        goal.name = name
        goal.pose_template = pose_template
        self.a_client.send_goal(goal)

        finished = self.a_client.wait_for_result()
        if finished:
            result = self.a_client.get_result()
            if result.success:
                return result.pose
            else:
                return None
        else:
            return None


# class CameraProgram(object):

#     def __init__(self):
#         self.a_client = actionlib.SimpleActionClient('camera_action', CameraAction)
#         self.a_client.wait_for_server()

#     def trigger(self, name, pose_template):
#         goal = CameraGoal()
#         goal.name = name
#         goal.pose_template = pose_template
#         self.a_client.send_goal(goal)

#         finished = self.a_client.wait_for_result()
#         if finished:
#             result = self.a_client.get_result()
#             if result.success:
#                 return result.pose
#             else:
#                 return None
#         else:
#             return None
    