

import rospy
import actionlib

from franka_gripper.msg import MoveAction, MoveGoal, GraspAction, GraspGoal

class FrankaProgram(object):

    def __init__(self):
        self.ac_move = actionlib.SimpleActionClient('/franka_gripper/gripper_action', MoveAction)
        self.ac_grasp = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)
        self.ac_move.wait_for_server()

    def gripper_open(self, width=0.08, speed=0.1):
        goal = MoveGoal()
        goal.width = width
        goal.speed = speed
        
        self.ac_move.send_goal(goal)
        self.ac_move.wait_for_result()
        result = self.ac_move.get_result()

        if result.success:
            rospy.loginfo('Move Action succeeded!')
        else:
            rospy.logerr('Move Action failed with error: %s', result.error)
    
    def gripper_grasp(self, width=0.004, epsilon=0.04, speed=0.1, force=10):
        goal = GraspGoal()
        goal.width = width
        goal.epsilon.inner = epsilon
        goal.epsilon.outer = epsilon
        goal.speed = speed
        goal.force = force

        self.ac_grasp.send_goal(goal)
        self.ac_grasp.wait_for_result()
        result = self.ac_grasp.get_result()

        if result.success:
            rospy.loginfo('Grasp Action succeeded!')
        else:
            rospy.logerr('Grasp Action failed with error: %s', result.error)    