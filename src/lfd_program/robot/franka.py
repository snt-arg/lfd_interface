

import rospy
import actionlib

from lfd_program.core.robot import RobotProgram
from franka_gripper.msg import MoveAction, MoveGoal, GraspAction, GraspGoal
from lfd_program.util.kinematics import FK, IK


class FR3(RobotProgram):
    def __init__(self) -> None:
        self.fk = FK("fr3_hand_tcp", "fr3_link0")
        self.ik = IK("fr3")
        self.ac_move = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)
        self.ac_grasp = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)
        self.ac_move.wait_for_server()
        self.gripper = FrankaGripper()
    
    def move(self, motion_program, debug=False):
        motion_program.run(debug)

class FrankaGripper:

    def __init__(self) -> None:
        self.ac_move = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)
        self.ac_grasp = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)
        self.ac_move.wait_for_server()
    
    def moveto(self, target_pos):
        pass
    
    def grasp(self, command):
        if command == "open":
            pass
        elif command == "close":
            pass
    
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

