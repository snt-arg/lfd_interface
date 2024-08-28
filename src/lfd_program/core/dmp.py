
import rospy
import actionlib

from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState

from lfd_interface.msg import LFDPipelineAction, LFDPipelineGoal
from lfd_interface.srv import GetDemonstration

class DMPProgram(object):

    def __init__(self, name):
        self.name = name
        self.a_client = actionlib.SimpleActionClient('lfd_pipeline', LFDPipelineAction)
        self.a_client.wait_for_server()
    
    def train(self):
        goal = LFDPipelineGoal(name=self.name, train=True)
        self.a_client.send_goal(goal)
        self.a_client.wait_for_result()

    def visualize(self, goal_joint = JointTrajectoryPoint(),
                  duration_scale = 0):
        goal = LFDPipelineGoal(name=self.name, visualize=True,
                               goal_joint=goal_joint, duration=duration_scale)
        self.a_client.send_goal(goal)
        self.a_client.wait_for_result()
        result = self.a_client.get_result()
        return result.plan
    
    def execute(self, goal_joint = JointTrajectoryPoint(),
                  duration_scale = 0):
        goal = LFDPipelineGoal(name=self.name, execute=True,
                               goal_joint=goal_joint, duration=duration_scale)
        self.a_client.send_goal(goal)
        self.a_client.wait_for_result()

    def _fetch_demo(self):
        demo_name = self.name + "0"
        s = rospy.ServiceProxy('get_demonstration', GetDemonstration)
        resp = s(demo_name)
        return resp.Demonstration
    
    def demo_goal_joint(self):
        goal_state = JointState()
        demonstration = self._fetch_demo()
        goal_state.name = demonstration.joint_trajectory.joint_names
        goal_state.position = demonstration.joint_trajectory.points[-1].positions
        return goal_state
    