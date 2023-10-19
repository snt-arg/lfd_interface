

import rospy
import actionlib

from lfd_interface.msg import LFDPipelineAction, LFDPipelineGoal

class DMPProgram(object):

    def __init__(self, name):
        self.name = name
        self.a_client = actionlib.SimpleActionClient('lfd_pipeline', LFDPipelineAction)
        self.a_client.wait_for_server()
    
    def train(self):
        goal = LFDPipelineGoal(name=self.name, train=True)
        self.a_client.send_goal(goal)
        self.a_client.wait_for_result()

    def visualize(self, duration_scale = 0):
        goal = LFDPipelineGoal(name=self.name, visualize=True, duration=duration_scale)
        self.a_client.send_goal(goal)
        self.a_client.wait_for_result()
    
    def execute(self, duration_scale=0):
        goal = LFDPipelineGoal(name=self.name, execute=True, duration=duration_scale)
        self.a_client.send_goal(goal)
        self.a_client.wait_for_result()
                