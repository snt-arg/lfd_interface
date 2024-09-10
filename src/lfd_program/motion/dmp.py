
import rospy
import actionlib

from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState

from lfd_interface.msg import LFDPipelineAction, LFDPipelineGoal
from lfd_interface.srv import GetDemonstration

from lfd_program.core.motion import MotionProgram

class DMPProgram(MotionProgram):
    def __init__(self, robot_ns) -> None:
        super().__init__(robot_ns)
        self.a_client = actionlib.SimpleActionClient(f'{robot_ns}/lfd_pipeline', LFDPipelineAction)
        self.a_client.wait_for_server()

        self.dmps = []
        self.demo_name = None
        self.default_joint_target = None
        self.duration_scale = 0


    def configure(self, **kwargs):
        if "demo_name" in kwargs:
            self.demo_name = kwargs.get("demo_name")
            self.default_joint_target = kwargs.get("default_joint_target", None)
            
            if self.default_joint_target is None:
                self.default_joint_target = self.demo_goal_joint()
            
            if self.demo_name not in self.dmps:
                self.train()
                self.dmps.append(self.demo_name)

        if "default_joint_target" in kwargs:
            self.default_joint_target = kwargs.get("default_joint_target", None)
            if self.default_joint_target is None:
                self.default_joint_target = self.demo_goal_joint()
        if "duration_scale" in kwargs:
            self.duration_scale = kwargs.get("duration_scale",0)
        
        
    def train(self):
        goal = LFDPipelineGoal(name=self.demo_name, train=True)
        self.a_client.send_goal(goal)
        self.a_client.wait_for_result()
    
    def visualize(self, goal_joint = JointTrajectoryPoint()):
        goal = LFDPipelineGoal(name=self.demo_name, visualize=True,
                               goal_joint=goal_joint, duration=self.duration_scale)
        self.a_client.send_goal(goal)
        self.a_client.wait_for_result()
        result = self.a_client.get_result()
        return result.plan

    def execute(self, goal_joint = JointTrajectoryPoint()):
        goal = LFDPipelineGoal(name=self.demo_name, execute=True,
                               goal_joint=goal_joint, duration=self.duration_scale)
        self.a_client.send_goal(goal)
        self.a_client.wait_for_result()
        result = self.a_client.get_result()
        return result.plan

    def _fetch_demo(self):
        demo_name = self.demo_name + "0"
        s = rospy.ServiceProxy('get_demonstration', GetDemonstration)
        resp = s(demo_name)
        return resp.Demonstration

    def demo_goal_joint(self):
        goal_state = JointState()
        demonstration = self._fetch_demo()
        goal_state.name = demonstration.joint_trajectory.joint_names
        goal_state.position = demonstration.joint_trajectory.points[-1].positions
        return goal_state
    
    def run(self, debug):
        if not self.joint_target:
            target = self.default_joint_target
        else:
            target = self.joint_target

        target = self.joint_state_to_trajectory_point(target)
        
        if debug:
            self.visualize(target)
            
        return self.execute(target)