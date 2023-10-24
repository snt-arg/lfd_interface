

import rospy

from controller_manager_msgs.srv import SwitchController

from abb_rapid_msgs.msg import *
from abb_robot_msgs.srv import *
from abb_robot_msgs.msg import *
from abb_rapid_sm_addin_msgs.srv import *
from abb_rapid_sm_addin_msgs.msg import RuntimeState


class RapidRunner():

    def __init__(self):
        self.is_running = False
        self.sub_sm_state = rospy.Subscriber("/yumi/rws/sm_addin/runtime_states",RuntimeState, self.cb_check_sm_state)
        self.rate = rospy.Rate(50)

    def run_rapid(self, l_routine = None, r_routine = None):
        """
        Function: run_rapid, to set and run RAPID Routine.
        """
        rospy.wait_for_service('/yumi/rws/sm_addin/set_rapid_routine')
        rospy.wait_for_service('/yumi/rws/sm_addin/run_rapid_routine')
        #
        set_rapid_routine = rospy.ServiceProxy('/yumi/rws/sm_addin/set_rapid_routine', SetRAPIDRoutine)
        run_rapid_routine = rospy.ServiceProxy('/yumi/rws/sm_addin/run_rapid_routine', TriggerWithResultCode)
        #
        if l_routine is not None:
            set_rapid_routine(task="T_ROB_L", routine=l_routine)
        if r_routine is not None:
            set_rapid_routine(task="T_ROB_R", routine=r_routine)
        rospy.sleep(0.5)
        #
        if (l_routine or r_routine) is not None:
            run_rapid_routine()
        rospy.sleep(0.5)
        while (self.is_running and not rospy.is_shutdown()):
            self.rate.sleep()

    def cb_check_sm_state(self, msg: RuntimeState):
        self.is_running = any([sm.sm_state !=2 for sm in msg.state_machines])


class EGMRunner():

    def __init__(self):
        pass

    def activate_egm(self):

        rospy.wait_for_service("/yumi/rws/sm_addin/start_egm_joint")
        rospy.wait_for_service("/yumi/egm/controller_manager/switch_controller")

        s_start_egm_joint = rospy.ServiceProxy("/yumi/rws/sm_addin/start_egm_joint", TriggerWithResultCode)
        s_switch_controller = rospy.ServiceProxy("/yumi/egm/controller_manager/switch_controller", SwitchController)

        s_start_egm_joint()
        rospy.sleep(0.5)
        s_switch_controller(start_controllers=["joint_trajectory_controller"], stop_controllers=[""], strictness=1, start_asap=False, timeout=0)

    def deactivate_egm(self):

        rospy.wait_for_service("/yumi/rws/sm_addin/stop_egm")
        s_stop_egm_joint = rospy.ServiceProxy("/yumi/rws/sm_addin/stop_egm", TriggerWithResultCode)
        s_stop_egm_joint()
