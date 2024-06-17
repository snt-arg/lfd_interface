import rospy

from controller_manager_msgs.srv import SwitchController

from abb_rapid_msgs.msg import *
from abb_robot_msgs.srv import *
from abb_robot_msgs.msg import *
from abb_rapid_sm_addin_msgs.srv import *
from abb_rapid_sm_addin_msgs.msg import RuntimeState, StateMachineState

from lfd_program.core.robot import RobotProgram

class StateMachineRunner():

    def __init__(self):
        rospy.wait_for_service("/yumi/rws/sm_addin/start_egm_joint")
        rospy.wait_for_service("/yumi/egm/controller_manager/switch_controller")
        rospy.wait_for_service("/yumi/rws/sm_addin/stop_egm")
        rospy.wait_for_service('/yumi/rws/sm_addin/set_rapid_routine')
        rospy.wait_for_service('/yumi/rws/sm_addin/run_rapid_routine')
        rospy.wait_for_service("/yumi/rws/sm_addin/set_sg_command")
        rospy.wait_for_service("/yumi/rws/sm_addin/run_sg_routine")

        self.rapid_running = False
        self.egm_running = False

        self.sub_sm_state = rospy.Subscriber("/yumi/rws/sm_addin/runtime_states",RuntimeState, self.cb_check_sm_state)
        self.rate = rospy.Rate(50)

    def run_rapid(self, l_routine = None, r_routine = None, nonblocking = False):
        """
        Function: run_rapid, to set and run RAPID Routine.
        """
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

        if not nonblocking:
            while (self.rapid_running and not rospy.is_shutdown()):
                self.rate.sleep()

    def activate_egm(self):
        s_start_egm_joint = rospy.ServiceProxy("/yumi/rws/sm_addin/start_egm_joint", TriggerWithResultCode)
        s_switch_controller = rospy.ServiceProxy("/yumi/egm/controller_manager/switch_controller", SwitchController)

        s_start_egm_joint()
        rospy.sleep(0.5)
        s_switch_controller(start_controllers=["joint_trajectory_controller"], stop_controllers=[""], strictness=1, start_asap=False, timeout=0)

    def deactivate_egm(self):
        s_stop_egm_joint = rospy.ServiceProxy("/yumi/rws/sm_addin/stop_egm", TriggerWithResultCode)
        s_stop_egm_joint()
    
    def set_sg_command(self, task, command, target_pos = 0.0):
        s_set_sg_command = rospy.ServiceProxy("/yumi/rws/sm_addin/set_sg_command", SetSGCommand)
        s_set_sg_command(task=task, command=command, target_position=target_pos)
    
    def run_sg_routine(self):
        s_run_sg_routine = rospy.ServiceProxy("/yumi/rws/sm_addin/run_sg_routine", TriggerWithResultCode)
        s_run_sg_routine()

    def cb_check_sm_state(self, msg: RuntimeState):
        self.rapid_running = any([sm.sm_state == StateMachineState.SM_STATE_RUN_RAPID_ROUTINE for sm in msg.state_machines])
        self.egm_running = any([sm.sm_state == StateMachineState.SM_STATE_RUN_EGM_ROUTINE for sm in msg.state_machines])


class YumiProgram(RobotProgram):

    def __init__(self, robot_arm = "yumi_l"):
        if robot_arm == "yumi_l":
            robot_arm = "T_ROB_L"
        elif robot_arm == "yumi_r":
            robot_arm = "T_ROB_R"

        self.robot_arm = robot_arm        
        self.sm_runner = StateMachineRunner()

    def gripper_moveto(self, arg):
        self.sm_runner.set_sg_command(task=self.robot_arm, command=SetSGCommandRequest.SG_COMMAND_MOVE_TO, target_pos=arg)
        rospy.sleep(0.5)
        self.sm_runner.run_sg_routine()

    def gripper_grasp(self, arg):
        print(f"Running gripper grasp {arg}")
        if arg == "open":
            self.sm_runner.set_sg_command(task=self.robot_arm, command=SetSGCommandRequest.SG_COMMAND_GRIP_OUT)
        elif arg == "close":
            self.sm_runner.set_sg_command(task=self.robot_arm, command=SetSGCommandRequest.SG_COMMAND_GRIP_IN)
        rospy.sleep(0.5)
        self.sm_runner.run_sg_routine()
    
    def gripper(self, command, arg):
        if command == "moveto":
            self.gripper_moveto(arg)
        elif command == "grasp":
            self.gripper_grasp(arg)

    def move(self, move_method, *args, **kwargs):
        if not self.sm_runner.egm_running:
            self.sm_runner.activate_egm()
        # self.egm_runner.activate_egm()
        super().move(move_method, *args, **kwargs)
        # self.egm_runner.deactivate_egm()
