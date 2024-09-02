import rospy
import math

import numpy as np

from controller_manager_msgs.srv import SwitchController
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState

from abb_rapid_msgs.msg import *
from abb_robot_msgs.srv import *
from abb_robot_msgs.msg import *
from abb_rapid_sm_addin_msgs.srv import *
from abb_rapid_sm_addin_msgs.msg import RuntimeState, StateMachineState

from lfd_program.core.robot import RobotProgram

zone_data = {
    "z0": [0.3, 0.3, 0.3, 0.03, 0.3, 0.03],
    "z1": [1, 1, 1, 0.1, 1, 0.1],
    "z5": [5, 8, 8, 0.8, 8, 0.8],
    "z10": [10, 15, 15, 1.5, 15, 1.5],
    "z15": [15, 23, 23, 2.3, 23, 2.3],
    "z20": [20, 30, 30, 3.0, 30, 3.0],
    "z30": [30, 45, 45, 4.5, 45, 4.5],
    "z40": [40, 60, 60, 6.0, 60, 6.0],
    "z50": [50, 75, 75, 7.5, 75, 7.5],
    "z60": [60, 90, 90, 9.0, 90, 9.0],
    "z80": [80, 120, 120, 12, 120, 12],
    "z100": [100, 150, 150, 15, 150, 15],
    "z150": [150, 225, 225, 23, 225, 23],
    "z200": [200, 300, 300, 30, 300, 30]
}

speed_data = {
    "v5": [5, 500, 5000, 1000],
    "v10": [10, 500, 5000, 1000],
    "v20": [20, 500, 5000, 1000],
    "v30": [30, 500, 5000, 1000],
    "v40": [40, 500, 5000, 1000],
    "v50": [50, 500, 5000, 1000],
    "v60": [60, 500, 5000, 1000],
    "v80": [80, 500, 5000, 1000],
    "v100": [100, 500, 5000, 1000],
    "v150": [150, 500, 5000, 1000],
    "v200": [200, 500, 5000, 1000],
    "v300": [300, 500, 5000, 1000],
    "v400": [400, 500, 5000, 1000],
    "v500": [500, 500, 5000, 1000],
    "v600": [600, 500, 5000, 1000],
    "v800": [800, 500, 5000, 1000],
    "v1000": [1000, 500, 5000, 1000],
    "v1500": [1500, 500, 5000, 1000],
    "v2000": [2000, 500, 5000, 1000],
    "v2500": [2500, 500, 5000, 1000],
    "v3000": [3000, 500, 5000, 1000],
    "v4000": [4000, 500, 5000, 1000]
}

class FormatTrajectory():

    def __init__(self, fk, num_points = 15):
        self.num_points = num_points
        self.fk = fk

    def segment(self, joint_trajectory_msg):
        trajectory_length = len(joint_trajectory_msg.points)
        
        if trajectory_length <= self.num_points:
            return joint_trajectory_msg
        
        indices = np.linspace(0, trajectory_length - 1, num=self.num_points, dtype=int)
        indices = indices[1:]
        
        sampled_trajectory_msg = JointTrajectory()
        
        sampled_trajectory_msg.joint_names = joint_trajectory_msg.joint_names
        
        for i in indices:
            sampled_trajectory_msg.points.append(joint_trajectory_msg.points[i])
        
        return sampled_trajectory_msg

    def move_third_to_end(self, lst):
        lst = list(lst)
        if len(lst) >= 3:
            third_element = lst.pop(2)
            lst.append(third_element)
        return lst

    def radians_to_degrees(self, radians):
        degrees_list = [radian * (180 / math.pi) for radian in radians]
        return degrees_list

    def format(self, joint_trajectory_msg):
        global speed_data
        joint_trajectory_msg = self.segment(joint_trajectory_msg)

        speeds = self.extract_velocities(joint_trajectory_msg)
        print(speeds)
        trajectory_str = "joint targets:\n"
        
        for i, point in enumerate(joint_trajectory_msg.points):
            positions = self.move_third_to_end(point.positions)
            positions = self.radians_to_degrees(positions)

            positions_str1 = ','.join([f"{pos:.4f}" for pos in positions[:-1]])
            positions_str1 = f"[{positions_str1}]\n"
            positions_str2 = f"[{positions[-1]:.4f}" + ",9E+09,9E+09,9E+09,9E+09,9E+09]\n"
            positions_str3 = f"{speed_data[speeds[i]]}\n"

            positions_str = positions_str1 + positions_str2 + positions_str3
            trajectory_str += positions_str
            
        
        return trajectory_str


    def extract_velocities(self, joint_trajectory):
        joint_names = joint_trajectory.joint_names
        speeds = []

        for i in range(len(joint_trajectory.points) - 1):
            joint_state_1 = JointState()
            joint_state_1.name = joint_names
            joint_state_1.position = joint_trajectory.points[i].positions
            joint_state_1.velocity = joint_trajectory.points[i].velocities if joint_trajectory.points[i].velocities else []
            joint_state_1.effort = joint_trajectory.points[i].effort if joint_trajectory.points[i].effort else []

            joint_state_2 = JointState()
            joint_state_2.name = joint_names
            joint_state_2.position = joint_trajectory.points[i + 1].positions
            joint_state_2.velocity = joint_trajectory.points[i + 1].velocities if joint_trajectory.points[i + 1].velocities else []
            joint_state_2.effort = joint_trajectory.points[i + 1].effort if joint_trajectory.points[i + 1].effort else []

            pose_1 = self.fk.get_pose(joint_state_1)
            pose_2 = self.fk.get_pose(joint_state_2)

            position_1 = [pose_1.position.x, pose_1.position.y, pose_1.position.z]
            position_2 = [pose_2.position.x, pose_2.position.y, pose_2.position.z]
            distance = np.linalg.norm(np.array(position_2) - np.array(position_1))

            time_diff = joint_trajectory.points[i + 1].time_from_start.to_sec() - joint_trajectory.points[i].time_from_start.to_sec()

            speed = distance * 1000 / time_diff if time_diff > 0 else 0

            speeds.append(speed)

        return self.categorize_velocities(speeds)


    def categorize_velocities(self, speeds):
        global speed_data
        categorized_speeds = []

        for speed in speeds:
            # Iterate over the predefined speed data
            for speed_zone, speed_value in speed_data.items():
                # Check if the speed falls within the speed zone
                if speed <= speed_value[0]:
                    # Append the speed and the speed zone to the categorized speeds list
                    categorized_speeds.append(speed_zone)
                    break
        
        categorized_speeds.append(categorized_speeds[-1])
        return categorized_speeds

class StateMachineRunner():

    def __init__(self):
        rospy.wait_for_service("/yumi/rws/sm_addin/start_egm_joint")
        rospy.wait_for_service("/yumi/egm/controller_manager/switch_controller")
        rospy.wait_for_service("/yumi/rws/sm_addin/stop_egm")
        rospy.wait_for_service('/yumi/rws/sm_addin/set_rapid_routine')
        rospy.wait_for_service('/yumi/rws/sm_addin/run_rapid_routine')
        rospy.wait_for_service("/yumi/rws/sm_addin/set_sg_command")
        rospy.wait_for_service("/yumi/rws/sm_addin/run_sg_routine")
        rospy.wait_for_service("/yumi/rws/set_file_contents")

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
        if self.egm_running:
            self.deactivate_egm()
            rospy.sleep(0.5)
            
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

    def set_file_contents(self, filename, contents):
        s_set_file_contents = rospy.ServiceProxy("/yumi/rws/set_file_contents", SetFileContents)
        s_set_file_contents(filename=filename, contents=contents)

class YumiProgram(RobotProgram):

    def __init__(self, robot_arm = "yumi_l", fk = None):
        if robot_arm == "yumi_l":
            robot_arm = "T_ROB_L"
        elif robot_arm == "yumi_r":
            robot_arm = "T_ROB_R"

        if fk is not None:
            self.fk = fk
            
        self.robot_arm = robot_arm        
        self.sm_runner = StateMachineRunner()

    def gripper_moveto(self, arg):
        self.sm_runner.set_sg_command(task=self.robot_arm, command=SetSGCommandRequest.SG_COMMAND_MOVE_TO, target_pos=float(arg))
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
        # if not self.sm_runner.egm_running:
        #     self.sm_runner.activate_egm()
        # self.egm_runner.activate_egm()
        # super().move(move_method, *args, **kwargs)
        # self.egm_runner.deactivate_egm()

        plan = move_method(*args, **kwargs)
        formatter = FormatTrajectory(num_points=25, fk=self.fk)
        content = formatter.format(plan)
        self.sm_runner.set_file_contents("joint_targets_l.txt", content)
        rospy.sleep(0.5)
        self.sm_runner.run_rapid(l_routine="execute", nonblocking=False)
