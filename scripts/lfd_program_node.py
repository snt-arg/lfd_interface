#!/usr/bin/env python3

import rospy

from lfd_program.dmp import DMPProgram
from lfd_program.moveit import MoveitProgram
from lfd_program.camera import CameraProgram

from trajectory_msgs.msg import JointTrajectoryPoint

from lfd_program.util.fk import FK

def test_dmp_program():
    dmp_prog = DMPProgram("smoothpicknplace")
    dmp_prog.train()
    rospy.loginfo(dmp_prog.demo_goal_joint())
    dmp_prog.visualize()

def test_fk():
    fk = FK("fr3_hand_tcp", "fr3_link0")
    jtp = JointTrajectoryPoint()
    rospy.sleep(1)
    jtp.positions = fk.last_js.position
    print(fk.get_pose(jtp))

def get_sample_joint_pose():
    dmp_prog = DMPProgram("smoothpicknplace")
    joint = dmp_prog.demo_goal_joint()
    fk = FK("fr3_hand_tcp", "fr3_link0")
    pose = fk.get_pose(joint)
    return joint,pose.pose    

def test_moveit_program():
    joint,pose = get_sample_joint_pose()
    moveit_prog = MoveitProgram()
    # moveit_prog.plan_joint(joint)
    moveit_prog.plan_pose(pose.pose,joint)

def test_camera_program():
    joint, pose = get_sample_joint_pose()
    camera_prog = CameraProgram()
    print(camera_prog.trigger("nomatter", pose))

if __name__ == "__main__":

    rospy.init_node("lfd_program", anonymous=False)

    # test_moveit_program()
    # test_dmp_program()
    # test_fk()
    test_camera_program()

    rospy.spin() 