#!/usr/bin/env python3

import os
import rospy
import pickle
import numpy as np

from lfd_interface.msg import DemonstrationMsg
from lfd_interface.srv import GetDemonstration, DemoCount



def cut_after_t_seconds(demonstration, t):
    for i, point in enumerate(demonstration.joint_trajectory.points):
        if point.time_from_start.to_sec() >= t:
            demonstration.joint_trajectory.points = demonstration.joint_trajectory.points[:i]
            break

    return demonstration

def scale_demonstration(demonstration, scale):
    for i, point in enumerate(demonstration.joint_trajectory.points):
        new_time = point.time_from_start.to_sec() * scale
        point.time_from_start = rospy.Duration.from_sec(new_time)

    return demonstration  

def fix_joint_pose_count(demonstration, pose_offset=0):
    len_joint = len(demonstration.joint_trajectory.points)
    len_pose = len(demonstration.pose_trajectory.points)
    print(len_joint)
    print(len_pose)
    if len_joint<len_pose:
        demonstration.pose_trajectory.points = demonstration.pose_trajectory.points[pose_offset:]
    
    return len_joint, demonstration


def crop_and_scale(demonstration, t=10, scale=0.2):
    demonstration = cut_after_t_seconds(demonstration, t)
    demonstration = scale_demonstration(demonstration, scale)
    demonstration.name = "filter" + demonstration.name
    return demonstration

if __name__ == '__main__':

    rospy.init_node('demonstration_filter_node')
    
    demo_name = rospy.get_param("~demo_name")
    crop_time = rospy.get_param("~crop_time")
    scaling_factor = rospy.get_param("~scaling_factor")

    rospy.wait_for_service("get_demonstration")
    sc_lfd_storage = rospy.ServiceProxy("get_demonstration", GetDemonstration)
    pub_save_demo = rospy.Publisher("save_demonstration", DemonstrationMsg , queue_size=1)

    resp = sc_lfd_storage(name=demo_name)
    demonstration = resp.Demonstration
    demonstration = crop_and_scale(demonstration, float(crop_time), float(scaling_factor))
    rospy.sleep(2)
    pub_save_demo.publish(demonstration)
    print("published")



    # resp = sc_lfd_storage(name=demo_name + "0")
    # demonstration = resp.Demonstration
    # len_joint, _ = fix_joint_pose_count(demonstration)
    # pub_save_demo.publish(demonstration)

    # resp = sc_lfd_storage(name=demo_name + "1")
    # demonstration = resp.Demonstration
    # _, demonstration = fix_joint_pose_count(demonstration, len_joint)
    # pub_save_demo.publish(demonstration)

    
    rospy.spin()



    