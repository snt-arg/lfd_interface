#!/usr/bin/env python3

import rospy
import actionlib

from lfd_camera.cognex import Cognex
from lfd_camera.util import CameraActionServer, CameraConfig

if __name__ == "__main__":

    rospy.init_node("lfd_camera", anonymous=False)

    cam_config = CameraConfig()
    cam = Cognex(cam_config)

    camera_action_server = CameraActionServer("camera_action", cam, cam_config.objects)

    # output = b'Welcome to In-Sight(tm)  8502P Session 0\r\nUser: Password: User Logged In\r\n1\r\n1\r\n(28.9,24.2) 153.2\xb0 score = 70.2\r\n'
    # print(calib.transform(cam._extract_pos(output)))
    # print(calib.transform([0.0408,0.0338]))
    # print(cam._extract_pos(output))

    rospy.spin() 