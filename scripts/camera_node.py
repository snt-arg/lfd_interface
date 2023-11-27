#!/usr/bin/env python3

import rospy
import actionlib

from lfd_camera.calib import Calibration
from lfd_camera.cognex import Cognex

from lfd_interface.msg import CameraAction, CameraGoal, CameraResult

class CameraActionServer:
    _result = CameraResult()

    def __init__(self, name, cam, calib):
        self.action_name = name
        self.cam = cam
        self.calib = calib
        self._as = actionlib.SimpleActionServer(self.action_name, 
                                                CameraAction, execute_cb=self.execute_cb, 
                                                auto_start = False)
        self.cam.connect()
        self._as.start()
    

    def execute_cb(self, goal : CameraGoal):
        # output = b'Welcome to In-Sight(tm)  8502P Session 0\r\nUser: Password: User Logged In\r\n1\r\n1\r\n(28.9,24.2) 153.2\xb0 score = 70.2\r\n'
        # coord = cam._extract_pos(output)
        coord = self.cam.read(goal.name)
        pose = self.calib.transform(coord,goal.pose_template)

        self._result.success = True
        self._result.pose = pose
        self._as.set_succeeded(self._result)

if __name__ == "__main__":

    rospy.init_node("lfd_camera", anonymous=False)
    
    output = b'Welcome to In-Sight(tm)  8502P Session 0\r\nUser: Password: User Logged In\r\n1\r\n1\r\n(28.9,24.2) 153.2\xb0 score = 70.2\r\n'
    cam = Cognex()
    calib = Calibration()

    camera_action_server = CameraActionServer("camera_action", cam, calib)

    # print(calib.transform(cam._extract_pos(output)))
    # print(calib.transform([0.0408,0.0338]))
    # print(cam._extract_pos(output))

    rospy.spin() 