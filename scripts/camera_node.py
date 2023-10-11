#!/usr/bin/env python3

import rospy

from lfd_camera.calib import Calibration


if __name__ == "__main__":

    rospy.init_node("lfd_camera", anonymous=False)
    

    calib = Calibration()
    print(calib.transform([0.0408,0.0338]))


    rospy.spin() 