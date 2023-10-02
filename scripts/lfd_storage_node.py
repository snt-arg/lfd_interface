#!/usr/bin/env python3


from fileinput import filename
from logging import exception
import os
import rospy

from lfd_storage.demonstration_storage import DemonstrationStorage


if __name__ == "__main__":

    rospy.init_node("lfd_storage", anonymous=False)

    os.chdir(rospy.get_param("~working_dir"))
    
    demostorage_manager = DemonstrationStorage()

    rospy.spin() 