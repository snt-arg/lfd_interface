#!/usr/bin/env python3


from fileinput import filename
from logging import exception
import os
import rospy
import pickle

from lfd_interface.msg import DemonstrationMsg
from lfd_interface.srv import GetDemonstration, GetDemonstrationRequest, GetDemonstrationResponse


class DemonstrationStorage(object):

    def __init__(self):
        self.filename_template = "demonstrations/{}.pickle"
        rospy.Subscriber("save_demonstration", DemonstrationMsg, self.subcb_save_trajectory)
        self.service = rospy.Service('get_demonstration', GetDemonstration, self.servicecb_get_demonstration)


    def subcb_save_trajectory(self, msg : DemonstrationMsg):
        #print (msg)
        filename = self.filename_template.format(msg.name)

        try:

            with open(filename, 'wb') as file:
                pickle.dump(msg, file)
    
            rospy.logdebug("joint trajectory saved successfully at {}".format(os.getcwd() + filename))
        except OSError as exception:
            rospy.logerr(exception)

    def servicecb_get_demonstration(self, req : GetDemonstrationRequest):
        filename = self.filename_template.format(req.name)
        res = GetDemonstrationResponse()
        try:

            with open(filename, 'rb') as file:
                demonstration = pickle.load(file)

            res.Demonstration = demonstration
            res.success = True

        except OSError as exception:
            res.success = False
            rospy.logerr(exception)
    
        return res

        
if __name__ == "__main__":

    rospy.init_node("lfd_storage", anonymous=False)

    os.chdir(rospy.get_param("~working_dir"))
    
    demostorgae_manager = DemonstrationStorage()

    rospy.spin() 