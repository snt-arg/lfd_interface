
from fileinput import filename
from logging import exception
import os
import rospy
import pickle
import re

from lfd_interface.msg import DemonstrationMsg
from lfd_interface.srv import GetDemonstration, GetDemonstrationRequest, GetDemonstrationResponse, DemoCount, DemoCountResponse


class DemonstrationStorage(object):

    def __init__(self):
        self.dir_template = "demonstrations/{}"
        self.filename_template = "demonstrations/{}/{}.pickle"
        rospy.Subscriber("save_demonstration", DemonstrationMsg, self.subcb_save_trajectory)
        self.service = rospy.Service('get_demonstration', GetDemonstration, self.servicecb_get_demonstration)
        self.service = rospy.Service('fetch_demo_count', DemoCount, self.servicecb_demo_count)

    def servicecb_demo_count(self, msg):
        dir = self.dir_template.format(msg.name)
        if not os.path.exists(dir):
            return DemoCountResponse(count=0)
        
        count = len([name for name in os.listdir(dir) if os.path.isfile(os.path.join(dir, name))])
        return DemoCountResponse(count=count)


    def format_filename(self, filename):
        # Split text (demo base name) and number (demo id) from  the original message
        temp = re.compile("([a-zA-Z]+)([0-9]+)")
        name = temp.match(filename).groups()

        # Check if the directory exists
        dir = self.dir_template.format(name[0])
        if not os.path.exists(dir):
            os.makedirs(dir)

        return self.filename_template.format(name[0],name[1])


    def subcb_save_trajectory(self, msg : DemonstrationMsg):
        filename = self.format_filename(msg.name) 
        try:

            with open(filename, 'wb') as file:
                pickle.dump(msg, file)
    
            rospy.logdebug("joint trajectory saved successfully at {}".format(os.getcwd() + filename))
        except OSError as exception:
            rospy.logerr(exception)

    def servicecb_get_demonstration(self, req : GetDemonstrationRequest):
        filename = self.format_filename(req.name)
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

