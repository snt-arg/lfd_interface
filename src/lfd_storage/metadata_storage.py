

import rospy
import yaml

from std_msgs.msg import String
from sensor_msgs.msg import JointState


class MetaDataStorage(object):

    def __init__(self):
        self.stored_pos_file = "metadata/stored_pos.yaml"
        rospy.Subscriber("store_pos_named", String, self.subcb_store_pos)
        rospy.Subscriber("/joint_states", JointState, self.subcb_joint_states)
        self.joint_states = JointState()

    def subcb_joint_states(self, msg: JointState):
        self.joint_states = msg

    def subcb_store_pos (self, msg: String):
        with open(self.stored_pos_file, 'r') as f:
            data = yaml.safe_load(f)    
        if data is None:
            data = {}
        data[msg.data] = list(self.joint_states.position)

        with open(self.stored_pos_file, 'w') as f:
            yaml.dump(data, f)        
                

