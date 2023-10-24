import rospy
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.srv import GetPositionFKRequest
from moveit_msgs.srv import GetPositionFKResponse
from sensor_msgs.msg import JointState

from trajectory_msgs.msg import JointTrajectoryPoint


class FK(object):
    def __init__(self, fk_link, frame_id):
        """
        A class to do FK calls thru the MoveIt!'s /compute_ik service.

        :param str fk_link: link to compute the forward kinematics
        :param str frame_id: frame_id to compute the forward kinematics
        """
        self.fk_link = fk_link
        self.frame_id = frame_id
        self.fk_srv = rospy.ServiceProxy('/compute_fk', GetPositionFK)
        self.fk_srv.wait_for_service()
        self.last_js = None
        self.js_sub = rospy.Subscriber('/joint_states',
                                       JointState,
                                       self.js_cb,
                                       queue_size=1)

    def js_cb(self, data):
        self.last_js = data

    def get_current_fk_pose(self):
        resp = self.get_current_fk()
        if len(resp.pose_stamped) >= 1:
            return resp.pose_stamped[0]
        return None

    def get_current_fk(self):
        while not rospy.is_shutdown() and self.last_js is None:
            rospy.sleep(0.1)
        return self.get_fk(self.last_js)

    def get_fk(self, joint_state, fk_link=None, frame_id=None):
        """
        Do an FK call to with.

        :param sensor_msgs/JointState joint_state: JointState message
            containing the full state of the robot.
        :param str or None fk_link: link to compute the forward kinematics for.
        """
        if fk_link is None:
            fk_link = self.fk_link
        if frame_id is None:
            frame_id = self.frame_id

        req = GetPositionFKRequest()
        req.header.frame_id = frame_id
        req.fk_link_names = [fk_link]
        req.robot_state.joint_state = joint_state
        try:
            resp = self.fk_srv.call(req)
            return resp
        except rospy.ServiceException as e:
            rospy.logerr("Service exception: " + str(e))
            resp = GetPositionFKResponse()
            resp.error_code = 99999  # Failure
            return resp
        
    def get_pose(self, joint_position: JointTrajectoryPoint):
        while not rospy.is_shutdown() and self.last_js is None:
            rospy.sleep(0.1)
        joint_state = self.last_js
        num_j = len(joint_position.positions)
        joint_state.position = joint_position.positions + joint_state.position[num_j:]
        resp = self.get_fk(joint_state)
        if len(resp.pose_stamped) >= 1:
            return resp.pose_stamped[0]
        return None
