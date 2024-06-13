import rospy
import actionlib

from geometry_msgs.msg import Pose

from lfd_camera.calib import Calibration

from lfd_interface.msg import CameraAction, CameraGoal, CameraResult
import yaml

class CameraActionServer:
    _result = CameraResult()

    def __init__(self, name, cam, objects):
        self.action_name = name
        self.cam = cam
        self.objects = objects
        self._as = actionlib.SimpleActionServer(self.action_name, 
                                                CameraAction, execute_cb=self.execute_cb, 
                                                auto_start = False)
        self.cam.connect()
        self._as.start()
    

    def execute_cb(self, goal : CameraGoal):
        # output = b'Welcome to In-Sight(tm)  8502P Session 0\r\nUser: Password: User Logged In\r\n1\r\n1\r\n(28.9,24.2) 153.2\xb0 score = 70.2\r\n'
        # coord = self.cam._extract_pos(output)

        if goal.name != "" and goal.pose_template != Pose():
            coord = self.cam.read(goal.name)
            calib = self.objects[goal.name]["calib"]
            pose = calib.transform(coord,goal.pose_template)
        else:
            coord = self.cam.read(None)
            pose = Pose()
            pose.position.x = coord[0]
            pose.position.y = coord[1]

        self._result.success = True
        self._result.pose = pose
        self._as.set_succeeded(self._result)



class CameraConfig:
    def __init__(self):
        # Read the yaml path from ros param "cam_config"
        yaml_path = rospy.get_param("~cam_config")

        # Load the config from the yaml file
        with open(yaml_path, 'r') as file:
            content = yaml.safe_load(file)

        self.general = content.pop("general")
        self.objects = content
        self._add_calib()

    def _add_calib(self):
        for object_name, object_config in self.objects.items():
            tf_matrix = object_config["transformation_matrix"]
            angle_offset = object_config.get("angle_offset", 0)
            symmetry = object_config.get("symmetry", None)
            self.objects[object_name]["calib"] = Calibration(tf_matrix, angle_offset, symmetry)
   