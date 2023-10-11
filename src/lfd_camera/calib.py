
import rospy

import numpy as np
import yaml


class Calibration:

    def __init__(self):
        self.T = rospy.get_param("~transformation_matrix")

    def transform(self, coord):
        """
        coord: [x [m],y [m], angle2D [rad]]
        """
        p_cam = np.array(coord)[:2]
        angle_cam = coord[-1]

        p_cam = np.append(p_cam, 1)

        p = np.dot(self.T, p_cam)
        return p