
import rospy
import numpy as np
import math

from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler, quaternion_multiply

class Calibration:

    def __init__(self, tf_matrix, angle_offset, symmetry=None):
        """
        tf_matrix: 3x3 transformation matrix
        angle_offset: angle offset in radians
        symmetry: None, "half", "full"
        """
        self.T = tf_matrix
        self.angle_offset = angle_offset
        self.symmetry = symmetry

    def transform(self, coord, pose_template: Pose):
        """
        coord: [x [m],y [m], angle2D [rad]]
        """
        p_cam = np.array(coord)[:2]
        angle_cam = coord[-1] + self.angle_offset

        p_cam = np.append(p_cam, 1)

        p = np.dot(self.T, p_cam)
        p[-1] = angle_cam

        return self._fill_pose_template(pose_template, p)
    
    def _fill_pose_template(self,pose_template: Pose, p):
        pose_template.position.x = p[0]
        pose_template.position.y = p[1]
        pose = self._rotate_pose_around_z(pose_template,-p[2])
        return pose
    
    def _rotate_pose_around_z(self,pose, x):
        """
        Rotates a geometry_msgs/Pose message around the z-axis by x radians.

        Parameters:
        pose (Pose): The input Pose message.
        x (float): Angle in radians to rotate around the z-axis.

        Returns:
        Pose: The rotated Pose message.
        """
        x = self._normalize_angle(x)
        orientation = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        rotation_quaternion = quaternion_from_euler(0, 0, x)
        rotated_orientation = quaternion_multiply(orientation, rotation_quaternion)
        
        rotated_pose = Pose()
        rotated_pose.position = pose.position 
        rotated_pose.orientation.x = rotated_orientation[0]
        rotated_pose.orientation.y = rotated_orientation[1]
        rotated_pose.orientation.z = rotated_orientation[2]
        rotated_pose.orientation.w = rotated_orientation[3]
        
        return rotated_pose
    
    def _normalize_angle(self, angle):
        """
        angle: radians
        """
        if self.symmetry is None:
            angle = math.atan2(math.sin(angle), math.cos(angle))

        elif self.symmetry == "half":
            max_rotation_angle = math.radians(90)
            angle = math.atan2(math.sin(angle), math.cos(angle))
            if abs(angle) > max_rotation_angle:
                angle = angle + math.radians(180)
                angle = math.atan2(math.sin(angle), math.cos(angle))

        elif self.symmetry == "full":
            angle = 0
            
        return angle

    