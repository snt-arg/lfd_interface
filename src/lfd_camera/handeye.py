
import rospy
import numpy as np
import yaml

from pydrake.solvers import MathematicalProgram, Solve
from pydrake import math
from math import pi

from geometry_msgs.msg import Pose
from lfd_program.core.camera import CameraProgram


class DataCollection:

    def __init__(self):
        self.robot_pose = None
        self.camera_pose = None

        self.camera = CameraProgram()

        rospy.Subscriber('/pose_state', Pose, self.pose_callback)

    def pose_callback(self, msg):
        self.robot_pose = msg

    def read_robot_pose(self):
        return [self.robot_pose.position.x, self.robot_pose.position.y]
    
    def read_camera_pose(self):
        pose = self.camera.trigger("", Pose())
        return [pose.position.x, pose.position.y]


class Solver:

    def __init__(self, data_path):
        with open(data_path, 'r') as file:
            data = yaml.safe_load(file)
        
        # Extract the coordinates from the loaded data
        camera = np.array([np.array(coordinate) for coordinate in data['camera']])
        robot = np.array([np.array(coordinate) for coordinate in data['robot']])

        # Add a 1 to the end of each 2D vector
        self.camera = np.hstack((camera, np.ones((camera.shape[0], 1))))
        self.robot = np.hstack((robot, np.ones((robot.shape[0], 1))))        
            
    def solve(self):
        prog = MathematicalProgram()

        ref = prog.NewContinuousVariables(2 , "ref")
        alpha = prog.NewContinuousVariables(1, "alpha")

        prog.AddConstraint(alpha[0]>=0)
        prog.AddConstraint(alpha[0]<=pi)

        cos_angle = math.cos(alpha[0])
        sin_angle = math.sin(alpha[0])
        transformation_matrix = np.array([[cos_angle, -sin_angle, ref[0]],
                                        [sin_angle, cos_angle, ref[1]],
                                        [0, 0, 1]])

        transformed_v = np.dot(transformation_matrix, self.camera.T).T

        cost = (self.robot - transformed_v) ** 2
        total_cost = np.sum(cost)
        prog.AddCost(total_cost)

        result = Solve(prog)
        # print("Success? ", result.is_success())     
        # print('ref = ', result.GetSolution(ref))
        # print('alpha = ', result.GetSolution(alpha))
        # print('optimal cost = ', result.get_optimal_cost())
        # print('solver is: ', result.get_solver_id().name())

        return result, self.calculate_transformation(result.GetSolution(ref), result.GetSolution(alpha)) 

    def calculate_transformation(self,ref, alpha):
        cos_angle = math.cos(alpha)
        sin_angle = math.sin(alpha)
        transformation_matrix = np.array([[cos_angle, -sin_angle, ref[0]],
                                        [sin_angle, cos_angle, ref[1]],
                                        [0, 0, 1]])
        return transformation_matrix

