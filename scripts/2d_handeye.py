#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed May  1 13:09:07 2024

@author: abrk
"""

import numpy as np

# from pydrake.all import *

from pydrake.solvers import MathematicalProgram, Solve
from pydrake import math

from math import pi
import yaml

#%%

prog = MathematicalProgram()

ref = prog.NewContinuousVariables(2 , "ref")
alpha = prog.NewContinuousVariables(1, "alpha")

prog.AddConstraint(alpha[0]>=0)
prog.AddConstraint(alpha[0]<=pi)

#%%

with open('/home/abrk/catkin_ws/src/lfd/lfd_interface/config/camera/handeye_ring.yaml', 'r') as file:
    data = yaml.safe_load(file)

# Extract the coordinates from the loaded data
camera = np.array([np.array(coordinate) for coordinate in data['camera']])
robot = np.array([np.array(coordinate) for coordinate in data['robot']])

# Add a 1 to the end of each 2D vector
camera = np.hstack((camera, np.ones((camera.shape[0], 1))))
robot = np.hstack((robot, np.ones((robot.shape[0], 1))))


#%%
cos_angle = math.cos(alpha[0])
sin_angle = math.sin(alpha[0])
transformation_matrix = np.array([[cos_angle, -sin_angle, ref[0]],
                                  [sin_angle, cos_angle, ref[1]],
                                  [0, 0, 1]])

transformed_v = np.dot(transformation_matrix, camera.T).T

#%%
cost = (robot - transformed_v) ** 2
total_cost = np.sum(cost)
prog.AddCost(total_cost)

#%%
result = Solve(prog)
print("Success? ", result.is_success())

print('ref = ', result.GetSolution(ref))
print('alpha = ', result.GetSolution(alpha))
# Print the optimal cost.
print('optimal cost = ', result.get_optimal_cost())
# Print the name of the solver that was called.
print('solver is: ', result.get_solver_id().name())