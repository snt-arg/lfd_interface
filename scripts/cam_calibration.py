#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Apr 29 11:44:30 2024

@author: abrk
"""

import math
import numpy as np

def calculate_vector_and_transformation(a, b, ref):
    vector_ab = (b[0] - a[0], b[1] - a[1])
    angle_ab = math.atan2(vector_ab[1], vector_ab[0])
    cos_angle = math.cos(angle_ab)
    sin_angle = math.sin(angle_ab)
    transformation_matrix = np.array([[cos_angle, -sin_angle, ref[0]],
                                      [sin_angle, cos_angle, ref[1]],
                                      [0, 0, 1]])
    return vector_ab, math.degrees(angle_ab), transformation_matrix


def calculate_transformation(ref1, ref2, alpha):
    cos_angle = math.cos(alpha)
    sin_angle = math.sin(alpha)
    transformation_matrix = np.array([[cos_angle, -sin_angle, ref1],
                                      [sin_angle, cos_angle, ref2],
                                      [0, 0, 1]])
    return transformation_matrix


def apply_transformation(matrix, coordinate):
    # Convert the coordinate to homogeneous coordinates
    coordinate_homogeneous = np.array([coordinate[0]/1000.0, coordinate[1]/1000.0, 1])
    # Apply the transformation
    transformed_coordinate = np.dot(matrix, coordinate_homogeneous)
    # Convert back to non-homogeneous coordinates
    return transformed_coordinate[0], transformed_coordinate[1]

#%%

# a and b should be along the camera x coordinates


ref = (0.4006, 0.3987)

a = (0.4006, 0.3987)
b = (0.400777, 0.428978)

# b = (0.4047, 0.4044)
# a = (0.4059, 0.340979)


vector_ab, angle_ab, transformation_matrix = calculate_vector_and_transformation(a, b, ref)
print(f"Vector AB is: {vector_ab}")
print(f"Angle of vector AB is: {angle_ab} degrees")
print(f"Transformation matrix is: \n{transformation_matrix}")

#%%


coordinate = (0.0277, -0.0297)
transformed_coordinate = apply_transformation(transformation_matrix, coordinate)
print(f"Transformed coordinate is: {transformed_coordinate}")


#%%

# # Ring coordinates
# ref = [0.4027422, 0.39740265]
# alpha =  1.57370049

# Screw coordinates
ref = [0.21145711, 0.43231239]
alpha =  1.56739902

matrix = calculate_transformation(ref[0], ref[1], alpha)


coordinate = (-72.6, -31.8)
transformed_coordinate = apply_transformation(matrix, coordinate)
print(f"Transformed coordinate is: {transformed_coordinate}")