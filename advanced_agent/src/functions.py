#!/usr/bin/env python3

import math 
import numpy as np


def get_distance(goal1, goal2):
    x = (goal2[0] - goal1[0])**2
    y = (goal2[1] - goal1[1])**2
    z = 0

    return math.sqrt(x + y + z)


def get_angle(goal1, goal2):
    vector_1 = [goal2[0] - goal1[0], goal2[1] - goal1[1], goal2[2] - goal1[2]]
    vector_2 = [1, 0, 0]

    unit_vector_1 = vector_1 / np.linalg.norm(vector_1)
    unit_vector_2 = vector_2 / np.linalg.norm(vector_2)
    dot_product = np.dot(unit_vector_1, unit_vector_2)

    angle = np.arccos(dot_product)
    degrees = math.degrees(angle)
    
    #Fix para ángulos mayores a 180
    if goal2[1] < goal1[1]:
        degrees = 360 - degrees
        
    return degrees


def get_path_length(actual_path):
    path_length = 0
    for i in range(len(actual_path.poses) - 1):
        goal1 = [actual_path.poses[i].pose.position.x, actual_path.poses[i].pose.position.y]
        goal2 = [actual_path.poses[i+1].pose.position.x, actual_path.poses[i+1].pose.position.y]
        path_length += get_distance(goal1, goal2)

    return path_length

