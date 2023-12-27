#!/usr/bin/env python3

import numpy as np
import math

def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'::

            >>> angle_between((1, 0, 0), (0, 1, 0))
            1.5707963267948966
            >>> angle_between((1, 0, 0), (1, 0, 0))
            0.0
            >>> angle_between((1, 0, 0), (-1, 0, 0))
            3.141592653589793
    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    direction = np.cross(v1_u,v2_u)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))


def angle_to_vector(theta):
    return (math.cos(theta), math.sin(theta))

def convert_to_0_to_2pi(angle):
    if 0<= angle < (math.pi*2) :
        return angle
    elif angle >= (math.pi*2):
        return angle % (math.pi*2)
    else:
        return angle+(math.pi*2)

poseAngle = math.pi*-0.1
targetAngle = math.pi*-0.75
v1 = unit_vector(angle_to_vector(poseAngle))
v2 = unit_vector(angle_to_vector(targetAngle))
# ang = math.degrees(angle_between(v1,v2))
# poseAngleDeg = convert_to_0_to_2pi(poseAngle)
# targetAngleDeg = convert_to_0_to_2pi(targetAngle)

if (np.cross(v1,v2) > 0):
    print("Turn Clockwise")
else:
    print("Turn Counter Clockwise")