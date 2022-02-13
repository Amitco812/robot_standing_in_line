import numpy as np

'''
Params: 
*radius - the radius of the point
*theta - the angle
*threash - threashold to stand from the original point
Return Value - the point on x,y axes
description - 
'''
def polar_to_cartesian(radius,theta,threash=0):
    return ((radius-threash)*np.cos(np.radians(theta)),(radius-threash)*np.sin(np.radians(theta)))
