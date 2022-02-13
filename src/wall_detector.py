from abc import ABC, abstractmethod
import rospy
from sensor_msgs.msg import LaserScan
from utils import find_two_closest_points,polar_to_cartesian,point_on_poly
import numpy as np

class WallDetector(ABC):
    @abstractmethod
    def detect_wall():
        raise NotImplementedError("This Function Is Not Implemented!")

class PointsWallDetector(WallDetector):
    def __init__(self,points_allowed_not_on_poly,angel_offset,wall_len):
        self.points_allowed_not_on_poly = points_allowed_not_on_poly
        self.angel_offset=angel_offset
        self.wall_len = wall_len #decide how many points combined considered a wall
        
    
    '''
    
    '''
    def detect_wall(self):
        laser_msg = rospy.wait_for_message('/scan', LaserScan, timeout=None)
        _,minp1_deg,_,minp2_deg = find_two_closest_points(laser_msg) #returns angle from the laser view!!
        min_wall_deg = np.max([np.min([minp1_deg,minp2_deg]) -  self.angel_offset,0]) #minimum angle to look for a wall, at least zero
        max_wall_deg = np.min([np.max([minp1_deg,minp2_deg]) +  self.angel_offset + self.points_allowed_not_on_poly/4,180]) #maximum angle to look for a wall, at most 180 (max angel of lidar)
        print("min wall deg: ",min_wall_deg,"max wall deg: ",max_wall_deg)
        wall_start = min_wall_deg * 4 #start from right side
        wall_end = wall_start + self.wall_len #current wall end view
        minDist  =100
        while wall_end <= max_wall_deg*4-1: 
            x1,y1 = polar_to_cartesian(laser_msg.ranges[wall_start],270+wall_start/4)
            x2,y2 = polar_to_cartesian(laser_msg.ranges[wall_end],270+wall_end/4)
            m,b = np.polyfit([x1,x2],[y1,y2],1) #fit to these points a linear line, y=mx+b
            points_not_on_poly = 0
            first_idx_not_on_poly = wall_start #temp
            first_seen = True
            for i in range(wall_start,wall_end):
                radius = laser_msg.ranges[i]
                x,y = polar_to_cartesian(radius,270+i/4)
                if not point_on_poly(x,y,m,b):
                    points_not_on_poly +=1 #increment amount of points not on poly
                    if first_seen: #save first index which is not on the poly
                        first_idx_not_on_poly = i
                        first_seen = False
                else:
                    minDist = np.min([minDist,radius])
                if points_not_on_poly >= self.points_allowed_not_on_poly:
                    wall_start = first_idx_not_on_poly
                    wall_end = wall_start + self.wall_len
                    break
            if points_not_on_poly < self.points_allowed_not_on_poly:
                return m,b # return poly of the wall           
        return False


