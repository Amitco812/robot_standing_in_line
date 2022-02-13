from abc import ABC, abstractmethod
import rospy
from sensor_msgs.msg import LaserScan

class WallDetector(ABC):
    @abstractmethod
    def detect_wall():
        raise NotImplementedError("This Function Is Not Implemented!")

class PointsWallDetector(WallDetector):
    def detect_wall():
        # laser_msg = rospy.wait_for_message('/scan', LaserScan, timeout=None)
        # minp1,minp1_deg,minp2,minp2_deg = find_two_closest_points(laser_msg) #returns angle from the laser view!!
        pass



