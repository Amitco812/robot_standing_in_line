import rospy
from laser_data_proxy import LaserDataProxy
from sensor_msgs.msg import LaserScan


class LaserDataReal(LaserDataProxy):

    def get_laser_data(self):
        return rospy.wait_for_message('/scan', LaserScan, timeout=None)
