#!/usr/bin/env python
import unittest
import rospy
from robot_standing_in_line.srv import TrackerMsg, TrackerMsgRequest

TRACKER_SERVICE_NAME = "/tracker_service"
PKG = "robot_standing_in_line"

class TrackerNodeTest(unittest.TestCase):

    def test_track_line_callback(self):
        rospy.wait_for_service(TRACKER_SERVICE_NAME)
        try:
            line_tracker = rospy.ServiceProxy("tracker_service", TrackerMsg)
            resp = line_tracker(TrackerMsgRequest(True))
            self.assertTrue(resp)

        except rospy.ServiceException as e:
            print("line tracking failed: %s" % e)
            self.fail()



if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG,'test_tracker_node',TrackerNodeTest)
