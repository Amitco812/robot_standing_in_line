#!/usr/bin/env python
import unittest
import rospy
from robot_standing_in_line.srv import TrackerMsg, TrackerMsgRequest

TRACKER_SERVICE_NAME = "/tracker_service"
PKG = "robot_standing_in_line"

class TrackerNodeTest(unittest.TestCase):

    #test full pipeline
    def test_track_line_callback(self):
        rospy.wait_for_service(TRACKER_SERVICE_NAME)
        try:
            # line_end_detection = rospy.ServiceProxy("/line_end_detection",ser_message)
            # line_end_resp = line_end_detection(ser_messageRequest(True))
            # print("response from line end: ",line_end_resp)
            line_tracker = rospy.ServiceProxy("tracker_service", TrackerMsg)
            resp = line_tracker(TrackerMsgRequest(True))
            self.assertTrue(resp)

        except rospy.ServiceException as e:
            print("line tracking failed: %s" % e)
            self.assertTrue(False)



if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG,'test_tracker_node',TrackerNodeTest)
