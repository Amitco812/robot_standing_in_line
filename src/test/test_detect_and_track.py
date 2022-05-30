#!/usr/bin/env python
import unittest
import rospy
from robot_standing_in_line.srv import TrackerMsg, TrackerMsgRequest
from line_detector.srv import ser_message,ser_messageRequest

TRACKER_SERVICE_NAME = "/tracker_service"
LINE_END_SERVICE = "/line_end_detection"
PKG = "robot_standing_in_line"

class DetectAndTrackTest(unittest.TestCase):

    #test full pipeline
    def test_detect_and_track(self):
        try:
            rospy.wait_for_service(LINE_END_SERVICE)
            line_end_detection = rospy.ServiceProxy(LINE_END_SERVICE,ser_message)
            line_end_resp = line_end_detection(ser_messageRequest(True))
            self.assertTrue(line_end_resp)
            rospy.wait_for_service(TRACKER_SERVICE_NAME)
            line_tracker = rospy.ServiceProxy(TRACKER_SERVICE_NAME, TrackerMsg)
            resp = line_tracker(TrackerMsgRequest(True))
            self.assertTrue(resp)

        except rospy.ServiceException as e:
            print("detect and track failed with: %s" % e)
            self.fail()



if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG,'test_detect_and_track_nodes',DetectAndTrackTest)
