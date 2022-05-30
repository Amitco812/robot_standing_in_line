#!/usr/bin/env python
import unittest
import rospy
from line_detector.srv import ser_message,ser_messageRequest

LINE_END_SERVICE = "/line_end_detection"
PKG = "robot_standing_in_line"

class LineEndDetectionTest(unittest.TestCase):

    #test full pipeline
    def test_line_end_detection(self):
        rospy.wait_for_service(LINE_END_SERVICE)
        try:
            line_end_detection = rospy.ServiceProxy("/line_end_detection",ser_message)
            line_end_resp = line_end_detection(ser_messageRequest(True))
            self.assertTrue(line_end_resp)

        except rospy.ServiceException as e:
            print("line end detection failed: %s" % e)
            self.fail()



if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG,'test_line_end_detection_node',LineEndDetectionTest)
