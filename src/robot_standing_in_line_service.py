#!/usr/bin/env python
import rospy
from line_detector.srv import ser_message,ser_messageRequest
from robot_standing_in_line.srv import TrackerMsg,TrackerMsgRequest

if __name__ =="__main__":
    rospy.wait_for_service("/line_end_detection")
    while True:
        try:
            line_end_detection = rospy.ServiceProxy("/line_end_detection",ser_message) 
            line_end_resp = line_end_detection(ser_messageRequest(True))
            print("response from line end: ",line_end_resp)
            line_tracker = rospy.ServiceProxy("line_tracker_service",TrackerMsg)
            line_tracker(TrackerMsgRequest(True))
            break
        except rospy.ServiceException as e:
            print("line end detection failed: %s"%e)
