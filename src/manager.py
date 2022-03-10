#!/usr/bin/env python
import rospy
# from line_detector.srv import ser_message,ser_messageRequest
from robot_standing_in_line.srv import TrackerMsg, TrackerMsgRequest
from robotican_demos_upgrade.srv import pick_unknown, pick_unknownRequest


TRACKER_SERVICE_NAME = "/tracker_service"
PICK_SERVICE_NAME = "/pick_unknown"
if __name__ == "__main__":
    # rospy.wait_for_service("/line_end_detection")
    rospy.wait_for_service(TRACKER_SERVICE_NAME)
    try:
        # line_end_detection = rospy.ServiceProxy("/line_end_detection",ser_message)
        # line_end_resp = line_end_detection(ser_messageRequest(True))
        # print("response from line end: ",line_end_resp)
        line_tracker = rospy.ServiceProxy("tracker_service", TrackerMsg)
        line_tracker(TrackerMsgRequest(True))
        # HERE COMES SPEECH SYNTHESIS SERVICE
        rospy.wait_for_service(PICK_SERVICE_NAME)
        pick_service = rospy.ServiceProxy("pick_unknown", pick_unknown)
        pick_service(pick_unknownRequest())  # '', '', ''))

    except rospy.ServiceException as e:
        print("line end detection failed: %s" % e)
