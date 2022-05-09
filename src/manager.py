#!/usr/bin/env python
import rospy
import sys
from line_detector.srv import ser_message,ser_messageRequest
from mocks.speech_service_mock import speech_service_mock
from voice_text_interface.srv import *
from robot_standing_in_line.srv import TrackerMsg, TrackerMsgRequest
from robotican_demos_upgrade.srv import pick_unknown, pick_unknownRequest


TRACKER_SERVICE_NAME = "/tracker_service"
PICK_SERVICE_NAME = "/pick_unknown"


def call_speech_service(is_mock,request):
    if is_mock:
        return speech_service_mock(request)
    else:
        rospy.wait_for_service('speech_to_text')
        speech_to_text_req = rospy.ServiceProxy('speech_to_text', speech_to_text) 
        resp1 = speech_to_text_req(request)  
        print("Responding to the voice command!")   
        return resp1.status


if __name__ == "__main__":
    speech_mock = sys.argv[1] == 'true'
    # rospy.wait_for_service("/line_end_detection")
    rospy.wait_for_service(TRACKER_SERVICE_NAME)
    try:
         # === LINE END DETECTION START ===
        line_end_detection = rospy.ServiceProxy("/line_end_detection",ser_message)
        line_end_resp = line_end_detection(ser_messageRequest(True))
        print("response from line end: ",line_end_resp)
        # === LINE END DETECTION ENDS ===
    except rospy.ServiceException as e:
        print("line end detection exception: %s" % e)

    try:
        # === LINE TRACKER START ===
        line_tracker = rospy.ServiceProxy("tracker_service", TrackerMsg)
        line_tracker(TrackerMsgRequest(True))
        # === LINE TRACKER END ===
        # === SPEECH SYNTHESIS START ===
        resp = call_speech_service(speech_mock,"Bring me coffee please")
        # === SPEECH SYNTHESIS END ===
        # === PICK SERVICE START ===
        rospy.wait_for_service(PICK_SERVICE_NAME)
        pick_service = rospy.ServiceProxy("pick_unknown", pick_unknown)
        pick_service(pick_unknownRequest())  # '', '', ''))
        # === PICK SERVICE END ===

    except rospy.ServiceException as e:
        print("exception: %s" % e)
        


