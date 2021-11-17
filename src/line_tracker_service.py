#!/usr/bin/env python

#check about the above line ... python/python3
import rospy
from robot_standing_in_line.srv import TrackerMsg,TrackerMsgResponse

service_name="line_tracker_service"
node_name= "line_tracker_service"

def init_server():
    rospy.init_node(node_name)
    service = rospy.Service(service_name,TrackerMsg , handle_request)
    print("Tracker service is up")
    rospy.spin()

    
def handle_request(request):
    print("got request, starting to track the person in front ...")
    print("request: ",request)
    return TrackerMsgResponse(False)

if __name__ == "__main__":
    init_server()