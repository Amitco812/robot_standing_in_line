#!/usr/bin/env python

#check about the above line ... python/python3
import rospy
import numpy as np
from robot_standing_in_line.srv import TrackerMsg,TrackerMsgResponse
from sensor_msgs.msg import LaserScan

service_name="tracker_service"
node_name= "tracker_service"

def init_server():
    rospy.init_node(node_name)
    service = rospy.Service(service_name,TrackerMsg , handle_request)
    print("Tracker service is up")
    rospy.spin()

def scan_callback(msg):
    relevantRanges=msg.ranges[240:540]
    print(relevantRanges)
    filtered=[x for x in relevantRanges if x>0.1]
    minP = np.min(filtered)
    print(minP,240+filtered.index(minP))

def handle_request(request):
    print("got request, starting to track the person in front ...")
    print("request: ",request)
    sub = rospy.Subscriber('/scan', LaserScan, scan_callback)
       

    # return TrackerMsgResponse(False)

if __name__ == "__main__":
    init_server()