#!/usr/bin/env python

#check about the above line ... python/python3
import rospy
import numpy as np
from robot_standing_in_line.srv import TrackerMsg,TrackerMsgResponse
from sensor_msgs.msg import LaserScan
from laser_line_tracker import LaserLineTracker

service_name="tracker_service"
node_name= "tracker_service"
    
def track_line_callback(request):
    print("got request, starting to track the person in front ...")
    print("request: ",request)
    loop_rate=rospy.Rate(2)
    line_tracker = LaserLineTracker()
    while not line_tracker.done_tracking():
        next_position = line_tracker.get_next_position_in_line()
        line_tracker.move(next_position)
        loop_rate.sleep()

    return TrackerMsgResponse(True)


if __name__ == "__main__":
    rospy.init_node(node_name)
    rospy.Service(service_name,TrackerMsg , track_line_callback)
    print("Tracker service is up")
    rospy.spin()
