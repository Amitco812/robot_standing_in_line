#!/usr/bin/env python

#check about the above line ... python/python3
import rospy
from robot_standing_in_line.srv import TrackerMsg,TrackerMsgResponse
from laser_line_tracker import LaserLineTracker
from laser_points_wall_detector import LaserPointsWallDetector
from utils import move

service_name="tracker_service"
node_name= "tracker_service"
    
def track_line_callback(request):
    print("got request, starting to track the person in front ...")
    print("request: ",request)
    loop_rate=rospy.Rate(2)
    laser_points_wall_detector = LaserPointsWallDetector(30,30,120)
    line_tracker = LaserLineTracker(laser_points_wall_detector,1.0,0.6)
    while not line_tracker.done_tracking():
        should_move,next_position = line_tracker.get_next_position_in_line()
        if should_move:
            move(next_position)
        loop_rate.sleep()

    return TrackerMsgResponse(True)


if __name__ == "__main__":
    rospy.init_node(node_name)
    rospy.Service(service_name,TrackerMsg , track_line_callback)
    print("Tracker service is up")
    rospy.spin()
