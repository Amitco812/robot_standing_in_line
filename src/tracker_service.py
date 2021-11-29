#!/usr/bin/env python

#check about the above line ... python/python3
import rospy
import numpy as np
from robot_standing_in_line.srv import TrackerMsg,TrackerMsgResponse
# from rospy import rostime
from sensor_msgs.msg import LaserScan
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
# import tf2_ros
# from tf2_geometry_msgs import PoseStamped
# from geometry_msgs.msg import Pose,Point,Quaternion
# from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler

service_name="tracker_service"
node_name= "tracker_service"
dist_threash = 1 # meter from person in front
start_angel = 80

def init_server():
    rospy.init_node(node_name)
    service = rospy.Service(service_name,TrackerMsg , handle_request)
    print("Tracker service is up")
    rospy.spin()

def polar_to_cartesian(radius,theta):
    return ((radius-dist_threash)*np.cos(np.radians(theta)),(radius-dist_threash)*np.sin(np.radians(theta)))

def move_base(target,yaw):
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    #wait for the action server to come up
    while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.loginfo("Waiting for the move_base action server to come up")
    goal = MoveBaseGoal()
    #set up the frame parameters
    goal.target_pose.header.frame_id = "base_footprint"
    goal.target_pose.header.stamp = rospy.Time.now()
    # moving towards the goal*/
    goal.target_pose.pose.position.x =  target[0]
    goal.target_pose.pose.position.y =  target[1]
    goal.target_pose.pose.orientation.w = quaternion_from_euler(0,0,np.radians(yaw))[3]
    print("target: ",target)
    print("goal location: ", goal)
    ac.send_goal(goal)	
    ac.wait_for_result(rospy.Duration(60))
    if(ac.get_state() ==  GoalStatus.SUCCEEDED):
        rospy.loginfo("You have reached the end of the queue")
        # ser_messageResponse(True)
    else:
        rospy.loginfo("The robot failed to reach the end of the queue")
        # ser_messageResponse(False)

def scan_callback(msg):
    relevantRanges=msg.ranges[start_angel*4:640]
    filtered=[x for x in relevantRanges if x>0.1]
    # code for following person in front 
    minP = np.min(filtered)
    theta_deg = (start_angel+relevantRanges.index(minP)/4)
    print("minP: ",minP, " theta: ", theta_deg)
    if minP > dist_threash:
        move_base(polar_to_cartesian(minP,theta_deg-90),theta_deg-90)
            

def handle_request(request):
    print("got request, starting to track the person in front ...")
    print("request: ",request)
    loop_rate = rospy.Rate(0.2) 
    while True :
        msg = rospy.wait_for_message('/scan', LaserScan, timeout=None)
        scan_callback(msg)
        loop_rate.sleep()
        # if wall:
        #     break;
    return TrackerMsgResponse(False)

if __name__ == "__main__":
    init_server()