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
start_angel = 70

def init_server():
    rospy.init_node(node_name)
    service = rospy.Service(service_name,TrackerMsg , handle_request)
    print("Tracker service is up")
    rospy.spin()

def polar_to_cartesian(radius,theta,threash=0):
    return ((radius-threash)*np.cos(np.radians(theta)),(radius-threash)*np.sin(np.radians(theta)))

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
    if minP > dist_threash+0.5:
        move_base(polar_to_cartesian(minP,theta_deg+270,dist_threash),theta_deg+270)


def find_position_by_two_points(r1,theta1,r2,theta2):
    x1,y1 = polar_to_cartesian(r1,theta1)
    x2,y2 = polar_to_cartesian(r2,theta2)
    print(x1," , " ,y1)
    print(x2," , " ,y2)

    m,b = np.polyfit([x1,x2],[y1,y2],1)
    if x2>x1 and y2>y1:
        return x1-dist_threash*(np.sqrt(1/(1+m**2))),y1-m*dist_threash*(np.sqrt(1/(1+m**2)))
    if x2>x1 and y2<y1:
        return x1-dist_threash*(np.sqrt(1/(1+m**2))),y1+m*dist_threash*(np.sqrt(1/(1+m**2)))
    if x2<x1 and y2>y1:
        return x1+dist_threash*(np.sqrt(1/(1+m**2))),y1-m*dist_threash*(np.sqrt(1/(1+m**2)))
    if x2<x1 and y2<y1:
        return x1+dist_threash*(np.sqrt(1/(1+m**2))),y1+m*dist_threash*(np.sqrt(1/(1+m**2)))

def scan_callback_two_people(msg):
    threash_to_next_person = 0.6
    relevantRanges=msg.ranges[start_angel*4:640]
    filtered=[x for x in relevantRanges if x>0.1]
    # code for following person in front 
    per1 = np.min(filtered)
    filtered_from_per1 = [x for x in filtered if x>per1+threash_to_next_person]
    per2=np.min(filtered_from_per1)
    per1_deg = (start_angel+relevantRanges.index(per1)/4)   #person 1 degree from positive side
    per2_deg = (start_angel + relevantRanges.index(per2)/4) #person 2 degree from positive side
    
    print("per1: ",per1, " per1_deg: ", per1_deg)
    print("per2: ",per2, " per2_deg: ", per2_deg)

    if per1 > dist_threash+0.5:
        x,y = find_position_by_two_points(per1,per1_deg+270,per2,per2_deg+270)
        move_base((x,y),per1_deg+270)


def point_on_poly(x,y,m,b):
    allowed_threash = 0.1
    if np.abs((m*x +b) - y) <allowed_threash: #if point is on the line with at most threash, return True
        return True
    return False        

def detect_wall(msg):
    wall_len = 170 #decide how many points combined considered a wall
    wall_start = len(msg.ranges)-1 #start the detection from the left!
    wall_end = wall_start - wall_len
    while wall_end >= 130*4: #cut the positive 130 angel view
        x1,y1 = polar_to_cartesian(msg.ranges[wall_start],180-wall_start/4)
        x2,y2 = polar_to_cartesian(msg.ranges[wall_end],180-wall_end/4)
        m,b = np.polyfit([x1,x2],[y1,y2],1) #fit to these points a linear line, y=mx+b
        for i in reversed(range(wall_end,wall_start)):
            radius = msg.ranges[i]
            x,y = polar_to_cartesian(radius,180-i/4)
            if not point_on_poly(x,y,m,b):
                wall_start = i
                wall_end = wall_start - wall_len
                break
        return True
                
    return False

def handle_request(request):
    print("got request, starting to track the person in front ...")
    print("request: ",request)
    while True:
        msg = rospy.wait_for_message('/scan', LaserScan, timeout=None)
        scan_callback_two_people(msg)
        #scan_callback(msg)
        #loop_rate.sleep()
        # if wall:
        #     break;
    return TrackerMsgResponse(False)

if __name__ == "__main__":
    init_server()