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
from line_detector.srv import ObjectDetection
from tf.transformations import quaternion_from_euler

service_name="tracker_service"
node_name= "tracker_service"
object_scanning_service_name="object_scanning_service"
line_classes = ['person']
dist_threash = 1 # meter from person in front
dist_from_wall = 0.6
start_angel = 70

def init_server():
    rospy.init_node(node_name)
    service = rospy.Service(service_name,TrackerMsg , handle_request)
    print("Tracker service is up")
    rospy.spin()

'''
Params: 
*radius - the radius of the point
*theta - the angle
*threash - threashold to stand from the original point
Return Value - the point on x,y axes
description - 
'''
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
    goal.target_pose.pose.position.z = 0
    goal_orientation = quaternion_from_euler(0,0,yaw)
    goal.target_pose.pose.orientation.x = goal_orientation[0]
    goal.target_pose.pose.orientation.y = goal_orientation[1]
    goal.target_pose.pose.orientation.z = goal_orientation[2]
    goal.target_pose.pose.orientation.w = goal_orientation[3]
    print("goal location: ", goal)
    ac.send_goal(goal)	
    ac.wait_for_result(rospy.Duration(60))
    if(ac.get_state() ==  GoalStatus.SUCCEEDED):
        rospy.loginfo("You have reached the end of the queue")
    else:
        rospy.loginfo("The robot failed to reach the end of the queue")


#https://math.stackexchange.com/questions/656500/given-a-point-slope-and-a-distance-along-that-slope-easily-find-a-second-p
def dy(distance, m):
    return np.abs(m*dx(distance, m))

def dx(distance, m):
    return distance/np.sqrt(m**2+1)


'''
Params: 
*point1 (x1,y1) - The closest point to the robot
*point2 (x2,y2) - The second closest point to the robot 
Return Value - The point that the robot needs to stand
Description - 
The robot can see points (x1,y1) and (x2,y2) from the first and forth quadrants only
because of his laser scanning range.

'''
def find_position_by_two_points(x1,y1,x2,y2):
    print("pos1: ",x1,"," ,y1," pos2: ",x2," , ",y2)
    m,b = np.polyfit([x1,x2],[y1,y2],1)
    if x2>x1 and y2>y1:
        return x1-dx(dist_threash,m), y1-dy(dist_threash,m)
    if x2>x1 and y2<y1:
        return x1-dx(dist_threash,m), y1+dy(dist_threash,m)
    if x2<x1 and y2>y1:
        return x1+dx(dist_threash,m), y1-dy(dist_threash,m)
    if x2<x1 and y2<y1:
        return x1+dx(dist_threash,m), y1+dy(dist_threash,m)


'''
Params: 
*laser_msg - 
Return Value - the point on x,y axes
description - 

'''
def find_two_closest_points(laser_msg):
    threash_to_next_person = 0.4                                # min distance to even record 
    relevantRanges = laser_msg.ranges[start_angel*4:640]
    filtered=[x for x in relevantRanges if x>0.1]
    # code for following person in front
    minp1 = np.min(filtered)
    filtered_from_per1 = [x for x in filtered if x>minp1+threash_to_next_person]
    minp2=np.min(filtered_from_per1)
    minp1_deg = (start_angel+relevantRanges.index(minp1)/4)     #person 1 degree from positive side
    minp2_deg = (start_angel + relevantRanges.index(minp2)/4)   #person 2 degree from positive side
    print("minp1: ",minp1, " minp1_deg: ", minp1_deg)
    print("minp2: ",minp2, " minp2_deg: ", minp2_deg)
    return minp1,minp1_deg,minp2,minp2_deg


def scan_callback_two_people(minp1,minp1_deg,minp2,minp2_deg):
    if minp1 > dist_threash + 0.6: #move at least when you have 60cm to move
        x1,y1 = polar_to_cartesian(minp1,minp1_deg+270)
        x2,y2 = polar_to_cartesian(minp2,minp2_deg+270)
        x,y = find_position_by_two_points(x1,y1,x2,y2)
        move_base((x,y),np.arctan2(y2-y1,x2-x1))


def point_on_poly(x,y,m,b):
    allowed_threash = 0.05
    return np.abs((m*x +b) - y) <allowed_threash #if point is on the line with at most threash, return True        

def detect_wall(laser_msg,minp1_deg,minp2_deg):
    points_allowed_not_on_poly = 30
    angel_offset = 30 # we look in (min-offset,max+offset) for a wall
    min_wall_deg = np.max([np.min([minp1_deg,minp2_deg]) - angel_offset,0]) #minimum angle to look for a wall, at least zero
    max_wall_deg = np.min([np.max([minp1_deg,minp2_deg]) + angel_offset + points_allowed_not_on_poly/4,180]) #maximum angle to look for a wall, at most 180 (max angel of lidar)
    print("min wall deg: ",min_wall_deg,"max wall deg: ",max_wall_deg)
    wall_len = 120 #decide how many points combined considered a wall
    wall_start = min_wall_deg * 4 #start from right side
    wall_end = wall_start + wall_len #current wall end view
    minDist  =100
    while wall_end <= max_wall_deg*4-1: 
        x1,y1 = polar_to_cartesian(laser_msg.ranges[wall_start],270+wall_start/4)
        x2,y2 = polar_to_cartesian(laser_msg.ranges[wall_end],270+wall_end/4)
        m,b = np.polyfit([x1,x2],[y1,y2],1) #fit to these points a linear line, y=mx+b
        points_not_on_poly = 0
        first_idx_not_on_poly = wall_start #temp
        first_seen = True
        for i in range(wall_start,wall_end):
            radius = laser_msg.ranges[i]
            x,y = polar_to_cartesian(radius,270+i/4)
            if not point_on_poly(x,y,m,b):
                points_not_on_poly +=1 #increment amount of points not on poly
                if first_seen: #save first index which is not on the poly
                    first_idx_not_on_poly = i
                    first_seen = False
            else:
                minDist = np.min([minDist,radius])
            if points_not_on_poly >= points_allowed_not_on_poly:
                wall_start = first_idx_not_on_poly
                wall_end = wall_start + wall_len
                break
        if points_not_on_poly < points_allowed_not_on_poly:
            return m,b # return poly of the wall           
    return False,False

def find_people():
    rospy.wait_for_service(object_scanning_service_name)
    object_detection_service = rospy.ServiceProxy(object_scanning_service_name, ObjectDetection)

    detection_results = object_detection_service(line_classes)

    coords = detection_results.object_coordinates
    
    return coords, detection_results.header


def find_orthogonal_line_through_point(m,x,y):
    slope = -1.0/m if m != 0 else 0
    return slope,y-slope*x


def handle_request(request):
    print("got request, starting to track the person in front ...")
    print("request: ",request)
    loop_rate=rospy.Rate(2)
    x_last_person,y_last_person,m_wall,b_wall = None,None,None,None
    while True:
        laser_msg = rospy.wait_for_message('/scan', LaserScan, timeout=None)
        minp1,minp1_deg,minp2,minp2_deg = find_two_closest_points(laser_msg) #returns angle from the laser view!!
        m_wall,b_wall = detect_wall(laser_msg,minp1_deg,minp2_deg)
        print("wall: m: ",m_wall,"b: ",b_wall)
        x2,y2 = polar_to_cartesian(minp2,minp2_deg+270)
        if (not isinstance(m_wall,bool)) and point_on_poly(x2,y2,m_wall,b_wall): #if the second point is a wall, break the loop
            x_last_person,y_last_person = polar_to_cartesian(minp1,minp1_deg+270)
            print("x_last_person: ",x_last_person,"y_last_person: ",y_last_person,"and wall is: m: ",m_wall,"b_wall: ",b_wall)
            break
        scan_callback_two_people(minp1,minp1_deg,minp2,minp2_deg)
        loop_rate.sleep()
    #from now on there is only one person and a wall

    print("loop broken! (second near point is a wall)")
    #while last point is not the wall
    last_point_x,last_point_y = None,None
    while ((last_point_x is None) and (last_point_y is None)) or (not point_on_poly(last_point_x,last_point_y,m_wall,b_wall)):
        laser_msg = rospy.wait_for_message('/scan', LaserScan, timeout=None)
        relevantRanges=laser_msg.ranges[start_angel*4:640]
        filtered=[x for x in relevantRanges if x>0.1]
        minp1 = np.min(filtered)
        minp1_deg = (start_angel+relevantRanges.index(minp1)/4)
        last_point_x,last_point_y = polar_to_cartesian(minp1,minp1_deg+270)
        loop_rate.sleep()

    #now we know we are the only ones in front of the wall
    m_last_person_and_wall,b_last_person_and_wall = find_orthogonal_line_through_point(m_wall,x_last_person,y_last_person)
    x_intersect = (b_last_person_and_wall - b_wall)/(m_wall - m_last_person_and_wall) # x = b2 - b1/ m1 - m2
    y_intersect = x_intersect * m_wall + b_wall # y = the x value of some line
    goal = (x_intersect-dx(dist_from_wall,m_last_person_and_wall), y_intersect-dy(dist_from_wall,m_last_person_and_wall))
    print("m_last_person_and_wall: ",m_last_person_and_wall,"b_last_person_and_wall: ",b_last_person_and_wall,
    "x_intersect: ",x_intersect,"y_intersect: ",y_intersect,"goal: ",goal)
    move_base(goal,0)
    return TrackerMsgResponse(False)


if __name__ == "__main__":
    init_server()
