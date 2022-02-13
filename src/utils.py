import rospy
import numpy as np
import actionlib
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
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


'''
Params: 
*distance - distance on line 
*m - slope of the line
Return Value- dx/dy
description- https://math.stackexchange.com/questions/656500/given-a-point-slope-and-a-distance-along-that-slope-easily-find-a-second-p
'''
def dy(distance, m):
    return np.abs(m*dx(distance, m))
def dx(distance, m):
    return distance/np.sqrt(m**2+1)

'''
Params:
*(x,y) - point
*m,b - describes the line
Return Value - true if point on poly, else false
dedcription - 
checks if point is on the line with at most threash
''' 
def point_on_poly(x,y,m,b):
    allowed_threash = 0.05
    return np.abs((m*x +b) - y) <allowed_threash      

'''
Params:
*m - slope of line
*(x,y) - point on the line
Return Value - m,b of the orthogonal line through the input (x,y)
'''
def find_orthogonal_line_through_point(m,x,y):
    slope = -1.0/m if m != 0 else 0
    return slope,y-slope*x

'''
Params: 
*laser_msg - 
*threash_to_next_person -  min distance to even record 
*start_angel - the angle to start scanning from
Return Value - r1,theta1 , r2,theta2 of the 2 closests points 
description - 

'''
def find_two_closest_points(laser_msg,threash_to_next_person = 0.4,start_angel=70):                             
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

def move_base(goal : MoveBaseGoal):
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    #wait for the action server to come up
    while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.loginfo("Waiting for the move_base action server to come up")
    print("goal location: ", goal)
    ac.send_goal(goal)	
    ac.wait_for_result(rospy.Duration(60))
    if(ac.get_state() ==  GoalStatus.SUCCEEDED):
        rospy.loginfo("You have reached the end of the queue")
    else:
        rospy.loginfo("The robot failed to reach the end of the queue")