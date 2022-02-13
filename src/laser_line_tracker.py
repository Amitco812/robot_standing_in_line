import rospy
from line_tracker import LineTracker
from wall_detector import WallDetector
from utils import dx,dy,polar_to_cartesian,move_base,find_two_closest_points,point_on_poly
import numpy as np
from move_base_msgs.msg import MoveBaseGoal
from sensor_msgs.msg import LaserScan
from tf.transformations import quaternion_from_euler


class LaserLineTracker(LineTracker):
    def __init__(self,wall_detector:WallDetector,dist_threash=1,dist_from_wall=0.6):
        self.dist_threash = dist_threash
        self.should_move = False
        self.wall_detector=wall_detector
        self.dist_from_wall = dist_from_wall


    '''
    @Description - 
    The robot can see points (x1,y1) and (x2,y2) from the first and forth quadrants only
    because of his laser scanning range.
    @Params: 
    *point1 (x1,y1) - The closest point to the robot
    *point2 (x2,y2) - The second closest point to the robot 
    Return Value - The point that the robot needs to stand
    
    '''
    def find_position_by_two_points(self,x1,y1,x2,y2):
        print("pos1: ",x1,"," ,y1," pos2: ",x2," , ",y2)
        m,_ = np.polyfit([x1,x2],[y1,y2],1)
        if x2>x1 and y2>y1:
            return x1-dx(self.dist_threash,m), y1-dy(self.dist_threash,m)
        if x2>x1 and y2<y1:
            return x1-dx(self.dist_threash,m), y1+dy(self.dist_threash,m)
        if x2<x1 and y2>y1:
            return x1+dx(self.dist_threash,m), y1-dy(self.dist_threash,m)
        if x2<x1 and y2<y1:
            return x1+dx(self.dist_threash,m), y1+dy(self.dist_threash,m)
    
    '''   
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
    '''

    def get_next_position_in_line(self):
        laser_msg = rospy.wait_for_message('/scan', LaserScan, timeout=None)
        minp1,minp1_deg,minp2,minp2_deg = find_two_closest_points(laser_msg) #returns angle from the laser view!!
        x1,y1 = polar_to_cartesian(minp1,minp1_deg+270)
        x2,y2 = polar_to_cartesian(minp2,minp2_deg+270)
        wall = self.wall_detector.detect_wall()
        if wall: #if wall found
            m_wall,b_wall = wall
            if point_on_poly(x2,y2,m_wall,b_wall):
                x_last_person,y_last_person = polar_to_cartesian(minp1,minp1_deg+270)
                print("x_last_person: ",x_last_person,"y_last_person: ",y_last_person,"and wall is: m: ",m_wall,"b_wall: ",b_wall)              

    '''
    def get_next_position_in_line(self):
        laser_msg = rospy.wait_for_message('/scan', LaserScan, timeout=None)
        minp1,minp1_deg,minp2,minp2_deg = find_two_closest_points(laser_msg) #returns angle from the laser view!!
        x1,y1 = polar_to_cartesian(minp1,minp1_deg+270)
        x2,y2 = polar_to_cartesian(minp2,minp2_deg+270)
        yaw = np.arctan2(y2-y1,x2-x1)
        self.should_move =  minp1 > self.dist_threash + 0.6 #move at least when you have 60cm to move
        return self.find_position_by_two_points(x1,y1,x2,y2),yaw
    '''
    


    '''
    @Description: Move if you have 60cm at least
    @Params:
    *data - a dictionary with data on the two closest points
    '''
    def move(self,data):
        if self.should_move:
            x,y,yaw = data['x'],data['y'],data['yaw']
            goal = self.preapare_goal(x,y,yaw)
            move_base(goal)

    '''
    @Params:
    *(x,y)- traget location
    *yaw- the target yaw
    '''
    def preapare_goal(self,x,y,yaw):
        goal = MoveBaseGoal()
        #set up the frame parameters
        goal.target_pose.header.frame_id = "base_footprint"
        goal.target_pose.header.stamp = rospy.Time.now()
        # moving towards the goal*/
        goal.target_pose.pose.position.x =  x
        goal.target_pose.pose.position.y =  y
        goal.target_pose.pose.position.z = 0
        goal_orientation = quaternion_from_euler(0,0,yaw)
        goal.target_pose.pose.orientation.x = goal_orientation[0]
        goal.target_pose.pose.orientation.y = goal_orientation[1]
        goal.target_pose.pose.orientation.z = goal_orientation[2]
        goal.target_pose.pose.orientation.w = goal_orientation[3]
        return goal
        

    