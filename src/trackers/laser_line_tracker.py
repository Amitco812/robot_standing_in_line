import rospy
from line_tracker import LineTracker
from utils import dx, dy, polar_to_cartesian, find_two_closest_points, point_on_poly, find_orthogonal_line_through_point
import numpy as np
from sensor_msgs.msg import LaserScan


class LaserLineTracker(LineTracker):
    '''

    @Params:
        *wall_detector- WallDetector
        *dist_tresh- float
        *dist_from_wall - int
    '''

    def __init__(self, wall_detector, dist_thresh=1.0, dist_from_wall=0.6):
        self.wall_detector = wall_detector
        self.dist_thresh = dist_thresh
        self.dist_from_wall = dist_from_wall
        self.p_last_person = None

    '''
    @ PreCondition:
        x1!=x2
    @Params:
        *point1 (x1,y1) - The closest point to the robot
        *point2 (x2,y2) - The second closest point to the robot 
    @Return Value:
        The point that the robot needs to stand.
    @Description:
        The robot can see points (x1,y1) and (x2,y2) from the first and forth quadrants only
        because of his laser scanning range.
    '''

    def find_position_by_two_points(self, x1, y1, x2, y2):
        print("pos1: ", x1, ",", y1, " pos2: ", x2, " , ", y2)
        m, _ = np.polyfit([x1, x2], [y1, y2], 1)
        if x2 > x1 and y2 > y1:
            return x1-dx(self.dist_thresh, m), y1-dy(self.dist_thresh, m)
        if x2 > x1 and y2 < y1:
            return x1-dx(self.dist_thresh, m), y1+dy(self.dist_thresh, m)
        if x2 < x1 and y2 > y1:
            return x1+dx(self.dist_thresh, m), y1-dy(self.dist_thresh, m)
        if x2 < x1 and y2 < y1:
            return x1+dx(self.dist_thresh, m), y1+dy(self.dist_thresh, m)
        # x1==x2 case is too rare

    '''
    notice : at the case that the line contains only one person, the variable self.p_last_person will be taken too early!
    @ PreCondition:
        At least 1 person in the line.
        Armadillo publishes to laser topic.
    @Params:
        None
    @Return Value:
        should_move, tuple of 3 marks the location to go to.
    @Description:
        The function returns the next position the robot needs to follow. 
        After retrieving the two closest points to the robot, we check whether those points are on the wall or not.
        If there are no people in line we take the last position of a person we saw and call 'no_people_in_line'.
        If we see one person we update self.p_last_person for later use. 
    '''

    def get_next_position_in_line(self):
        laser_msg = rospy.wait_for_message('/scan', LaserScan, timeout=None)
        minp1, minp1_deg, minp2, minp2_deg = find_two_closest_points(
            laser_msg)        # returns angle from the laser view!!
        x1, y1 = polar_to_cartesian(minp1, minp1_deg+270)
        x2, y2 = polar_to_cartesian(minp2, minp2_deg+270)
        wall = self.wall_detector.detect_wall()
        if wall:                                                                    # if wall found
            m_wall, b_wall = wall
            # the second point is a wall, first time only!!
            if self.p_last_person == None and point_on_poly(x2, y2, m_wall, b_wall):
                self.p_last_person = polar_to_cartesian(minp1, minp1_deg+270)
                print("x_last_person: ", self.p_last_person[0], "y_last_person: ",
                      self.p_last_person[1], "and wall is: m: ", m_wall, "b_wall: ", b_wall)
            # no people in line, only wall (we have the last person point)
            elif self.p_last_person != None and point_on_poly(x1, y1, m_wall, b_wall):
                return True, self.no_people_in_line(m_wall, b_wall)
            else:  # no people in line, no last person point contradicts preconditions
                raise Exception("Empty Line Case")
        yaw = np.arctan2(y2-y1, x2-x1)
        # move at least when you have 60cm to move
        should_move = minp1 > self.dist_thresh + 0.6
        x, y = self.find_position_by_two_points(x1, y1, x2, y2)
        # 2 people
        return should_move, (x, y, yaw)

    '''
    @Pre:
        m_wall != m_last_person_and_wall
    @Params:
        *m_wall- The incline of the wall from robot's prespective
        *b_wall- The b of the line
    @Return Value:
        x,y and yaw of the new position the robot should go to.
    Description:
        The function calculates the new position of the robot to go to.
        This happens by taking the wall line and make a orthognal line with it and the last person in line.
        After finding the orthognal line, we find the point on wall that intesects the wall line and the perpendicular line.
        the robot is placed on the perpendicular line at a 'dist_from_wall' from the intersaction point on the wall.
    '''

    def no_people_in_line(self, m_wall, b_wall):
        m_last_person_and_wall, b_last_person_and_wall = find_orthogonal_line_through_point(
            m_wall, self.p_last_person[0], self.p_last_person[1])
        x_intersect = (b_last_person_and_wall - b_wall)/(m_wall -
                                                         m_last_person_and_wall)       # x = b2 - b1/ m1 - m2
        # y = the x value of some line
        y_intersect = x_intersect * m_wall + b_wall
        goal = (x_intersect-dx(self.dist_from_wall, m_last_person_and_wall),
                y_intersect-dy(self.dist_from_wall, m_last_person_and_wall))
        print("m_last_person_and_wall: ", m_last_person_and_wall, "b_last_person_and_wall: ", b_last_person_and_wall,
              "x_intersect: ", x_intersect, "y_intersect: ", y_intersect, "goal: ", goal)
        return x_intersect, y_intersect, m_last_person_and_wall
