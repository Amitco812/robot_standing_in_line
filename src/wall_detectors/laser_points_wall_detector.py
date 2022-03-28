from data_objects.laser_data_real import LaserDataReal
from wall_detector import WallDetector
import numpy as np
from utils import find_two_closest_points, polar_to_cartesian, point_on_poly


class LaserPointsWallDetector(WallDetector):
    def __init__(self, laser_data_generator=LaserDataReal(), points_allowed_not_on_poly=30, angel_offset=30, wall_len=120):
        self.laser_data_generator = laser_data_generator
        self.points_allowed_not_on_poly = points_allowed_not_on_poly  # point is 1/4 angle
        self.angel_offset = angel_offset
        self.wall_len = wall_len  # decide how many points combined considered a wall

    '''
    @Pre: 
        None
    @Params: 
        None
    @Return Value:
        m,b if wall found else False
    Description: 
        The function calculates the min_wall_deg and max_wall_deg,
        then iterates over each potential sub wall with wall_len between wall_start and wall_end.
    '''

    def detect_wall(self):
        laser_msg = self.laser_data_generator.get_laser_data()
        _, minp1_deg, _, minp2_deg = find_two_closest_points(
            laser_msg)                    # returns angle from the laser view!!
        # minimum angle to look for a wall, at least zero
        min_wall_deg = np.max(
            [np.min([minp1_deg, minp2_deg]) - self.angel_offset, 0])
        # maximum angle to look for a wall, at most 180 (max angel of lidar)
        max_wall_deg = np.min(
            [np.max([minp1_deg, minp2_deg]) + self.angel_offset, 180])
        print("min wall deg: ", min_wall_deg, "max wall deg: ", max_wall_deg)
        # start from right side
        wall_start = int(min_wall_deg * 4)
        # current wall end view
        wall_end = int(wall_start + self.wall_len)
        while wall_end <= max_wall_deg*4-1:
            found_wall, data = self.find_wall_in_range(
                wall_start, wall_end, laser_msg.ranges)
            if found_wall:
                # here data is (m,b)
                return data
            else:
                # here data is first_idx_not_on_poly
                wall_start = data
                wall_end = wall_start + self.wall_len
        return False

    '''
    @ PreCondition:
        None
    @Params:
        *wall_start - first index of range(laser inde range 0-720) to search a wall in 
        *wall_end - last index of range(laser inde range 0-720) to search a wall in 
        *laser_msg - the laser array of distances 
    @Return Value:
        found_wall, data - 
        a boolean if a wall was found:
            - if fails, returns the first index that was failing the wall we tried
            - if succeds, returns a tuple (m,b) of the wall found 
    @Description:
        The function tries to find a wall from the laser's data parameter, goes through the indexes given.
        If a dot is not on the potential wall's line it is added to the 'points_not_on_poly' var, if exceeds,
        self.points_allowed_not_on_poly the potential wall fails. If iterates all points and "most" are on wall line then functions succeeds.
    '''

    def find_wall_in_range(self, wall_start, wall_end, ranges):
        ranges = self.filter_laser_noise(ranges)
        x1, y1 = polar_to_cartesian(
            ranges[wall_start], 270+wall_start/4.0)
        x2, y2 = polar_to_cartesian(
            ranges[wall_end], 270+wall_end/4.0)
        m, b = np.polyfit([x1, x2], [y1, y2], 1)
        first_idx_not_on_poly = wall_start
        first_seen = True
        points_not_on_poly = 0
        for i in range(wall_start, wall_end):
            radius = ranges[i]
            x, y = polar_to_cartesian(radius, 270+i/4.0)
            if not point_on_poly(x, y, m, b):
                points_not_on_poly += 1                  # increment amount of points not on poly
                if first_seen:                          # save first index which is not on the poly
                    first_idx_not_on_poly = i
                    first_seen = False
            if points_not_on_poly >= self.points_allowed_not_on_poly:
                return False, first_idx_not_on_poly
        # return poly of the wall
        return True, (m, b)
    '''
    @ PreCondition:
        None
    @Params:
        *ranges - the laser ranges to filter
    @Return Value:
        filtered ranges with a random value in (30,100) for each value greater than 30
    @Description:
        This function replaces all ranges with value greater than 30 with a random value in (30,100).
    '''

    def filter_laser_noise(self, ranges):
        return map(lambda range: np.min(
            [range, np.random.uniform(30.0, 100.0)]), ranges)
