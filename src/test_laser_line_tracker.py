from numpy import dot
from laser_points_wall_detector import LaserPointsWallDetector
from unittest import TestCase
import numpy as np
from laser_line_tracker import LaserLineTracker
from laser_data_dummy import LaserDataDummy, LaserMsgDummy
from utils import find_two_closest_points


TRACKER = LaserLineTracker(LaserPointsWallDetector(), LaserDataDummy())
DOT_PERCISION = 3


class Test_TestLaserLineTracker(TestCase):

    def test_find_position_by_two_points_first_if(self):

        x, y = TRACKER.find_position_by_two_points(5, 12, 7, 14)
        self.assertAlmostEqual(x, (10-np.sqrt(2))/2, 0, DOT_PERCISION)
        self.assertAlmostEqual(y, (24-np.sqrt(2))/2, 0, DOT_PERCISION)

        x, y = TRACKER.find_position_by_two_points(2, 7, 4, 11)
        self.assertAlmostEqual(x, (10-np.sqrt(5))/5, 0, DOT_PERCISION)
        self.assertAlmostEqual(y, (35-2*np.sqrt(5))/5, 0, DOT_PERCISION)

    def test_find_position_by_two_points_second_if(self):

        x, y = TRACKER.find_position_by_two_points(0, 3, 3, 2)
        self.assertAlmostEqual(x, -3*np.sqrt(10)/10, 0, DOT_PERCISION)
        self.assertAlmostEqual(y, (30+np.sqrt(10))/10, 0, DOT_PERCISION)

        x, y = TRACKER.find_position_by_two_points(2, 3, 5, 1.5)
        self.assertAlmostEqual(x, (10-2*np.sqrt(5))/5, 0, DOT_PERCISION)
        self.assertAlmostEqual(y, (15+np.sqrt(5))/5, 0, DOT_PERCISION)

    def test_find_position_by_two_points_third_if(self):

        x, y = TRACKER.find_position_by_two_points(1.5, 1, 0.5, 3)
        self.assertAlmostEqual(x, (15+2*np.sqrt(5))/10, 0, DOT_PERCISION)
        self.assertAlmostEqual(y, (5-2*np.sqrt(5))/5, 0, DOT_PERCISION)

        x, y = TRACKER.find_position_by_two_points(1, 0, 0, 4)
        self.assertAlmostEqual(x, (17+np.sqrt(17))/17, 0, DOT_PERCISION)
        self.assertAlmostEqual(y, (-4*np.sqrt(17))/17, 0, DOT_PERCISION)

    def test_find_position_by_two_points_forth_if(self):

        x, y = TRACKER.find_position_by_two_points(2.5, 0, 0, -5)
        self.assertAlmostEqual(x, (25+2*np.sqrt(5))/10, 0, DOT_PERCISION)
        self.assertAlmostEqual(y, (2*np.sqrt(5))/5, 0, DOT_PERCISION)

        x, y = TRACKER.find_position_by_two_points(3, -2, 2, -4)
        self.assertAlmostEqual(x, (15+np.sqrt(5))/5, 0, DOT_PERCISION)
        self.assertAlmostEqual(y, (-10+2*np.sqrt(5))/5, 0, DOT_PERCISION)

    def test_get_next_position_in_line(self):
        laser = list(np.random.rand(720))
        laser = LaserMsgDummy(list(map(lambda x: 1 + float(x)*29, laser)))
        laser.ranges[420] = 1.0
        laser[460] = 0.7
        data_obj = LaserDataDummy(laser)
        tracker = LaserLineTracker(LaserPointsWallDetector(
            laser_data_generator=data_obj), laser_data_generator=data_obj)
        should_move, (x, y, yaw) = tracker.get_next_position_in_line()
        self.assertAlmostEqual(x, 0, DOT_PERCISION,
                               'x: '+str(x))
        self.assertAlmostEquals(should_move, False)
        self.assertAlmostEqual(y, 0, DOT_PERCISION,
                               'y: '+str(y))
        self.assertAlmostEqual(yaw, 0, DOT_PERCISION,
                               'yaw: '+str(yaw))

    def test_no_people_in_line(self):
        self.fail()

        # def test_scan_callback_two_people(self):
    #     try:
    #         tracker.scan_callback_two_people(1,80,2,88)
    #     except Exception as e:
    #         self.assertEqual( str(e) , '''('min wall deg: ', 70, 'max wall deg: ', 180)
    #         ('m: ', 2, 'x: ', 1, 'y: ', 2)
    #         ('pos1: ', 5, ',', 12, ' pos2: ', 7, ' , ', 14)
    #         ('pos1: ', 2, ',', 7, ' pos2: ', 4, ' , ', 11)
    #         ('minp1: ', 1.0, ' minp1_deg: ', 106)
    #         ('minp2: ', 2.0, ' minp2_deg: ', 114)''' , str(e))
