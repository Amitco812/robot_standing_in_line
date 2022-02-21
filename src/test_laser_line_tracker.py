from numpy import dot
from laser_points_wall_detector import LaserPointsWallDetector
import unittest
import numpy as np
from laser_line_tracker import LaserLineTracker
from laser_data_dummy import LaserDataDummy, LaserMsgDummy
from utils import find_two_closest_points


TRACKER = LaserLineTracker(LaserPointsWallDetector(), LaserDataDummy())
DOT_PERCISION = 3


class Test_TestLaserLineTracker(unittest.TestCase):

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

    def test_get_next_position_in_line_shouldnt_move(self):
        laser = list(np.random.rand(720))
        laser = LaserMsgDummy(list(map(lambda x: 1 + float(x)*29, laser)))
        laser.ranges[420] = 1.0
        laser[460] = 0.7
        data_obj = LaserDataDummy(laser)
        tracker = LaserLineTracker(LaserPointsWallDetector(
            laser_data_generator=data_obj), laser_data_generator=data_obj)
        should_move, _ = tracker.get_next_position_in_line()
        self.assertAlmostEquals(should_move, False)
        # self.assertAlmostEqual(x, 0, DOT_PERCISION,
        #                        'x: ' + str(x))
        # self.assertAlmostEqual(y, 0, DOT_PERCISION,
        #                        'y: ' + str(y))
        # self.assertAlmostEqual(yaw, 0, DOT_PERCISION,
        #                        'yaw: ' + str(yaw))
        # x=0.5350 y= yaw= shoulmove=

    def test_get_next_position_in_line_should_move(self):
        laser = list(np.random.rand(720))
        laser = LaserMsgDummy(list(map(lambda x: 7 + float(x)*20, laser)))
        laser.ranges[440] = 1.7
        laser[460] = 2.5
        data_obj = LaserDataDummy(laser)
        tracker = LaserLineTracker(LaserPointsWallDetector(
            laser_data_generator=data_obj), laser_data_generator=data_obj)
        should_move, (x, y, _) = tracker.get_next_position_in_line()
        self.assertEquals(should_move, True)
        #   x1,y1 = polar_to_cartesian(1.7, 440/4+270)
        #   x2,y2 = polar_to_cartesian(2.5,460/4+270)
        #   # (1.597477455336044, 0.5814342436536372)
        #   # (2.2657694675916247, 1.0565456543517489)
        #   m = (0.5814342436536372-1.0565456543517489)/(1.597477455336044-2.2657694675916247)
        #   #0.7109338462606235

        #   b = 1.0565456543517489 - m*2.2657694675916247
        #   #-0.55426654798305
        #   x,y = find_position_by_two_points(x1,y1,x2,y2)
        self.assertAlmostEqual(x, 0.7824539028836085, DOT_PERCISION,
                               'x: ' + str(x))
        self.assertAlmostEqual(y, 0.0020064147156302337, DOT_PERCISION,
                               'y: ' + str(y))

    def test_find_position_in_front_of_wall(self):
        m_wall = 1
        b_wall = 2
        TRACKER.set_p_last_person((2, 2))
        x, y, _ = TRACKER.find_position_in_front_of_wall(m_wall, b_wall)
        self.assertAlmostEqual(x, 1.4242640687119286, DOT_PERCISION,
                               'x: ' + str(x))
        self.assertAlmostEqual(y, 2.5757359312880714, DOT_PERCISION,
                               'y: ' + str(y))
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

if __name__ == '__main__':
    unittest.main()