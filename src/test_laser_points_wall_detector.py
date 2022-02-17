from unittest import TestCase
import numpy as np
from laser_points_wall_detector import LaserPointsWallDetector
from laser_data_dummy import LaserMsgDummy

DETECTOR = LaserPointsWallDetector()


class Test_TestLaserPointsWallDetector(TestCase):

    def test_find_wall_in_range(self):
        DETECTOR.detect_wall()
        self.fail()

    def test_detect_wall(self):
        # (no wall at all case) generate random 720 points and expect a wall not to be found
        laser = LaserMsgDummy(np.random.uniform(low=0, high=15, size=720))
        self.assertEqual((False, False), DETECTOR.detect_wall(laser, 100, 150))
        # (wall inside search zone case) generate 100 random, 200 on some line, and 420 random and expect the line of the 200.
        # (wall not in search zone case) generate 100 points on a line and 620 random and expect no wall
