import unittest
import numpy as np
from laser_points_wall_detector import LaserPointsWallDetector
from laser_data_dummy import LaserDataDummy, LaserMsgDummy
from test_laser_line_tracker import DOT_PERCISION

'''
Description:
    generate random point in (0,30) for laser msg
'''


def generate_noise():
    return np.random.uniform(0, 30)


'''
@Params:
    *start- start index in the laser msg to start filling from radiuses
    *end- end index
    *m,b- the poly to get distances from
@Return Value:
    |end-start| distances from (0,0) - the robot point
Description:
    generate |end-start| distances from the robot on a poly
'''


def generate_distances_on_poly(start, end, m_poly, b_poly):
    distances = []
    origin = np.array((0, 0))
    # origin is (0,0) so y =m_robot * x is the poly of the robot
    b_robot = 0
    for i in range(start, end):
        angel = i/4.0  # angle in degrees of the current point
        m_robot = np.tan(np.deg2rad(angel-90)
                         ) if angel >= 90 else np.tan(np.deg2rad(angel+270))
        # find two lines intersection
        xi = (b_poly-b_robot) / (m_robot-m_poly)  # b1-b2/m2-m1
        yi = m_robot * xi + b_robot
        interscrion = np.array((xi, yi))
        # consider intersections only in the sight direction of the robot
        if xi < 0 or (angel >= 90 and yi < 0) or (angel < 90 and yi > 0):
            distances.append(generate_noise())
        else:
            distances.append(np.linalg.norm(interscrion - origin))
    return distances


class Test_TestLaserPointsWallDetector(unittest.TestCase):

    def test_find_wall_in_range(self):
        distances = generate_distances_on_poly(
            0, 720, -2, 5)  # 720 points on same poly
        detector = LaserPointsWallDetector(
            laser_data_generator=LaserDataDummy())
        found, (m, b) = detector.find_wall_in_range(
            360, 500, LaserMsgDummy(distances).ranges)
        self.assertEquals(found, True)
        self.assertAlmostEquals(m, -2.0, DOT_PERCISION)
        self.assertAlmostEquals(b, 5.0, DOT_PERCISION)

    def test_detect_wall(self):
        # (no wall at all case) generate random 720 points and expect a wall not to be found
        laser = LaserMsgDummy(np.random.uniform(low=0, high=15, size=720))
        # self.assertEqual((False, False), DETECTOR.detect_wall(laser, 100, 150))
        # (wall inside search zone case) generate 100 random, 200 on some line, and 420 random and expect the line of the 200.
        # (wall not in search zone case) generate 100 points on a line and 620 random and expect no wall


if __name__ == '__main__':
    unittest.main()
