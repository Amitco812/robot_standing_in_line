from data_objects.laser_data_dummy import LaserMsgDummy
import numpy as np
import unittest
from utils import polar_to_cartesian, point_on_poly, find_orthogonal_line_through_point, dy, dx, find_two_closest_points


DOT_PERCISION = 3
LASER_SIZE_SCAN = 720


class Test_TestUtils(unittest.TestCase):
    def test_polar_to_cartesian_first_quadrant(self):
        inputs = [(1, 45), (2, 37), (3, 70)]
        outputs = [(0.707, 0.707), (1.597, 1.203), (1.02, 2.819)]
        for idx, inp in enumerate(inputs):
            p_out = polar_to_cartesian(inp[0], inp[1])
            p_expected = outputs[idx]
            self.assertAlmostEqual(
                p_out[0], p_expected[0], 0, DOT_PERCISION)
            self.assertAlmostEqual(
                p_out[1], p_expected[1], 0, DOT_PERCISION)

    def test_polar_to_cartesian_second_quadrant(self):
        inputs = [(1, 135), (2, 127), (3, 160)]
        outputs = [(-0.707, 0.707), (-1.203, 1.597), (-2.819, 1.026)]
        for idx, inp in enumerate(inputs):
            p_out = polar_to_cartesian(inp[0], inp[1])
            p_expected = outputs[idx]
            self.assertAlmostEqual(
                p_out[0], p_expected[0], 0, DOT_PERCISION)
            self.assertAlmostEqual(
                p_out[1], p_expected[1], 0, DOT_PERCISION)

    def test_polar_to_cartesian_third_quadrant(self):
        inputs = [(1, 225), (2, 217), (3, 250)]
        outputs = [(-0.707, -0.707), (-1.597, -1.203), (-1.026, -2.819)]

        for idx, inp in enumerate(inputs):
            p_out = polar_to_cartesian(inp[0], inp[1])
            p_expected = outputs[idx]
            self.assertAlmostEqual(
                p_out[0], p_expected[0], 0, DOT_PERCISION)
            self.assertAlmostEqual(
                p_out[1], p_expected[1], 0, DOT_PERCISION)

    def test_polar_to_cartesian_forth_quadrant(self):
        inputs = [(1, 315), (2, 307), (3, 340)]
        outputs = [(0.707, -0.707), (1.203, -1.597), (2.819, -1.026)]

        for idx, inp in enumerate(inputs):
            p_out = polar_to_cartesian(inp[0], inp[1])
            p_expected = outputs[idx]
            self.assertAlmostEqual(
                p_out[0], p_expected[0], 0, DOT_PERCISION)
            self.assertAlmostEqual(
                p_out[1], p_expected[1], 0, DOT_PERCISION)

    def test_dy(self):
        self.assertAlmostEqual(dy(1, 1), 1/np.sqrt(2), DOT_PERCISION)
        self.assertAlmostEqual(dy(2, -3), 1.8973, DOT_PERCISION)

    def test_dx(self):
        self.assertAlmostEqual(dx(1, 1), 1/np.sqrt(2), DOT_PERCISION)
        self.assertAlmostEqual(dx(2, -3), 0.6324, DOT_PERCISION)

    def test_point_on_poly(self):
        self.fail()

    def test_point_on_poly(self):
        points = [(1, 2), (2, 3), (3, 4)]
        polys = [(1, 1), (0.5, 2), (10, -26)]
        for idx, point in enumerate(points):
            x, y = point
            m, b = polys[idx]
            self.assertTrue(point_on_poly(x, y, m, b))

    def test_find_orthogonal_line_through_point(self):
        slopes_and_points = [(2, (1, 2)), (3, (2, 2)), (-2, (4, 5))]
        polys = [(-0.5, 2.5), (-1.0/3, 2.0+2.0/3), (1.0/2, 3)]
        for idx, slope_and_point in enumerate(slopes_and_points):
            m = slope_and_point[0]
            x, y = slope_and_point[1]
            self.assertEqual(
                polys[idx], find_orthogonal_line_through_point(m, x, y))

    def test_find_two_closest_points_main_test_case(self):
        laser = list(np.random.rand(720))
        laser = LaserMsgDummy(list(map(lambda x: 2 + float(x)*29, laser)))
        laser[424] = 1
        laser[456] = 2
        minp1, minp1_deg, minp2, minp2_deg = find_two_closest_points(
            laser)
        self.assertAlmostEqual(minp1, 1, DOT_PERCISION, 'minp1: ' +
                               str(minp1))
        self.assertAlmostEqual(minp2, 2, DOT_PERCISION, 'minp2: ' +
                               str(minp2))
        self.assertAlmostEqual(minp1_deg, 424/4, DOT_PERCISION,
                               'minp1_deg: '+str(minp1_deg))
        self.assertAlmostEqual(minp2_deg, 456/4, DOT_PERCISION,
                               'minp2_deg: '+str(minp2_deg))

    def test_find_two_closest_points_no_start_angel_and_under_thresh(self):
        laser_msg = [0.7, 0.3, 0.2]
        laser_msg *= LASER_SIZE_SCAN/len(laser_msg)
        laser_msg = LaserMsgDummy(laser_msg)
        output = (0.2, 0.5, 0.7, 0)  # (minp1,minp1deg,minp2,minp2deg)
        self.assertEquals(find_two_closest_points(
            laser_msg, start_angel=0), output)

    def test_find_two_closest_points_with_start_angel(self):
        laser_msg = [0.7, 0.3, 0.2, 0.9]
        laser_msg *= LASER_SIZE_SCAN/len(laser_msg)
        laser_msg = LaserMsgDummy(laser_msg)
        output = (0.2, 70.5, 0.7, 70.0)  # (minp1,minp1deg,minp2,minp2deg)
        self.assertEquals(find_two_closest_points(
            laser_msg, start_angel=70), output)


if __name__ == '__main__':
    unittest.main()
