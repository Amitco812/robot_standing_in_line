from unittest import TestCase
import tracker_service as tracker
import unittest
import numpy as np

class LaserMsgDummy:
        def __init__(self, ranges):
            self.ranges = ranges
            
        def __getitem__(self,key):
            return self.ranges[key]
  
        def __setitem__(self, key, newvalue):
            self.ranges[key]=newvalue
            
class Test(TestCase):

    def test_polar_to_cartesian_first_quadrant(self):
        err_threash = 0.01
        inputs = [(1,45),(2,37),(3,70)]
        outputs = [(0.707,0.707),(1.597,1.203),(1.02,2.819)]
        for idx,inp in enumerate(inputs):
            p_out = tracker.polar_to_cartesian(inp[0],inp[1])
            p_expected = outputs[idx]
            self.assertTrue(np.isclose(p_out[0],p_expected[0],0,err_threash))
            self.assertTrue(np.isclose(p_out[1],p_expected[1],0,err_threash))

    
    def test_polar_to_cartesian_second_quadrant(self):
        err_threash = 0.01
        inputs = [(1,135),(2,127),(3,160)]
        outputs = [(-0.707,0.707),(-1.203,1.597),(-2.819,1.026)]
        for idx,inp in enumerate(inputs):
            p_out = tracker.polar_to_cartesian(inp[0],inp[1])
            p_expected = outputs[idx]
            self.assertTrue(np.isclose(p_out[0],p_expected[0],0,err_threash))
            self.assertTrue(np.isclose(p_out[1],p_expected[1],0,err_threash))

    
    def test_polar_to_cartesian_third_quadrant(self):
        err_threash = 0.01
        inputs = [(1,225),(2,217),(3,250)]
        outputs = [(-0.707,-0.707),(-1.597,-1.203),(-1.026,-2.819)]

        for idx,inp in enumerate(inputs):
            p_out = tracker.polar_to_cartesian(inp[0],inp[1])
            p_expected = outputs[idx]
            self.assertTrue(np.isclose(p_out[0],p_expected[0],0,err_threash))
            self.assertTrue(np.isclose(p_out[1],p_expected[1],0,err_threash))

    def test_polar_to_cartesian_forth_quadrant(self):
        err_threash = 0.01
        inputs = [(1,315),(2,307),(3,340)]
        outputs = [(0.707,-0.707),(1.203,-1.597),(2.819,-1.026)]

        for idx,inp in enumerate(inputs):
            p_out = tracker.polar_to_cartesian(inp[0],inp[1])
            p_expected = outputs[idx]
            self.assertTrue(np.isclose(p_out[0],p_expected[0],0,err_threash))
            self.assertTrue(np.isclose(p_out[1],p_expected[1],0,err_threash))
        

    def test_find_position_by_two_points_first_if(self):
        err_threash = 0.001
        #firstIF!!!!!!!!!!1
        #input1 = 5,12 ,7,14
        #output1 = (10-sqrt(2))/2,(24-sqrt(2))/2
        x,y = tracker.find_position_by_two_points(5,12,7,14)
        self.assertTrue(np.isclose(x,(10-np.sqrt(2))/2,0,err_threash))
        self.assertTrue(np.isclose(y,(24-np.sqrt(2))/2,0,err_threash))
        #input2 = 2,7 ,4,11
        #output2 = (10-sqrt(5))/5,(35-2sqrt(5))/5
        x,y = tracker.find_position_by_two_points(2,7,4,11)
        self.assertTrue(np.isclose(x,(10-np.sqrt(5))/5,0,err_threash))
        self.assertTrue(np.isclose(y,(35-2*np.sqrt(5))/5,0,err_threash))

    def test_find_position_by_two_points_second_if(self):
        err_threash = 0.001
        #secounfIF!!!!!
        #input3 = 0,3 ,3,2
        #output3 = -3sqrt(10)/10,(30+sqrt(10))/10
        x,y = tracker.find_position_by_two_points(0,3,3,2)
        self.assertTrue(np.isclose(x,-3*np.sqrt(10)/10,0,err_threash))
        self.assertTrue(np.isclose(y,(30+np.sqrt(10))/10,0,err_threash))

        #Input4 = 2,3 ,5,1.5
        #Output4 = (10-2sqrt(5))/5 , (15+sqrt(5))/5
        x,y = tracker.find_position_by_two_points(2,3,5,1.5)
        self.assertTrue(np.isclose(x,(10-2*np.sqrt(5))/5,0,err_threash))
        self.assertTrue(np.isclose(y,(15+np.sqrt(5))/5,0,err_threash))

    def test_find_position_by_two_points_third_if(self):
        err_threash = 0.001
        #thirdIF!!!!!!!
        #Input5 = 1.5,1 , 0.5,3
        #Output5 = (15+2sqrt(5))/10 , (5-2sqrt(5))/5
        x,y = tracker.find_position_by_two_points(1.5,1,0.5,3)
        self.assertTrue(np.isclose(x,(15+2*np.sqrt(5))/10,0,err_threash))
        self.assertTrue(np.isclose(y,(5-2*np.sqrt(5))/5,0,err_threash))
        
        #Input6 = 1,0 , 0,4
        #Output6 = (17+sqrt(17))/17 , -4sqrt(17)/17
        x,y = tracker.find_position_by_two_points(1,0,0,4)
        self.assertTrue(np.isclose(x,(17+np.sqrt(17))/17,0,err_threash))
        self.assertTrue(np.isclose(y,(-4*np.sqrt(17))/17,0,err_threash))
    def test_find_position_by_two_points_forth_if(self):
        err_threash = 0.001
        #ForthIf!!!!!
        #Input7 = 2.5,0 , 0,-5
        #Output7 = (25+2sqrt(5))/10,2sqrt(5)/5
        x,y = tracker.find_position_by_two_points(2.5,0,0,-5)
        self.assertTrue(np.isclose(x,(25+2*np.sqrt(5))/10,0,err_threash))
        self.assertTrue(np.isclose(y,(2*np.sqrt(5))/5,0,err_threash))

        #Input8 = 3,-2 , 2,-4
        #Output8 = (15+sqrt(5))/5 , (-10+2sqrt(5))/5
        x,y = tracker.find_position_by_two_points(3,-2,2,-4)
        self.assertTrue(np.isclose(x,(15+np.sqrt(5))/5,0,err_threash))
        self.assertTrue(np.isclose(y,(-10+2*np.sqrt(5))/5,0,err_threash))          

    def test_find_two_closest_points_main_test_case(self):
        laser = list(np.random.rand(720))
        laser = LaserMsgDummy( list( map(lambda x : 2 + float(x)*29 , laser) ))
        laser[424] = 1
        laser[456] = 2
        minp1,minp1_deg,minp2,minp2_deg = tracker.find_two_closest_points(laser)
        self.assertAlmostEqual(minp1,1,5,'minp1: '+str(minp1),0.001)
        self.assertAlmostEqual(minp2,2,5,'minp2: '+str(minp2),0.001)
        self.assertAlmostEqual(minp1_deg,424/4,5,'minp1_deg: '+str(minp1_deg),0.001)
        self.assertAlmostEqual(minp2_deg,456/4,5,'minp2_deg: '+str(minp2_deg),0.001) 
        
    def test_find_two_closest_points_under_thresh(self):
        try :
            laser = list(np.random.rand(720))
            laser = LaserMsgDummy( list( map(lambda x : float(x)*0.1 , laser) ))
        except ValueError:  #raised if `y` is empty.
            pass
        minp1,minp1_deg,minp2,minp2_deg = tracker.find_two_closest_points(laser)
        self.assertEqual(minp1,None,'minp1: '+str(minp1))
        self.assertEqual(minp2,None,'minp2: '+str(minp2))
       
        
        
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
            

        

    def test_point_on_poly(self):
        points = [(1,2),(2,3),(3,4)]
        polys = [(1,1),(0.5,2),(10,-26)]
        for idx,point in enumerate(points):
            x,y = point
            m,b = polys[idx]
            self.assertTrue(tracker.point_on_poly(x,y,m,b))

    def test_detect_wall(self):
        #(no wall at all case) generate random 720 points and expect a wall not to be found
        laser = LaserMsgDummy(np.random.uniform(low=0,high=15,size=720))
        self.assertEqual((False,False),tracker.detect_wall(laser,100,150))
        #(wall inside search zone case) generate 100 random, 200 on some line, and 420 random and expect the line of the 200.
        #(wall not in search zone case) generate 100 points on a line and 620 random and expect no wall

    def test_find_orthogonal_line_through_point(self):
        slopes_and_points = [(2,(1,2)),(3,(2,2)),(-2,(4,5))]
        polys = [(-0.5,2.5),(-1.0/3,2.0+2.0/3),(1.0/2,3)]
        for idx,slope_and_point in enumerate(slopes_and_points):
            m = slope_and_point[0]
            x,y = slope_and_point[1]
            self.assertEqual(polys[idx],tracker.find_orthogonal_line_through_point(m,x,y))


if __name__ == '__main__': 
    unittest.main() 