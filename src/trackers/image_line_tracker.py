from line_tracker import LineTracker


class ImageLineTracker(LineTracker):
    '''
    @Params:
        *wall_detector- WallDetector
        *dist_tresh- float
        *dist_from_wall - int
    '''

    def __init__(self, wall_detector, dist_thresh=1.0, dist_from_wall=1):
        LineTracker.__init__(self)
        self.wall_detector = wall_detector
        self.dist_thresh = dist_thresh
        self.dist_from_wall = dist_from_wall

    def find_position_by_two_points(self):
        raise NotImplementedError()

    def get_next_position_in_line(self):
        raise NotImplementedError()

    def find_position_in_front_of_wall(self):
        raise NotImplementedError("Is this even needed here? ")
