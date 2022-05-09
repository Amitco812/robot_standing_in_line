from line_tracker import LineTracker
import rospy


class ImageLineTracker(LineTracker):
    '''
    @Params:
        *wall_detector- WallDetector
        *dist_tresh- float
        *dist_from_wall - int
    '''

    def __init__(self, wall_detector , dist_thresh=1.0, dist_from_wall=1):
        LineTracker.__init__(self)
        self.wall_detector = wall_detector
        self.dist_thresh = dist_thresh
        self.dist_from_wall = dist_from_wall
        self.sub = rospy.Subscriber('/yolo4_result/detections',self.collect_messages) # the subscriber to the yolo
        self.messages = [] #messages from the same detection
        self.collecting = True #True if we still collecting messages of same detection

    def collect_messages(self,msg):
        if len(self.messages) == 0: #first message we collect
            self.messages.append(msg)
        elif self.messages[0].stamp == msg.stamp: # the new message is from the same detection
            self.messages.append(msg)
        else:
            #collection ended
            self.sub.unregister()
            self.collecting = False #this is problematic due to thread safety ...


    def find_position_by_two_points(self):
        raise NotImplementedError()

    def get_next_position_in_line(self):
        message = rospy.wait_for_message('')
        current_detections = []
        self.last_detection_id = message.header.stamp
        while message.header.stamp == self.last_detection_id:
            current_detections.append(message.pose)
            message = rospy.wait_for_message('/yolo4_result/detections')
        print(current_detections)
        return current_detections

    def find_position_in_front_of_wall(self):
        raise NotImplementedError("Is this even needed here? ")
