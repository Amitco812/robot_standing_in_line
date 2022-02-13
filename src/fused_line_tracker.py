import rospy
from line_tracker import LineTracker
from line_detector.srv import ObjectDetection

class FusedLineTracker(LineTracker):
    def __init__(self):
        self.object_scanning_service_name="object_scanning_service"
        self.line_classes = ['person']

    def find_people(self):
        rospy.wait_for_service(self.object_scanning_service_name)
        object_detection_service = rospy.ServiceProxy(self.object_scanning_service_name, ObjectDetection)

        detection_results = object_detection_service(self.line_classes)

        coords = detection_results.object_coordinates
        
        return coords, detection_results.header