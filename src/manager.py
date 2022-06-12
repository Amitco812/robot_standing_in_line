#!/usr/bin/env python
import rospy
import sys
from line_detector.srv import ser_message,ser_messageRequest
from mocks.speech_service_mock import speech_service_mock
from voice_text_interface.srv import *
from robot_standing_in_line.srv import TrackerMsg, TrackerMsgRequest
from robotican_demos_upgrade.srv import pick_unknown, pick_unknownRequest
from std_srvs.srv import TriggerRequest,Trigger
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from tf.transformations import quaternion_from_euler

TRACKER_SERVICE_NAME = "/tracker_service"
PICK_SERVICE_NAME = "/pick_unknown"
LINE_END_SERVICE = "/line_end_detection"
GIVE_SERVICE_NAME = "/deliver_to_person"


def move(data):
    x, y, yaw = data
    goal = MoveBaseGoal()
    # set up the frame parameters
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    # moving towards the goal*/
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = 0
    goal_orientation = quaternion_from_euler(0, 0, yaw)
    goal.target_pose.pose.orientation.x = goal_orientation[0]
    goal.target_pose.pose.orientation.y = goal_orientation[1]
    goal.target_pose.pose.orientation.z = goal_orientation[2]
    goal.target_pose.pose.orientation.w = goal_orientation[3]
    move_base(goal)


def move_base(goal):
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    # wait for the action server to come up
    while not ac.wait_for_server(rospy.Duration.from_sec(5.0)):
        rospy.loginfo("Waiting for the move_base action server to come up")
    print("goal location: ", goal)
    ac.send_goal(goal)
    ac.wait_for_result(rospy.Duration(60))
    if ac.get_state() == GoalStatus.SUCCEEDED:
        rospy.loginfo("You have reached the end of the queue")
    else:
        rospy.loginfo("The robot failed to reach the end of the queue")




def call_speech_service(is_mock,request):
    if is_mock:
        return speech_service_mock(request)
    else:
        rospy.wait_for_service('speech_to_text')
        speech_to_text_req = rospy.ServiceProxy('speech_to_text', speech_to_text) 
        resp1 = speech_to_text_req(request)  
        print("Responding to the voice command!")   
        return resp1.status


if __name__ == "__main__":
    rospy.init_node("manegar")
    speech_mock = sys.argv[1] == 'true'
    rospy.wait_for_service(LINE_END_SERVICE)
    
    try:
         # === LINE END DETECTION START ===
        line_end_detection = rospy.ServiceProxy(LINE_END_SERVICE,ser_message)
        line_end_resp = line_end_detection(ser_messageRequest(True))
        print("response from line end: ",line_end_resp)
        # ===nnot import name re LINE END DETECTION ENDS ===
    except rospy.ServiceException as e:
        print("line end detection exception: %s" % e)
    
    rospy.wait_for_service(TRACKER_SERVICE_NAME)
    try:
        # === LINE TRACKER START ===
        line_tracker = rospy.ServiceProxy(TRACKER_SERVICE_NAME, TrackerMsg)
        line_tracker(TrackerMsgRequest(True))
        # === LINE TRACKER END ===
        # === SPEECH SYNTHESIS START ===
        resp = call_speech_service(speech_mock,"Bring me coffee please")
        # === SPEECH SYNTHESIS END ===
        # === PICK SERVICE START ===
        rospy.wait_for_service(PICK_SERVICE_NAME)
        pick_service = rospy.ServiceProxy(PICK_SERVICE_NAME, pick_unknown)
        pick_service(pick_unknownRequest())  # '', '', ''))
        # === PICK SERVICE END ===
        # === GIVE SERVICE START ===
        move([2.37,3.56,-1.25])
        give_service = rospy.ServiceProxy(GIVE_SERVICE_NAME, Trigger)
        give_service(TriggerRequest())
        
    except rospy.ServiceException as e:
        print("exception: %s" % e)
   
    
   


    
        


