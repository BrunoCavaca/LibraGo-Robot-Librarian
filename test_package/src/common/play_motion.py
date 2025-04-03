#!/usr/bin/env python3
import rospy
import actionlib
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal


class PlayMotion():

    def __init__(self):
        self.motionClient = actionlib.SimpleActionClient('/play_motion',PlayMotionAction)
        self.motionClient.wait_for_server()
        
    def playMotion(self,motionType):
        goal = PlayMotionGoal()
        goal.motion_name = motionType
        goal.skip_planning = False
        self.motionClient.send_goal_and_wait(goal)
    