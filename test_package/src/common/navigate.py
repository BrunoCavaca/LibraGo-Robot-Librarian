#!/usr/bin/env python3

import rospy
import math
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class MoveTIAGo():

    def __init__(self):
        self.moveBaseClient = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.moveBaseClient.wait_for_server()

    def execute_movement(self, coordinates,goingRightToLeft = True, facingNorth = False):
        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.get_rostime()
        goal.target_pose.header.frame_id = 'map'

        goal.target_pose.pose.position.x = coordinates[0]
        goal.target_pose.pose.position.y = coordinates[1]
        goal.target_pose.pose.position.z = 0

        goal.target_pose.pose.orientation.x = 0
        goal.target_pose.pose.orientation.y = 0

        if goingRightToLeft:
            goal.target_pose.pose.orientation.z = math.sin(-math.pi / 4)
            goal.target_pose.pose.orientation.w = math.cos(math.pi / 4)
        elif facingNorth:
            goal.target_pose.pose.orientation.z = -3.1
            goal.target_pose.pose.orientation.w = 0
        else:
            goal.target_pose.pose.orientation.z = math.sin(math.pi / 4)
            goal.target_pose.pose.orientation.w = math.cos(math.pi / 4)

        self.moveBaseClient.send_goal(goal)
        self.moveBaseClient.wait_for_result()

        return self.moveBaseClient.get_state() == actionlib.GoalStatus.SUCCEEDED