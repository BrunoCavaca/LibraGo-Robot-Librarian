#!/usr/bin/env python3

import rospy
import actionlib
import smach

from common.controller_manager import ControllerManager
from common.navigate import MoveTIAGo
from common.play_motion import PlayMotion
from control_msgs.msg import JointTrajectoryControllerState

DESK_COORDS = [-3.41,2.50]
MIN_MAX_GRIP = [0.00, 0.70]

# Implement ability to reset head position

class ReturnToDesk(smach.State):
    def __init__(self,mediator):
        smach.State.__init__(self, outcomes=['moveSuccess', 'moveFailure'])
        # Initialize the action client for torso controller
        self.moveRobot = MoveTIAGo()
        self.mediator = mediator
        self.gripperSub = rospy.Subscriber('/gripper_controller/state',JointTrajectoryControllerState,self.gripper_cb)
        self.reachedDestination = False
        self.gripperEmpty = False
        self.playMotion = PlayMotion()

    def execute(self,userdata):


        executed = self.moveRobot.execute_movement(DESK_COORDS,goingRightToLeft= False,facingNorth=False)
        
        if executed:
            self.reachedDestination = True
            if self.gripperEmpty:
                self.set_gripper_state()
            self.mediator.setNavigationStatus() 
            return 'moveSuccess' 
        return 'moveFailure'

    def set_gripper_state(self):
        self.mediator.setGripperStatus()

    def gripper_cb(self,msg):
        if self.reachedDestination:
            if round(msg.actual.positions[0],2)  == 0.00 or round(msg.actual.positions[0],2)  == 0.70:
                self.gripperEmpty = True