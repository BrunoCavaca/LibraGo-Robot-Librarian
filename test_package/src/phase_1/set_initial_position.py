#!/usr/bin/env python3
import rospy
import actionlib
import smach
from common.controller_manager import ControllerManager
from common.navigate import MoveTIAGo
from common.play_motion import PlayMotion
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal

DESK_COORDS = [-3.41,2.50]


"""
Sets the robot's initial positions
"""
class SetRobot(smach.State):
    def __init__(self, mediator):
        smach.State.__init__(self, outcomes=['moveSuccess', 'moveFailure'])
        # Initialize the action client for torso controller
        self.playMotion = PlayMotion()
        self.headcontrollerManager = ControllerManager('head')
        self.controllerManager = ControllerManager('torso')
        self.gripperControllerManager = ControllerManager('gripper')
        self.moveRobot = MoveTIAGo()
        self.mediator = mediator

    def execute(self,userdata):
        self.playMotion.playMotion("home")
        if self.lower_torso():
            self.reset_gripper()
            self.resetHead()
            executed = self.moveRobot.execute_movement(DESK_COORDS,goingRightToLeft= False,facingNorth=False)
            return 'moveSuccess' if executed else 'moveFailure'

        return 'moveFailure'
    
    def reset_gripper(self):
        jointInformation = {}
        jointInformation['jointName'] = ['gripper_finger_joint']
        jointInformation['positions'] = [0.0]
        return self.gripperControllerManager.execute(jointInformation)
    
    def lower_torso(self):
        jointInformation = {}
        jointInformation['jointName'] = ['torso_lift_joint']
        jointInformation['positions'] = [0.00]
        return self.controllerManager.execute(jointInformation)


    def resetHead(self):
        jointInformation = {}
        jointInformation['jointName'] = ['head_1_joint','head_2_joint']

        jointInformation['positions'] = [0,0]
        self.headcontrollerManager.execute(jointInformation)
        