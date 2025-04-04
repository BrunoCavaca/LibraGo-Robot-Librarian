#!/usr/bin/env python3
import rospy
import smach
from common.controller_manager import ControllerManager
from common.navigate import MoveTIAGo

HEAD_ROTATION_MIN_MAX = (-1.24,1.24)
PATHS = [
    {'x':-2,'y': 0.5}, #face right
    {'x':-2,'y':-3.35},
    {'x':0.00313,'y':-3.35}, #face left
    {'x':0.00313 ,'y':1},
    {'x':2.2 ,'y':1}, #face right
    {'x':2.2 ,'y':-3.35}
    ]


"""
1.Moves the robot through the library
2. Moves its head s.t it is facing bookshelves at all times
3. Moves the head slightly downwards to be able to see all books
"""
 
class SetRobotAlignment(smach.State):


    def __init__(self, mediator):
        smach.State.__init__(self, outcomes=['navigationSuccess', 'navigationFailure'])
        self.controllerManager = ControllerManager('head')
        self.movementManager = MoveTIAGo()
        self.jointInformation = {}
        self.headDirection = "RIGHT"
        self.orientationFacingLeft = True
        self.mediator = mediator


    # Sets head facing in a downwards position
    def setHeadOrientation(self):
        self.jointInformation['jointName'] = ['head_1_joint','head_2_joint']
        self.jointInformation['positions'] = [0.00,-0.37]
        self.controllerManager.execute(self.jointInformation)
        self.jointInformation.clear()

    def setHeadDirection(self):
        self.jointInformation['jointName'] = ['head_1_joint','head_2_joint']

        if self.headDirection == "RIGHT":
            self.jointInformation['positions'] = [HEAD_ROTATION_MIN_MAX[0],-0.37]
        elif self.headDirection == "LEFT":
            self.jointInformation['positions'] = [HEAD_ROTATION_MIN_MAX[1],-0.37]

        self.controllerManager.execute(self.jointInformation)
        self.jointInformation.clear()

    def execute(self,userdata):
        
        self.setHeadOrientation()
        self.setHeadDirection()
        for idx, waypoint in enumerate(PATHS,start= 1):
            print(self.mediator.getDetectionStatus())
            if self.mediator.getDetectionStatus():
                break
            x, y = waypoint['x'], waypoint['y']
            if idx % 2 != 0 and idx != 1:
                self.headDirection = "LEFT" if self.headDirection == "RIGHT" else "RIGHT"
                self.orientationFacingLeft = not self.orientationFacingLeft

            self.setHeadDirection()

            if not self.movementManager.execute_movement([x,y],goingRightToLeft= self.orientationFacingLeft):
                return 'moveFailure'
            rospy.sleep(0.25)
        self.mediator.setNavigationStatus()
        return 'navigationSuccess'