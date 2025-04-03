#!/usr/bin/env python3
import rospy
import actionlib
import smach
from common.controller_manager import ControllerManager
from common.navigate import MoveTIAGo
from common.tts import TTS
from common.play_motion import PlayMotion
from common.genai_api import GenerateAISpeech

# Implement ability to reset head position
class Goodbye(smach.State):

    def __init__(self,mediator):
        smach.State.__init__(self, outcomes=['goodbyeSuccess'])

        self.controllerManager = ControllerManager('torso')
        self.moveRobot = MoveTIAGo()
        self.mediator = mediator
        self.tts = TTS()
        self.playMotion = PlayMotion()
        self.genSpeech = GenerateAISpeech()

    # The 2 is hardcoded so that the listener has ample time to adjust audio retreival
    @staticmethod
    def calculate_tts_time(msg):
        wordNumber = len(msg.split(" "))
        return (wordNumber / 3.1 )

    def communicate_goodbye(self):
        requestedBook = self.mediator.getRequestedBookName()
        if self.mediator.getGripperStatus():
            speech = self.genSpeech.generateSuccessOrFailMessage(bookName = requestedBook,successfulFind=False)
            self.tts.execute_tts(speech)
            rospy.sleep(self.calculate_tts_time(speech))
        else:
            speech = self.genSpeech.generateSuccessOrFailMessage(bookName = requestedBook,successfulFind=True)
            self.tts.execute_tts(speech)
            rospy.sleep(self.calculate_tts_time(speech))

    def execute(self,userdata):
        self.communicate_goodbye()
        self.playMotion.playMotion("home")
        self.mediator.reset()
        return 'goodbyeSuccess'