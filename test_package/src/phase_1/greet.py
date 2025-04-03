#!/usr/bin/env python3
import rospy
import smach
from common.tts import TTS
from common.genai_api import GenerateAISpeech

"""
In charge of greeting the individual that is going to be helped
"""
class Greet(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['greetSuccess', 'greetFailure'])
        self.speechGenerator = GenerateAISpeech()
        self.tts = TTS()

    def execute(self,userdata):
        greeting = self.speechGenerator.generateGreeting()
        self.tts.execute_tts(greeting)
        rospy.sleep(self.tts.calculate_tts_time(greeting))
        return 'greetSuccess'