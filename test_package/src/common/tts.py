#!/usr/bin/env python3

import rospy
import actionlib
from pal_interaction_msgs.msg import TtsAction, TtsGoal, TtsFeedback, TtsResult
from actionlib_msgs.msg import GoalStatus


class TTS():

    def __init__(self):
        self._tts_client = actionlib.SimpleActionClient("/tts", TtsAction)
        self._can_tts = self._tts_client.wait_for_server()

    @staticmethod
    def calculate_tts_time(msg):
        """
        Used to prevent TIAGo from going to the next state in the state machine
        before speech module has finalised operation.
        """
        wordNumber = len(msg.split(" "))
        return (wordNumber / 3.1 )

    def execute_tts(self,msg):
        if self._can_tts:
            rospy.loginfo('Initiating TTS')

            goal = TtsGoal()
            goal.rawtext.text = msg
            goal.rawtext.lang_id = "en_GB"

            # Send the goal to the TTS server
            self._tts_client.send_goal(goal)

            # Wait for the result (a blocking call)
            self._tts_client.wait_for_result()
        else:
            rospy.logwarn('TTS server not available')

    def is_running(self):
        return self._tts_client.get_state()
