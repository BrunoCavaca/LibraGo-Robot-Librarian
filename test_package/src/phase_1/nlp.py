#!/usr/bin/env python3
import rospy
import smach
from common.genai_api import GenerateAISpeech
from common.tts import TTS

"""
In charge of intent discovery- detecting the book in the user's inputted speech
"""

SUCCESS_PROMPT = "I have heard the following book {book}. I will now go and try to find it for you."

class RetrieveBooks(smach.State):
    def __init__(self,mediator):
        smach.State.__init__(self,
                             outcomes=['bookObtained','listenSuccess','listenFailure'],
        )
        self.tts = TTS()
        self.mediator = mediator
        self.speechGenerator = GenerateAISpeech()


    def execute(self, userdata):
        speechDetection = self.mediator.getSpeechDetected()
        response = self.speechGenerator.generateResponse(speechDetection)
        if response.startswith("WANTS BOOK:"):
            bookName = response.split("WANTS BOOK:")[1]
            self.tts.execute_tts(SUCCESS_PROMPT.format(book = bookName))
            rospy.sleep(self.tts.calculate_tts_time(SUCCESS_PROMPT.format(book = bookName))/3)            
            self.mediator.setRequestedBookName(bookName)
            self.mediator.turnOffConversationalMode()
            return 'bookObtained'
        elif response != "INVALID ARGS":
            self.speechGenerator.addToUserConversationHistory(speechDetection,wasUser= True)
            self.speechGenerator.addToUserConversationHistory(response,wasUser=False)
            self.tts.execute_tts(response)
            rospy.sleep(self.tts.calculate_tts_time(response)/3)
            self.mediator.turnOnConversationalMode()
            return 'listenSuccess'
        else:
            self.produceErrorMessage()
            return 'listenFailure'

    def produceErrorMessage(self):
        failMessage = "It seems like I was not able to find that book or you may have said too many books. Please try again"
        self.tts.execute_tts(failMessage)
