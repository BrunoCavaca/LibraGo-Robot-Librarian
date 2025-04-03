#!/usr/bin/env python3
import rospy
import smach
import speech_recognition as sr
import numpy as np
import io
from common.tts import TTS

"""
Takes in audio input from user and processes it before passing on to intent detection engine
"""

DIAGNOSTIC_MSG = 'I have been unable to understand what you are saying. I will now restart.'

class Listener(smach.State):

    def __init__(self, mediator,communicationType):
        smach.State.__init__(self, outcomes=['listenSuccess', 'listenFailure'])
        self.tts = TTS()
        self.mediator = mediator
        self.communicationType = communicationType.lower()

    def execute(self, userdata):
        recognizer = sr.Recognizer()
        if self.communicationType == "text":
            try:
                self.tts.execute_tts("BEEP")
                recognized_text = input("Please type your request:")
                print(recognized_text)
                self.mediator.setSpeechDetected(recognized_text)
                return 'listenSuccess'
            
            except Exception as e:
                print(f"Error during text input: {e}")
                return 'listenFailure'
        elif self.communicationType == "file":
            with sr.AudioFile('hello_harry_potter.wav') as source:
                try:
                    # Adjust for ambient noise
                    recognized_text = recognizer.recognize_google(source)
                    self.mediator.setSpeechDetected(recognized_text)
                    print(f"Recognized text: {recognized_text}")
                    return 'listenSuccess'

                except sr.UnknownValueError:
                    self.tts.execute_tts(DIAGNOSTIC_MSG)
                    print("Could not understand audio")
                    return 'listenFailure'

                except sr.RequestError as e:
                    print(f"Error with the speech recognition request: {e}")
                    return 'listenFailure'

        elif self.communicationType == "live":
            # Record live audio from the microphone
            recognizer = sr.Recognizer()
            with sr.Microphone() as source:
                try:
                    print("Adjusting for ambient noise...")
                    recognizer.adjust_for_ambient_noise(source, duration=1)  # Adjusting for ambient noise over 1 second
                    self.tts.execute_tts("BEEP")
                    print("Listening for live audio...")
                    audio = recognizer.listen(source, timeout=2, phrase_time_limit=5)

                    print("Recognizing...")
                    recognized_text = recognizer.recognize_google(audio)
                    self.mediator.setSpeechDetected(recognized_text)
                    print(f"Recognized text: {recognized_text}")
                    return 'listenSuccess'

                except sr.UnknownValueError:
                    self.tts.execute_tts(DIAGNOSTIC_MSG)
                    print("Could not understand audio")
                    return 'listenFailure'

                except sr.RequestError as e:
                    print(f"Error with the speech recognition request: {e}")
                    return 'listenFailure'

        else:
            # If the communicationType is not recognized
            print("Invalid communication type. Expected 'text', 'microphone', or 'live'.")
            self.tts.execute_tts("Invalid communication type. Please check your configuration.")
            return 'listenFailure'
