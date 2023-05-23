#!/usr/bin/python3

import roslib
import time
import rospy

import speech_recognition as sr
from std_msgs.msg import String


class SpeechTranscriber:
    def __init__(self):
        rospy.init_node('speech_transcriber', anonymous=True)
        
        # The documentation is here: https://github.com/Uberi/speech_recognition

        # The main interface to the speech recognition engines
        self.sr = sr.Recognizer()
        
        # These are the methods that are available to us for recognition.
        # Please note that most of them use an internet connection and currently they are using
        # a default API user/pass, so there are restrictions on the number of requests we can make.
        # recognize_google(): Google Web Speech API
        # recognize_google_cloud(): Google Cloud Speech - requires installation of the google-cloud-speech package
        
        # An interface to the default microphone
        self.mic = sr.Microphone()

        self.colors = {"blue", "red", "green", "yellow"}

        self.listening_sub = rospy.Subscriber("listen", String, self.listening_callback)
        self.cylinder_color_pub = rospy.Publisher("cylinder_colors", String, queue_size=10)

        # You can get the list of available devices: sr.Microphone.list_microphone_names()
        # You can set the fault microphone like this: self. mic = sr.Microphone(device_index=3)
        # where the device_index is the position in the list from the first command.

    def listening_callback(self, msg):
        print(msg)
        self.detectCylinderColors(msg.data)

    def recognize_speech(self):
        with self.mic as source:
            self.sr.adjust_for_ambient_noise(source)
            print('SPEAK')
            audio = self.sr.listen(source)
           
        recognized_text = ''
        try:
            recognized_text = self.sr.recognize_google(audio)
        except sr.RequestError as e:
            print('API is probably unavailable', e)
        except sr.UnknownValueError:
            print('Did not manage to recognize anything.')
            
        return recognized_text

    def detectCylinderColors(self, msgtext):
        if msgtext == "listen":
            text = self.recognize_speech()
        else:
            text = msgtext

        cylinderColors = ""

        for color in self.colors:
            if text.find(color) != -1:
                print(color)
                if color == "red":
                    cylinderColors += 'Red '
                if color == "blue":
                    cylinderColors += 'Blue '
                if color == "green":
                    cylinderColors += 'Green '
                if color == "yellow":
                    cylinderColors += 'Yellow '
        print(cylinderColors)
        self.cylinder_color_pub.publish(cylinderColors)

def main():
    speech_recognition = SpeechTranscriber()

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()
    
if __name__ == '__main__':
    main()