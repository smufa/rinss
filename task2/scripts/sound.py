#!/usr/bin/python3

import sys
import rospy
import dlib
import numpy as np
import cv2

from std_msgs.msg import String
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

class TextToSpeach:
    def __init__(self):
        rospy.init_node('sound_node', anonymous=True)

        self.speak_sub = rospy.Subscriber('speak', String, self.sound_node_callback)

        self.soundhandle = SoundClient()
        rospy.sleep(1)
        self.voice = 'voice_kal_diphone'
    
    def sound_node_callback(self, msg):
        self.soundhandle.say(msg.data, self.voice, 1.0)


def main():
    text_to_speech = TextToSpeach()

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()
    
if __name__ == '__main__':
    main()