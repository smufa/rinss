#!/usr/bin/python3

import face_recognition
import rospy
import cv2
import dlib

from cv_bridge import CvBridge, CvBridgeError
from task2.msg import Poster
from std_msgs.msg import Bool
from sensor_msgs.msg import Image

class face_comparer:
    def __init__(self):
        rospy.init_node('face_comparer', anonymous = True)

        self.most_wanted_sub = rospy.Subscriber('/most_wanted', Poster, self.most_wanted_callback)
        self.compare_sub = rospy.Subscriber('/compare', Bool, self.compare_callback)

        self.face_compare_pub = rospy.Publisher('/compare_result', Bool, queue_size=1)

        self.bridge = CvBridge()
        self.face_detector = dlib.get_frontal_face_detector()

        self.mostWantedImage = None

    def most_wanted_callback(self, msg):
        print(msg.reward)
        self.mostWantedImage = msg.face

    def compare_callback(self, msg):
        if msg.data and self.mostWantedImage:
            # transform wanted image to cv2
            try:
                cv2_image = self.bridge.imgmsg_to_cv2(self.mostWantedImage)
            except CvBridgeError as e:
                print(e)
                return 
            
            print('most wanted')

            # encode wanted image
            wanted_encoding = face_recognition.face_encodings(cv2_image)
            if len(wanted_encoding) != 0:
                print('nasu wanted faco')

                wanted_encoding = wanted_encoding[0]
            else:
                print('nism nasu wanted faco')

                return

            # get camera image
            try:
                rgb_image_message = rospy.wait_for_message("/arm_camera/rgb/image_raw", Image)
            except Exception as e:
                print(e)
                return

            print('dobim image')
            # transform camera image to cv2
            try:
                rgb_image = self.bridge.imgmsg_to_cv2(rgb_image_message, "bgr8")
            except CvBridgeError as e:
                print(e)
                return

            face_rectangles = self.face_detector(rgb_image, 0)

            cv2.imshow('yooho', rgb_image)
            cv2.waitKey()
            
            for face_rectangle in face_rectangles:
                print('dlib found')
                x1 = face_rectangle.left()
                x2 = face_rectangle.right() 
                y1 = face_rectangle.top()
                y2 = face_rectangle.bottom()

                # Extract region containing face
                face_region = rgb_image[y1:y2,x1:x2]

                cv2.imshow('yooho', face_region)
                cv2.waitKey()

            # encode camera image
            camera_encoding = face_recognition.face_encodings(rgb_image)
            if len(camera_encoding) != 0:
                camera_encoding = camera_encoding[0]
            else:
                print('nism nausu sm faco')
                return

            
            # Compare the face encodings
            results = face_recognition.compare_faces([wanted_encoding], camera_encoding)
            bool_msg = Bool()
            if results[0]:
                bool_msg.data = True
                # return "The faces match. Reidentified as the same person."
            else:
                bool_msg.data = False
                # return "The faces do not match. Not the same person."
            print("Result:", bool_msg)
            self.face_compare_pub.publish(bool_msg)


if __name__ == "__main__":
    fc = face_comparer()
    #time.sleep(.5)
    
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        r.sleep()
