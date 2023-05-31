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

            # get camera image
            try:
                rgb_image_message = rospy.wait_for_message("/arm_camera/rgb/image_raw", Image)
            except Exception as e:
                print(e)
                return

            # transform camera image to cv2
            try:
                rgb_image = self.bridge.imgmsg_to_cv2(rgb_image_message, "bgr8")
            except CvBridgeError as e:
                print(e)
                return

            # rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
            # cv2_image = cv2.cvtColor(cv2_image, cv2.COLOR_BGR2GRAY)

            face_cascade = cv2.CascadeClassifier('/home/nana/ROS/src/task2/model/haarcascade_frontalface_default.xml')
            faces = face_cascade.detectMultiScale(rgb_image, scaleFactor=1.01, minNeighbors=7)

            offset = 10
            bool_msg = Bool()
            bool_msg.data = True
            
            for (x, y, w, h) in faces:
                adjusted = rgb_image[y-offset:y+h+offset,x-offset:x+w+offset]

                height, width = adjusted.shape[:2]

                # alpha = 3  # contrast:  [0,1) => lower    [1,] => higher
                # beta = 2   # brightness
                # adjusted = cv2.convertScaleAbs(adjusted, alpha, beta)
                
                # cv2.imshow('zaznana faca', adjusted)
                # cv2.waitKey(0)
                # cv2.imshow('Wanted faca', cv2_image)
                # cv2.waitKey(0)

                histogram1 = cv2.calcHist([adjusted], [0], None, [256], [0, 256])
                histogram2 = cv2.calcHist([cv2_image], [0], None, [256], [0, 256])

                # Normalize the histograms (optional)
                histogram1 = cv2.normalize(histogram1, histogram1, 0, 1, cv2.NORM_MINMAX)
                histogram2 = cv2.normalize(histogram2, histogram2, 0, 1, cv2.NORM_MINMAX)

                # Calculate the histogram similarity using the Bhattacharyya coefficient
                similarity = cv2.compareHist(histogram1, histogram2, cv2.HISTCMP_BHATTACHARYYA)

                if similarity < 0.6:
                    bool_msg.data = True
                    # return "The faces match. Reidentified as the same person."
                print("Result:", bool_msg)

                rospy.sleep(2)
                self.face_compare_pub.publish(bool_msg)
            
            self.face_compare_pub.publish(bool_msg)


if __name__ == "__main__":
    fc = face_comparer()
    #time.sleep(.5)
    
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        r.sleep()
