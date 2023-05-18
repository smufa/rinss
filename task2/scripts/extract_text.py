#!/usr/bin/python3

import roslib
import sys
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import ColorRGBA
import pytesseract

dictm = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

# The object that we will pass to the markerDetect function
params =  cv2.aruco.DetectorParameters_create()

print(params.adaptiveThreshConstant) 
print(params.adaptiveThreshWinSizeMax)
print(params.adaptiveThreshWinSizeMin)
print(params.minCornerDistanceRate)
print(params.adaptiveThreshWinSizeStep)

# To see description of the parameters
# https://docs.opencv.org/3.3.1/d1/dcd/structcv_1_1aruco_1_1DetectorParameters.html

# You can set these parameters to get better marker detections
params.adaptiveThreshConstant = 25
adaptiveThreshWinSizeStep = 2


class DigitExtractor:
    def __init__(self):
        rospy.init_node('image_converter', anonymous=True)

        # An object we use for converting images between ROS format and OpenCV format
        self.bridge = CvBridge()

        # Subscribe to the image and/or depth topic
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)

    def image_callback(self,data):
        # print('Iam here!')

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            
        print("Will try to process the image...")


        
        ################################################
        #### Extraction of digits starts here
        ################################################
        
        # Convert the image to grayscale
        img_out = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # Option 1 - use ordinairy threshold the image to get a black and white image
        #ret,img_out = cv2.threshold(img_out,100,255,0)

        # Option 1 - use adaptive thresholding
        img_out = cv2.adaptiveThreshold(img_out,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,5)
        
        # Use Otsu's thresholding
        #ret,img_out = cv2.threshold(img_out,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        
        # Pass some options to tesseract
        config = '--psm 11'
        
        # Visualize the image we are passing to Tesseract
        cv2.imshow('Warped image',img_out)
        cv2.waitKey(1)
    
        # Extract text from image
        text = pytesseract.image_to_string(img_out, config = config)
        
        # Check and extract data from text
        print('Extracted>>',text)
        
        # Remove any whitespaces from the left and right
        text = text.strip()
        
#             # If the extracted text is of the right length
        if len(text)==2:
            x=int(text[0])
            y=int(text[1])
            print('The extracted datapoints are x=%d, y=%d' % (x,y))
        else:
            print('The extracted text has is of length %d. Aborting processing' % len(text))



def main(args):

    de = DigitExtractor()

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
