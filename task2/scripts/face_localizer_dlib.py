#!/usr/bin/python3

import sys
import rospy
import dlib
import cv2
import sys
import numpy as np
import tf2_geometry_msgs
import tf2_ros
import pytesseract
import re

#import matplotlib.pyplot as plt
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, Vector3, Pose, Quaternion
from cv_bridge import CvBridge, CvBridgeError
from task2.msg import Face, GreetingDelta, Poster

from tf.transformations import quaternion_from_euler

class face_localizer:
    def __init__(self):
        rospy.init_node('face_detector', anonymous=True)

        # An object we use for converting images between ROS format and OpenCV format
        self.bridge = CvBridge()

        # The function for performin HOG face detection
        self.face_detector = dlib.get_frontal_face_detector()

        # A help variable for holding the dimensions of the image
        self.dims = (0, 0, 0)

        # Publiser for the visualization markers
        self.face_pub = rospy.Publisher('captured_face', Image, queue_size=1000)
        self.face_and_pose_pub = rospy.Publisher('face_and_pose', Face, queue_size=1000)

        #Posteer
        self.poster_pub = rospy.Publisher('poster', Poster, queue_size=1)

        # Publisher for greeting delta
        self.greet_pub = rospy.Publisher('greeting_delta', GreetingDelta, queue_size=1000)

        # Object we use for transforming between coordinate frames
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)

    def get_pose(self,coords,dist,stamp):
        # Calculate the position of the detected face

        k_f = 554 # kinect focal length in pixels

        x1, x2, y1, y2 = coords

        face_x = self.dims[1] / 2 - (x1+x2)/2.
        face_y = self.dims[0] / 2 - (y1+y2)/2.

        angle_to_target = np.arctan2(face_x,k_f)

        # Get the angles in the base_link relative coordinate system
        x, y = dist*np.cos(angle_to_target), dist*np.sin(angle_to_target)

        ### Define a stamped message for transformation - directly in "base_link"
        #point_s = PointStamped()
        #point_s.point.x = x
        #point_s.point.y = y
        #point_s.point.z = 0.3
        #point_s.header.frame_id = "base_link"
        #point_s.header.stamp = rospy.Time(0)

        # Define a stamped message for transformation - in the "camera rgb frame"
        point_s = PointStamped()
        point_s.point.x = -y
        point_s.point.y = 0
        point_s.point.z = x
        point_s.header.frame_id = "camera_rgb_optical_frame"
        point_s.header.stamp = stamp

        # Get the point in the "map" coordinate system
        try:
            point_world = self.tf_buf.transform(point_s, "map")

            # Create a Pose object with the same position
            pose = Pose()
            pose.position.x = point_world.point.x
            pose.position.y = point_world.point.y
            pose.position.z = point_world.point.z
            pose.orientation = Quaternion(*quaternion_from_euler(0, 0, angle_to_target))
        except Exception as e:
            print(e)
            pose = None

        return pose, angle_to_target
    
    def find_faces(self):

        # Get the next rgb and depth images that are posted from the camera
        try:
            rgb_image_message = rospy.wait_for_message("/camera/rgb/image_raw", Image)
        except Exception as e:
            print(e)
            return 0

        try:
            depth_image_message = rospy.wait_for_message("/camera/depth/image_raw", Image)
        except Exception as e:
            print(e)
            return 0

        # Convert the images into a OpenCV (numpy) format

        try:
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_image_message, "bgr8")
        except CvBridgeError as e:
            print(e)

        try:
            depth_image = self.bridge.imgmsg_to_cv2(depth_image_message, "32FC1")
        except CvBridgeError as e:
            print(e)

        # Set the dimensions of the image
        self.dims = rgb_image.shape

        # Tranform image to gayscale
        #gray = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)

        # Do histogram equlization
        #img = cv2.equalizeHist(gray)

        # Detect the faces in the image
        face_rectangles = self.face_detector(rgb_image, 0)

        # For each detected face, extract the depth from the depth image
        for face_rectangle in face_rectangles:
            print('Faces were detected')
            self.find_text(rgb_image, face_rectangle)

            # The coordinates of the rectanle
            x1 = face_rectangle.left()
            x2 = face_rectangle.right()
            y1 = face_rectangle.top()
            y2 = face_rectangle.bottom()

            # Extract region containing face
            face_region = rgb_image[y1:y2,x1:x2]

            # Find the distance to the detected face
            face_distance = float(np.nanmean(depth_image[y1:y2,x1:x2]))
            try:
                face_image = self.bridge.cv2_to_imgmsg(face_region)
                self.face_pub.publish(face_image)
            except CvBridgeError as e:
                print(e)

            print('Distance to face', face_distance)

            # Get the time that the depth image was recieved
            depth_time = depth_image_message.header.stamp

            # Find the location of the detected face
            pose, angle = self.get_pose((x1,x2,y1,y2), face_distance, depth_time)

            if pose is not None:
                msg = GreetingDelta()
                msg.angleToFace = angle
                msg.distanceToFace = face_distance

                self.greet_pub.publish(msg)

                msg = Face()
                msg.id = 0
                msg.faceImage = face_image
                msg.pose = pose

                self.face_and_pose_pub.publish(msg)

    def find_text(self, image, face):
        # Convert the image to grayscale
        img_out = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Option 1 - use adaptive thresholding
        img_out = cv2.adaptiveThreshold(img_out,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,5)
        
        # Use Otsu's thresholding
        #ret,img_out = cv2.threshold(img_out,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    
        # Extract text from image
        text = pytesseract.image_to_string(img_out, config = '--psm 11')
        
        
        # Remove any whitespaces from the left and right
        text = text.strip()

        color = re.search('BLACK|GREEN', text)
        print(color.group(0))

        btc = int(re.search('d+\.\d+|\d+ \d+|\d+', re.search('(\d+\.\d+|\d+ \d+|\d+) (?:B\w*)', text).group(0)).group(0).replace(' ', '').replace(',', ''))
        print(btc)
        msg = Poster()
        msg.color = color
        msg.reward = btc
        msg.face = self.bridge.cv2_imgmsg(face) 

    def depth_callback(self,data):

        try:
            depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
        except CvBridgeError as e:
            print(e)

        # Do the necessairy conversion so we can visuzalize it in OpenCV
        
        image_1 = depth_image / np.nanmax(depth_image)
        image_1 = image_1*255
        
        image_viz = np.array(image_1, dtype=np.uint8)

        #cv2.imshow("Depth window", image_viz)
        #cv2.waitKey(1)

        #plt.imshow(depth_image)
        #plt.show()


def main():
        face_finder = face_localizer()

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            face_finder.find_faces()
            rate.sleep()

        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
