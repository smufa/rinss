#!/usr/bin/python3

from tokenize import String
import rospy
import cv2
import numpy as np
import tf
import tf2_ros
import tf2_geometry_msgs
import message_filters
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PointStamped, Vector3, Point, Pose
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

def distance(p1, p2):
    return np.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2 + (p1.z - p2.z)**2)

class Park:

    def __init__(self):
        rospy.init_node('park', anonymous=True)

        # An object we use for converting images between ROS format and OpenCV format
        self.bridge = CvBridge()

        #subscriber for timestamp synchronizer
        self.image_sub = message_filters.Subscriber("/arm_camera/rgb/image_raw", Image)
        self.depth_sub = message_filters.Subscriber("/arm_camera/depth/image_raw", Image)

        self.park_spot_pub = rospy.Publisher('park_spot', Point, queue_size=10)
        self.markers_pub = rospy.Publisher('/park_markers', MarkerArray, queue_size=10)

        ts = message_filters.TimeSynchronizer([self.image_sub, self.depth_sub], 10)
        ts.registerCallback(self.timestamp_callback)

        # Object we use for transforming between coordinate frames
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)

        self.dims = (0, 0, 0)
        self.markersArray = MarkerArray()
    
    def foundRings(self):
        for ring in self.known_rings:
            if ring.detectedColor == self.ringColor:
                self.prisonLocation = ring.get_average_pose()
        
        self.prison_pub.publish(self.prisonLocation)
        return


    def get_pose(self, e, dist, time):    # e = elipse centers, dist = distance to ellipses
        # Calculate the position of the detected ellipse

        k_f = 525 # kinect focal length in pixels
        
        elipse_x = self.dims[1] / 2 - e[0][0]
        elipse_y = self.dims[0] / 2 - e[0][1]

        angle_to_target = np.arctan2(elipse_x,k_f)

        # Get the angles in the base_link relative coordinate system
        x,y = dist*np.cos(angle_to_target), dist*np.sin(angle_to_target)

        # Define a stamped message for transformation - in the "camera rgb frame"
        point_s = PointStamped()
        point_s.point.x = -y
        point_s.point.y = 0
        point_s.point.z = x
        point_s.header.frame_id = "arm_camera_depth_optical_frame"
        point_s.header.stamp = time

        # Get the point in the "map" coordinate system
        try:
            point_world = self.tf_buf.transform(point_s, "map")
            #print("%.2f %.2f %.2f" % (point_world.point.x, point_world.point.y, point_world.point.z))
        except Exception as e:
            #print(e)
            #print("No image yet.")
            return
        # Create a Pose object with the same position
        #print("------------------")self.marker_num
        
        point = Point()
        point.x = point_world.point.x
        point.y = point_world.point.y
        point.z = point_world.point.z

        new_marker = Marker()
        new_marker.header.stamp = rospy.Time(0)
        new_marker.header.frame_id = 'map'
        new_marker.pose.position.x = point.x
        new_marker.pose.position.y = point.y
        new_marker.pose.position.z = point.z
        new_marker.type = Marker.SPHERE
        new_marker.action = Marker.ADD
        new_marker.frame_locked = False
        new_marker.lifetime = rospy.Time(0)
        new_marker.id = 0
        new_marker.scale = Vector3(0.3, 0.3, 0.3)
        new_marker.color = ColorRGBA(1,0,0,1)

        self.markersArray.markers.append(new_marker)
        self.markers_pub.publish(self.markersArray)

        #print('nasu sm parking')
        self.park_spot_pub.publish(point)
            
    def timestamp_callback(self, rgb_image, depth_image):
        time = rgb_image.header.stamp
        #print('I got a new image!')
        try:
            cv_image = self.bridge.imgmsg_to_cv2(rgb_image, "bgr8")
        except CvBridgeError as e:
            print(e)
        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        #print(cv_image)

        self.dims = cv_image.shape
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        img = cv2.equalizeHist(gray)
        _, thresh = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY_INV)
        # thresh = cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 5)
        # cv2.imshow("Image window",thresh)
        # cv2.waitKey(0)
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        contour_image = np.zeros_like(cv_image)

        # Draw contours on the image
        cv2.drawContours(contour_image, contours, -1, (0, 255, 0), 2)

        # Display the result
        # cv2.imshow("Contours", contour_image)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        # Fit elipses to all extracted contours

        elps = []
        for cnt in contours:
            if cnt.shape[0] >= 700:
                ellipse = cv2.fitEllipse(cnt)
                elps.append(ellipse)

        #print(len(elps))      

        # cv2.ellipse(cv_image, elps[0], (0, 255, 0), 2)
        # cv2.imshow("Image window",cv_image)
        # cv2.waitKey(0)

        # Find two elipses with same centers
        candidates = []
        for n in range(len(elps)):
            # for m in range(n + 1, len(elps)):
            #     e1 = elps[n]
            #     e2 = elps[m]
            #     dist = np.sqrt(((e1[0][0] - e2[0][0]) ** 2 + (e1[0][1] - e2[0][1]) ** 2))
            #     cv2.ellipse(cv_image, elps[n], (0, 255, 0), 2)
            #     cv2.imshow("Image window",cv_image)
            #     cv2.waitKey(0)
            #     #print (dist)
            #     #print("------------------------")

            #     if dist < 5:
            candidates.append((elps[n], elps[n]))

        rings = []

        # Extract the depth from the depth image
        depth_image = self.bridge.imgmsg_to_cv2(depth_image, "32FC1")
        for c in candidates:
            # the centers of the ellipses
            e1 = c[0]
            e2 = c[1]

            size = (e1[1][0]+e1[1][1])/2
            center = (e1[0][1], e1[0][0])

            x1 = int(center[0] - size / 2)
            x2 = int(center[0] + size / 2)
            x_min = x1 if x1>0 else 0
            x_max = x2 if x2<cv_image.shape[0] else cv_image.shape[0]

            y1 = int(center[1] - size / 2)
            y2 = int(center[1] + size / 2)
            y_min = y1 if y1 > 0 else 0
            y_max = y2 if y2 < cv_image.shape[1] else cv_image.shape[1]

            di = depth_image[x_min:x_max,y_min:y_max].copy()
            mask = np.isnan(di)
            di[mask] = 0
            depth = np.mean(di)
            #print(depth_image[x_min:x_max,y_min:y_max])
            if depth_image[x_min:x_max,y_min:y_max].size == 0:
                continue
        
            nandepth = np.sqrt(np.nanmedian(depth_image[x_min:x_max,y_min:y_max])**2 - 0.1**2)  
            self.get_pose(e1, nandepth, time)

def main():
    Park()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()