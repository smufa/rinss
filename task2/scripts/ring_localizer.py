#!/usr/bin/python3

from tokenize import String
import rospy
import cv2
import numpy as np
import tf2_ros
import message_filters
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PointStamped, Vector3, Point
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA


def distance(p1, p2):
    return np.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2 + (p1.z - p2.z)**2)


class rings:
    def __init__(self, color, colorname, pose):
        self.number_of_rings = 4
        self.sample_size = 15
        self.detectedColor = color
        self.color = colorname
        self.poses = [pose]
        self.detections = 1

    def add(self, pose):
        self.detections += 1
        if len(self.poses) > self.sample_size:
            del self.poses[0]

        self.poses.append(pose)

    def get_average_pose(self):
        xSum = 0
        ySum = 0
        zSum = 0
        
        for p in self.poses:
            if p is not None:
                xSum += p.x
                ySum += p.y
                zSum += p.z
            else: 
                print("NoneType sm dobu")
        
        l = len(self.poses)
        
        newPose = Point()
        newPose.x = xSum/l
        newPose.y = ySum/l
        newPose.z = zSum/l 
        
        return newPose


class The_Ring:

    def __init__(self):
        rospy.init_node('rings', anonymous=True)

        # An object we use for converting images between ROS format and OpenCV format
        self.bridge = CvBridge()

        # A help variable for holding the dimensions of the image
        self.dims = (0, 0, 0)

        #subscriber for timestamp synchronizer
        self.image_sub = message_filters.Subscriber("/arm_camera/rgb/image_raw", Image)
        self.depth_sub = message_filters.Subscriber("/arm_camera/depth/image_raw", Image)
        self.ring_color_sub = rospy.Subscriber("/ring_color", String, self.ring_color_callback)

        ts = message_filters.TimeSynchronizer([self.image_sub, self.depth_sub], 10)
        ts.registerCallback(self.timestamp_callback)

        # Object we use for transforming between coordinate frames
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)

        self.wait_state = False
        self.wait_sub = rospy.Subscriber("wait", Bool, self.wait_status)

        # markers
        self.markers_pub = rospy.Publisher('/ring_markers', MarkerArray, queue_size=10)
        self.prison_pub = rospy.Publisher('/prison', Point, queue_size=10)

        self.marker_array = MarkerArray()
        self.ringColor = ''
        self.prisonLocation = Point()
        self.marker_num = 0
        self.known_rings = []
    
    def ring_color_callback(self, msg):
        self.ringColor = msg.data
        self.foundRings()
        return
    
    def foundRings(self):
        for ring in self.known_rings:
            if ring.detectedColor == self.ringColor:
                self.prisonLocation = ring.get_average_pose()
        
        self.prison_pub.publish(self.prisonLocation)
        return


    def wait_status(self, msg):
        self.wait_state = msg.data

    def get_pose(self,e,dist, color, time):    # e = elipse centers, dist = distance to ellipses
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
        #print("------------------")
        point = Point()
        point.x = point_world.point.x
        point.y = point_world.point.y
        point.z = point_world.point.z

        color_name, color = color

        # loop thru all known_rings
        detected = False
        for i, ring in enumerate(self.known_rings):
            p1 = point
            p2 = ring.get_average_pose()
            distance = np.linalg.norm(np.array([p1.x, p1.y, p1.z]) - np.array([p2.x, p2.y, p2.z]))

            if distance < 0.6:
                detected = True
                self.known_rings[i].add(point)

                if self.known_rings[i].detections%10 == 0:
                    self.refresh_markers(i)

                break

        if not detected:
            print("New ring", len(self.known_rings))
            
            self.known_rings.append(rings(color, color_name, point))
            self.refresh_markers(len(self.known_rings)-1)

        if self.ringColor == "":
            self.foundRings()
            
    def timestamp_callback(self, rgb_image, depth_image):
        if self.wait_state: return
        time = rgb_image.header.stamp
        #print('I got a new image!')
        try:
            cv_image = self.bridge.imgmsg_to_cv2(rgb_image, "bgr8")
        except CvBridgeError as e:
            print(e)
        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        #print(cv_image)

        # Set the dimensions of the image
        self.dims = cv_image.shape

        # Tranform image to gayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Do histogram equlization
        img = cv2.equalizeHist(gray)

        # Binarize the image, there are different ways to do it
        thresh = cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 15, 25)
        # Extract contours
        contours, hierarchy = cv2.findContours(thresh[0:420, :], cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

        # Fit elipses to all extracted contours
        elps = []
        for cnt in contours:
            if cnt.shape[0] >= 20:
                ellipse = cv2.fitEllipse(cnt)
                elps.append(ellipse)


        # Find two elipses with same centers
        candidates = []
        for n in range(len(elps)):
            #print(elps[n])
            #cv2.ellipse(cv_image, elps[n], (0, 255, 0), 2)
            #cv2.imshow("Image window",cv_image)
            #cv2.waitKey(0)

            for m in range(n + 1, len(elps)):
                e1 = elps[n]
                e2 = elps[m]
                dist = np.sqrt(((e1[0][0] - e2[0][0]) ** 2 + (e1[0][1] - e2[0][1]) ** 2))
                #print (dist)
                #print("------------------------")

                if dist < 5:
                    candidates.append((e1,e2))

        #print("Processing is done! found", len(candidates), "candidates for rings")

        """try:
            depth_img = rospy.wait_for_message('/camera/depth/image_raw', Image)
        except Exception as e:
            print(e)"""

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
            # print("Depth: %.2f, nandepth %.2f" % (depth, nandepth))
            if round(depth) == 0 and nandepth < 2.5:
                rings.append(c)
                ring = hsv_img[x_min:x_max,y_min:y_max,0][~mask]
                ring = ring[ring != 0]
                #color = color.astype(np.float)
                #color[x_min:x_max,y_min:y_max] = np.nan
                color = np.nanmean(ring)

                sat = hsv_img[x_min:x_max,y_min:y_max,1][~mask]
                #print(hsv_img[x_min:x_max,y_min:y_max, :].flatten().reshape((-1,3)))
                sat = np.nanmean(sat)
                val = np.median(hsv_img[x_min:x_max,y_min:y_max,2][~mask])
                #print(hu, sat, val)
                if np.isnan(color) and sat == 0.0 and val < 100:
                    name, marker_color = "black", [0, 0, 0, 1]
                elif color > 160 or color < 20:
                    name, marker_color = "red", [1, 0, 0, 1]
                elif color < 75:
                    name, marker_color = "green", [0, 1, 0, 1]
                elif 90 < color < 140:
                    name, marker_color = "blue", [0, 0, 1, 1]
                else: continue
                #print(color, sat)
                
                self.get_pose(e1, nandepth, (name, marker_color), time)
                #plt.show()
                #cv2.imshow("center", color)
                #cv2.waitKey(0)

    def add_marker(self, pose, color, index):
        color = ColorRGBA(*color)
        color.a = 0.7
        new_marker = Marker()
        new_marker.header.stamp = rospy.Time(0)
        new_marker.header.frame_id = 'map'
        new_marker.pose.position.x = pose.x
        new_marker.pose.position.y = pose.y
        new_marker.pose.position.z = pose.z
        new_marker.type = Marker.SPHERE
        new_marker.action = Marker.ADD
        new_marker.frame_locked = False
        new_marker.lifetime = rospy.Time(0)
        new_marker.id = self.marker_num
        new_marker.scale = Vector3(0.3, 0.3, 0.3)
        new_marker.color = color

        self.marker_num += 1
        return new_marker

    def add_text(self, pose, index):
        new_marker = Marker()
        new_marker.header.stamp = rospy.Time(0)
        new_marker.header.frame_id = 'map'
        new_marker.pose.position.x = pose.x
        new_marker.pose.position.y = pose.y
        new_marker.pose.position.z = pose.z
        new_marker.type = Marker.TEXT_VIEW_FACING
        new_marker.action = Marker.ADD
        new_marker.frame_locked = False
        new_marker.text = str(self.known_rings[index].detections)
        new_marker.lifetime = rospy.Time(0)
        new_marker.id = self.marker_num
        new_marker.scale = Vector3(0.2, 0.2, 0.2)
        new_marker.color = ColorRGBA(0, 0, 0, 1)

        self.marker_num += 1
        return new_marker 

    def refresh_markers(self, index):
        delete_marker = Marker()
        delete_marker.header.frame_id = 'map'
        delete_marker.action = Marker.DELETEALL

        self.marker_array.markers.append(delete_marker)

        for i, ring in enumerate(self.known_rings):
            # if ring.detections > 10:
            avg_pos = ring.get_average_pose()
            self.marker_array.markers.append(self.add_marker(avg_pos, ring.color_rgb, i))
            self.marker_array.markers.append(self.add_text(avg_pos, i))

        self.markers_pub.publish(self.marker_array)



def main():
    The_Ring()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()