#!/usr/bin/python3

import sys
import rospy
import dlib
import numpy as np
import cv2

from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped, Vector3, Point, Pose
from std_msgs.msg import ColorRGBA, String, Bool
from task2.msg import ColorAndPose, RobberLocations

class cylinders:
    def __init__(self, color, pose, speak_node):
        self.speak = speak_node
        self.number_of_cylinders = 4
        self.sample_size = 15
        self.colors = [color]
        self.detectedColor = None
        self.poses = [pose]
        self.detections = 1

    def add(self, color, pose):
        if self.detections == 1:
            hsv = cv2.cvtColor(np.uint8([[[color.r, color.g, color.b]]]), cv2.COLOR_RGB2HSV)
            self.detectedColor = self.detect_color(hsv[0,0,0])
        self.detections += 1

        if len(self.poses) > self.sample_size:
            del self.colors[0]
            del self.poses[0]

        self.colors.append(color)
        self.poses.append(pose)

    def detect_color(self, hue):
        if 46 <= hue <= 69: # green
            return 'Green'
        elif 95 <= hue <= 127: # blue
            return 'Blue'
        elif hue <= 8 or 170 <= hue: # red
            return 'Red'
        elif 25 <= hue <= 32: # yellow
            return 'Yellow'

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

    def get_average_color(self):
        r = 0
        g = 0
        b = 0
        
        for c in self.colors:
            if c is not None:
                r += c.r
                g += c.g
                b += c.b
            else: 
                print("NoneType sm dobu")
        
        l = len(self.colors)
        
        newColor = ColorRGBA()
        newColor.r = r/l/255
        newColor.g = g/l/255
        newColor.b = b/l/255
        newColor.a = 0.7
        
        return newColor

class cylinder_recognizer:
    def __init__(self):
        rospy.init_node('cylinder', anonymous=True)

        self.markers_pub = rospy.Publisher('cylinder_markers', MarkerArray, queue_size=1000)
        self.sound_pub = rospy.Publisher('speak', String, queue_size=1000)
        self.robber_locations_pub = rospy.Publisher('robber_locations', RobberLocations, queue_size=10)

        self.cylinder_sub = rospy.Subscriber('cylinder_detected', ColorAndPose, self.cylinder_detected_callback)
        self.cylinder_colors_sub = rospy.Subscriber('cylinder_colors', String, self.cylinder_colors_callback)

        self.robberLocation = []
        self.cylinderColors = []
        self.known_cylinders = []
        self.marker_array = MarkerArray()
        self.marker_num = 0
    
    def cylinder_colors_callback(self, msg):
        print(msg)
        self.cylinderColors = msg.data.split(" ")
        self.checkFoundCylinders()

    def checkFoundCylinders(self):
        robberLocations = RobberLocations()

        self.robberLocation = []
        for cylinder in self.known_cylinders:
            if cylinder.detectedColor in self.cylinderColors:
                avg = cylinder.get_average_pose()
                self.robberLocation.append(avg)
                robberLocations.locations.append(avg)
        
        self.robber_locations_pub.publish(robberLocations)
        return

    def cylinder_detected_callback(self, msg):

        detected = False
        for i, cylinder in enumerate(self.known_cylinders):
            p1 = msg.pose.point
            p2 = cylinder.get_average_pose()
            distance = np.linalg.norm(np.array([p1.x, p1.y, p1.z]) - np.array([p2.x, p2.y, p2.z]))

            if distance < 0.5:
                detected = True
                self.known_cylinders[i].add(msg.color, msg.pose.point)
                self.refresh_markers(i)

        if not detected:
            print("New cylinder", len(self.known_cylinders))
            
            self.known_cylinders.append(cylinders(msg.color, msg.pose.point, self.sound_pub))
            self.refresh_markers(len(self.known_cylinders)-1)

        if len(self.cylinderColors) != 0:
            print(self.robberLocation)
            self.checkFoundCylinders()

        # actualCylinders = 0
        # for cylinder in self.known_cylinders:
        #     if cylinder.detections > 1:
        #         actualCylinders += 1

    def add_marker(self, pose, color, index):
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
        new_marker.text = str(self.known_cylinders[index].detections)
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

        for i, cylinder in enumerate(self.known_cylinders):
            if cylinder.detections > 1:
                avg_pos = cylinder.get_average_pose()
                avg_color = cylinder.get_average_color()
                self.marker_array.markers.append(self.add_marker(avg_pos, avg_color, i))
                self.marker_array.markers.append(self.add_text(avg_pos, i))

        self.markers_pub.publish(self.marker_array)

def main():
    cylinder_recog = cylinder_recognizer()

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()
    
if __name__ == '__main__':
    main()
