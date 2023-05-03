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
from task2.msg import ColorAndPose

class rings:
    def __init__(self, color, pose, speak_node):
        self.speak = speak_node
        self.number_of_rings = 4
        self.green_ring = False
        self.sample_size = 15
        self.colors = [color]
        self.poses = [pose]
        self.detections = 1

    def add(self, color, pose):
        if self.detections == 11:
            print(color)
            hsv = cv2.cvtColor(np.uint8([[[color.r, color.g, color.b]]]), cv2.COLOR_RGB2HSV)
            print(hsv)
            self.say_color(hsv[0,0,0], hsv[0,0,1], hsv[0,0,2])

        self.detections += 1
        if len(self.poses) > self.sample_size:
            del self.colors[0]
            del self.poses[0]

        self.colors.append(color)
        self.poses.append(pose)

    def say_color(self, hue, saturation, value):
        if value <= 130 or saturation < 10:
            self.speak.publish('Black ring')
            return

        if 46 <= hue <= 69: # green
            self.speak.publish('Green ring')
            self.green_ring = True
        elif 95 <= hue <= 127: # blue
            self.speak.publish('Blue ring')
        elif hue <= 8 or 170 <= hue: # red
            self.speak.publish('Red ring')
        elif 25 <= hue <= 32: # yellow
            self.speak.publish('Yellow ring')

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

class ring_recognizer:
    def __init__(self):
        rospy.init_node('ring', anonymous=True)

        self.markers_pub = rospy.Publisher('ring_markers', MarkerArray, queue_size=1000)
        self.ring_sub = rospy.Subscriber('ring_detected', ColorAndPose, self.ring_detected_callback)
        self.sound_pub = rospy.Publisher('speak', String, queue_size=1000)
        self.ring_pub = rospy.Publisher('all_rings_detected', Pose, queue_size=1000)
        
        self.known_rings = []
        self.marker_array = MarkerArray()
        self.marker_num = 0
    
    def ring_detected_callback(self, msg):
        
        detected = False
        for i, ring in enumerate(self.known_rings):
            p1 = msg.pose.point
            p2 = ring.get_average_pose()
            distance = np.linalg.norm(np.array([p1.x, p1.y, p1.z]) - np.array([p2.x, p2.y, p2.z]))

            if distance < 0.3:
                detected = True
                self.known_rings[i].add(msg.color, msg.pose.point)
                if self.known_rings[i].detections%10 == 0:
                    self.refresh_markers(i)

        if not detected:
            print("New ring", len(self.known_rings))
            
            self.known_rings.append(rings(msg.color, msg.pose.point, self.sound_pub))
            self.refresh_markers(len(self.known_rings)-1)

        actualRings = 0
        for ring in self.known_rings:
            if ring.detections > 10:
                actualRings += 1

        print(actualRings)
        if actualRings >= self.known_rings[0].number_of_rings:
            for ring in self.known_rings:
                print(ring.green_ring)
                if ring.green_ring == True:
                    avg = ring.get_average_pose()
                    pose = Pose()
                    pose.position.x = avg.x
                    pose.position.y = avg.y
                    pose.position.z = avg.z
                        
                    self.ring_pub.publish(pose)

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
            if ring.detections > 10:
                avg_pos = ring.get_average_pose()
                avg_color = ring.get_average_color()
                self.marker_array.markers.append(self.add_marker(avg_pos, avg_color, i))
                self.marker_array.markers.append(self.add_text(avg_pos, i))

        self.markers_pub.publish(self.marker_array)

def main():
    ring_recog = ring_recognizer()

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()
    
if __name__ == '__main__':
    main()
