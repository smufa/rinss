#!/usr/bin/python3

import sys
import rospy
import dlib
import cv2
import numpy as np
import face_recognition

from task1.msg import Greet, Face
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Vector3, PoseStamped
from std_msgs.msg import ColorRGBA, Int8


class face_cluster:
    def __init__(self, embedding, pose):
        self.sample_size = 15
        self.embeddings = [embedding]
        self.poses = [pose]
        self.detections = 1
        # self.iterator = 0

    def add(self, embedding, pose):
        self.detections += 1
        if len(self.embeddings) > self.sample_size:
            del self.embeddings[0]
            del self.poses[0]

        self.embeddings.append(embedding)
        self.poses.append(pose)
    
    def get_average_pose(self):
        xSum = 0
        ySum = 0
        zSum = 0
        
        for p in self.poses:
            if p is not None:
                xSum += p.position.x
                ySum += p.position.y
                zSum += p.position.z
            else: 
                print("NoneType sm dobu")
        
        l = len(self.poses)
        
        newPose = Pose()
        newPose.position.x = xSum/l
        newPose.position.y = ySum/l
        newPose.position.z = zSum/l 
        
        return newPose


class face_recognizer:
    def __init__(self):
        rospy.init_node('face_recognizer', anonymous=True)

        self.face_sub = rospy.Subscriber("face_and_pose", Face, self.image_callback)
        self.markers_pub = rospy.Publisher('face_markers', MarkerArray, queue_size=1000)
        self.greet_pub = rospy.Publisher('greet', Greet, queue_size=1000)

        self.bridge = CvBridge()

        self.known_faces = []
        self.marker_array = MarkerArray()
        self.marker_num = 0

    def image_callback(self, msg):
        try:
            cv2_image = self.bridge.imgmsg_to_cv2(msg.faceImage)
        except CvBridgeError as e:
            print(e)
            return

        encoding = face_recognition.face_encodings(cv2_image)
        if len(encoding) != 0:
            encoding = encoding[0]
        else:
            return

        recognized = False
        for i, face_encoding in enumerate(self.known_faces):
            truth_array = np.sum(face_recognition.compare_faces(face_encoding.embeddings, encoding))
            #print("compared faces:", i, face_recognition.compare_faces(face_encoding.embeddings, encoding))

            if truth_array > len(face_encoding.embeddings)*0.5:
                recognized = True
                self.known_faces[i].add(encoding, msg.pose)
                self.refresh_markers(i)

        if not recognized:
            print("New face", len(self.known_faces))
            
            self.known_faces.append(face_cluster(encoding, msg.pose))
            self.refresh_markers(len(self.known_faces)-1)

            msg = Greet()
            msg.id = len(self.known_faces)
            self.greet_pub.publish(msg)


    def add_marker(self, pose, index):
        new_marker = Marker()
        new_marker.header.stamp = rospy.Time(0)
        new_marker.header.frame_id = 'map'
        new_marker.pose = pose
        new_marker.type = Marker.SPHERE
        new_marker.action = Marker.ADD
        new_marker.frame_locked = False
        new_marker.lifetime = rospy.Time(0)
        new_marker.id = self.marker_num
        new_marker.scale = Vector3(0.3, 0.3, 0.3)
        new_marker.color = ColorRGBA(1, 0, 0, 0.7)

        self.marker_num += 1
        return new_marker

    def add_text(self, pose, index):
        new_marker = Marker()
        new_marker.header.stamp = rospy.Time(0)
        new_marker.header.frame_id = 'map'
        new_marker.pose = pose
        new_marker.type = Marker.TEXT_VIEW_FACING
        new_marker.action = Marker.ADD
        new_marker.frame_locked = False
        new_marker.text = str(self.known_faces[index].detections)
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

        for i, faces in enumerate(self.known_faces):
            avg_pos = faces.get_average_pose()
            self.marker_array.markers.append(self.add_marker(avg_pos, i))
            self.marker_array.markers.append(self.add_text(avg_pos, i))

        self.markers_pub.publish(self.marker_array)


def main():
    face_recogn = face_recognizer()

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()
        

if __name__ == '__main__':
    main()