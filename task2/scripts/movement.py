#!/usr/bin/python3

import rospy

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import os 
import cv2
import numpy as np
import math
from sklearn.cluster import KMeans
import matplotlib.pyplot as plt

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from actionlib_msgs.msg import GoalStatusArray
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from std_msgs.msg import Int8, Bool, String
from task2.msg import GreetingDelta, Greet, RobberLocations
from geometry_msgs.msg import Pose, Twist, Vector3, Point
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from tf2_msgs.msg import TFMessage
from task2.srv import isIDPoster
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import euler_from_quaternion


class TextToSpeach:
    def __init__(self):
        self.soundhandle = SoundClient()
        rospy.sleep(1)
        self.voice = 'voice_kal_diphone'
    
    def speak(self, text, volume = 1.0):
        self.soundhandle.say(text, self.voice, volume)


class Robot:
    def __init__(self):
        rospy.init_node('movement', anonymous=True)

        self.states = {
            'exploring': 0,
            'greeting': 1,
            'idle': 2,
            'parking': 3,
            'fine_parking': 4,
            'cylinder': 5,
            'done': 10
        }

        self.state = self.states['exploring']

        self.ps = PoseStamped()
        self.ps.header.frame_id = "map"
        self.ps.pose.orientation.x = 0
        self.ps.pose.orientation.y = 0
        self.ps.pose.orientation.z = 0
        self.ps.pose.orientation.w = 1
        self.ps.pose.position.z = 0.0

        self.iterator = 0

        self.maxFaces = 10

        self.detectedFaces = 0
        self.detectedPosters = 0
        self.numberOfSuspiciousLocations = 0

        self.faceID = 0
        self.facePos = PoseStamped()
        self.facePos.header.frame_id = "map"

        self.distanceToFace = 0.0
        self.angleToFace = 0.0

        self.cylinders = False
        self.rings = False

        self.havePrisoner = False

        self.prison_location = None
        self.park_spot = None

        self.robot_position = []

        # self.xs = [-0.75, -1.55, -0.7, -1.2, -1.2, -0.6, -0.05, 0.25, 1.2, 2.05, 2.75, 2.6,   1.8,  1.25, 1.1,  1.1,  2.1,   2.85, 3.35, 2.5, 1.75,  0.85, 0,   -0.25, -0.35, -0.1]
        # self.ys = [-0.45, -0.45,  0.3,  1.25, 2.2,  1.45, 1.6,  2.05, 1.9, 1.75, 1.25, 0.25, -0.25, 0.8, -0.3, -0.9, -0.65, -1.3, -1.7, -2,  -2.15, -2.1, -1.9, -1.4,  -0.7,  -0.25]
        # self.xs = [-0.3, 1.85, 3.45, 1.1, 2.75, 0.45, -1.0, -1.05]
        # self.ys = [-1.3, -1.5, -1.25, -0.04, 1.8, 2.35, 1.85, 0.15]
        # -1.3
        # 2.05
        #self.xs = [-1.15, -0.45, 0.55, 1.2, 2.55, 2.25, 3.3, 1.8, -0.05, -0.3]
        #self.ys = [0.1, 1.3, 2.55, 0.5, 1.55, -0.45, -0.9, -1.45, -1.35, -0.1]
        self.xs = []
        self.ys = []
        
        self.tts = TextToSpeach()

        self.bridge = CvBridge()
        
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1000)
        self.twist_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=1000)
        self.listen_command = rospy.Publisher("listen", String, queue_size=10)
        # self.state_sub = rospy.Subscriber('move_base/status', GoalStatusArray, self.status_callback)
        self.new_face_sub = rospy.Subscriber("greet", Greet, self.new_face_callback)
        self.greeting_delta_sub = rospy.Subscriber("greeting_delta", GreetingDelta, self.correction_callback)

        self.robot_position_sub = rospy.Subscriber('tf', TFMessage, self.robot_position_callback)
        self.park_sub = rospy.Subscriber('park_spot', Point, self.park_callback)

        self.arm_command_pub = rospy.Publisher('arm_command', String, queue_size=1000)

        self.cylinder_sub = rospy.Subscriber('all_cylinders_detected', Bool, self.all_cylinders_detected)
        self.ring_sub = rospy.Subscriber('all_rings_detected', Pose, self.all_rings_detected)
        
        self.robber_locations_sub = rospy.Subscriber('robber_locations', RobberLocations, self.robber_locations_callback) # potencialne lokacije roparja
        self.prison_locaiton_sub = rospy.Subscriber('prison', Point, self.prison_location_callback)

        self.movement_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.movement_client.wait_for_server()

        self.detected_posters_sub = rospy.Subscriber('num_of_posters', Int8, self.num_of_posters_callback) # potencialne lokacije roparja
        self.compare_pub = rospy.Publisher('compare', Bool, queue_size=10)
        self.parking_try = 0

        self.get_closest = None

    def num_of_posters_callback(self, msg):
        self.detectedPosters = msg.data
        print(self.detectedPosters)

    def robber_locations_callback(self, msg):
        self.numberOfSuspiciousLocations = len(msg.locations)
        self.suspicious_locations = msg.locations
        
        # if len(msg.locations) == 2 and self.detectedPosters == 3:
            
        #     print(self.suspicious_locations)
        #     self.state = self.states['cylinder']
    
    def prison_location_callback(self, msg):
        # print(self.prison_location)
        self.prison_location = msg

    def park_callback(self, msg):
        self.park_spot = msg

    def robot_position_callback(self, msg):
        self.robot_rotation = msg.transforms[0].transform.rotation
        self.robot_position = msg.transforms[0].transform.translation
       
    def calculate_distance(self, p1, p2):
        return np.linalg.norm(np.array(p1) - np.array(p2))

    def generate_goals(self, goal_count, seed):
        map_data = rospy.wait_for_message("map", OccupancyGrid)     

        scale_factor = 4

        position = map_data.info.origin.position
        resolution = map_data.info.resolution

        # Read the map
        directory = os.path.dirname(__file__)
        whole_map = cv2.imread(os.path.join(
            directory, "../mymap.pgm"), cv2.IMREAD_GRAYSCALE)

        # Scale it down so that KMeans takes less time
        scaled_map = cv2.resize(whole_map, (0, 0), fx=1/scale_factor, fy=1/scale_factor)

        # Make a binary map
        roaming_area = np.where(scaled_map == 255, 1, 0).astype(np.uint8)

        # plt.imshow(roaming_area, cmap='gray')
        # plt.show()

        # Erode the map a little
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        roaming_area = cv2.erode(roaming_area, kernel)

        # plt.imshow(roaming_area, cmap='gray')
        # plt.show()

        # Calculate the centroids
        roaming_area = list(zip(*np.nonzero(roaming_area)))
        kmeans = KMeans(n_clusters=goal_count, random_state=seed).fit(roaming_area)

        # Rescale goals back to original size
        goals = scale_factor * kmeans.cluster_centers_
        goals[:, [1, 0]] = goals[:, [0, 1]]

        # plt.imshow(whole_map, cmap='gray')
        # plt.scatter(goals[:, 0], goals[:, 1], c='r', s=100)
        # plt.show()

        # Move into the navigation frame
        goals[:, 0] = goals[:, 0] * resolution + position.x
        goals[:, 1] = (whole_map.shape[1] - goals[:, 1]) * resolution + position.y

        # goals = [Goal_Pose(x, y, 0, 1) for x, y in goals]

        def get_closest_goal(goal):
            closest = None
            min_dist = 99999
            i = 0
            closestIndex = 0
            for x, y in goals:
                dist = self.calculate_distance([x, y], goal)
                
                if dist < min_dist:
                    closest = [x, y]
                    min_dist = dist
                    closestIndex = i
                
                i += 1

            return closest, closestIndex

        self.get_closest = get_closest_goal

        # Get the shortest path through all goals
        start, i = get_closest_goal([0, 0])
        ordered_goals = [start]
        goals = np.delete(goals, i, 0)

        while len(goals) > 0:
            prev_goal = ordered_goals[-1]
            closest_next, i = get_closest_goal(prev_goal)
            goals = np.delete(goals, i, 0)

            ordered_goals.append(closest_next)

        # Create goal poses including rotations
        final_goals = self.recalculate_angles(ordered_goals)


    def recalculate_angles(self, ordered_goals):
        self.xs = []
        self.ys = []
        for i in range(len(ordered_goals)):
            self.xs.append(ordered_goals[i][0])
            self.ys.append(ordered_goals[i][1])

        print('done')

        # final_goals = []
        # # self.markers = []
        # # self.marker_pub.publish([Marker(header=Header(frame_id="map", stamp=rospy.Time(0)), action=Marker.DELETEALL)])

        # for i in range(len(ordered_goals)):
        #     goal = ordered_goals[i]
        #     next_goal = ordered_goals[(i + 1) % len(ordered_goals)]

        #     x, y = goal.x, goal.y

        #     dx = next_goal.x - x
        #     dy = next_goal.y - y

        #     yaw = np.arctan2(dy, dx)
        #     quat = quaternion_from_euler(0, 0, yaw)
        #     rot_z, rot_w = quat[2], quat[3]

        #     # final_goals.append(Goal_Pose(x, y, rot_z, rot_w))
        #     self.xs.append(x)
        #     self.ys.append(y)
        #     # self.add_arrow_marker(x, y, rot_z, rot_w)

        # return final_goals


    def correction_callback(self, msg):
        self.distanceToFace = msg.distanceToFace
        self.angleToFace = msg.angleToFace
        self.isPoster = msg.isPoster.data
        self.last_face_detection = rospy.Time().now()

    def move(self):

        # self.prison_location = Point(-0.049999, 1.550000, 0.0)

        # self.state = self.states['cylinder']
        # self.suspicious_locations = [Point(-0.9981273449, -0.3515172701, 0.32473115908), Point(2.809075429450, 2.525695187160, 0.32673843826)]

        if self.state == self.states['exploring']:
            state = self.movement_client.get_state()
            # print(state)
            if state == 3 or state == 9 or state == 4:
                self.ps.header.stamp = rospy.Time.now()
                self.ps.pose.position.x = self.xs[self.iterator]
                self.ps.pose.position.y = self.ys[self.iterator]
                quaternion = quaternion_from_euler(0,0,np.random.uniform(0, 2*np.pi))
                self.ps.pose.orientation.x = quaternion[0]
                self.ps.pose.orientation.y = quaternion[1]
                self.ps.pose.orientation.z = quaternion[2]
                self.ps.pose.orientation.w = quaternion[3]
                
                goal = MoveBaseGoal()
                goal.target_pose = self.ps
                #print(goal)
                self.movement_client.send_goal(goal)

                self.iterator += 1
                if self.iterator == len(self.xs):
                    self.loop += 1
                    self.generate_goals(16 + self.loop % 5 , self.loop)
                    self.iterator = 0

                print('Detected posters:',  self.detectedPosters, ' Detected sus loc:', self.numberOfSuspiciousLocations)
                if self.detectedPosters >= 3 and self.numberOfSuspiciousLocations == 2 and self.detectedFaces >= 2:
                    self.state = self.states['cylinder']

                if self.havePrisoner and self.prison_location and self.prison_location.x != 0:
                    self.state = self.states['parking']

        elif self.state == self.states['greeting']:

            duration = rospy.Duration(3) # 1.5

            closest = None
            min_dist = 99999
            i = 0
            closestIndex = 0
            goal = [self.currentFaceLocation.x, self.currentFaceLocation.y]
            for x, y in np.vstack([self.xs, self.ys]).T:
                dist = self.calculate_distance([x, y], goal)*0.3 + self.calculate_distance([x, y], [self.robot_position.x, self.robot_position.y])*0.7
                if dist < min_dist:
                    closest = [x, y]
                    min_dist = dist
                    closestIndex = i
                
                i += 1

            self.movement_client.cancel_all_goals()
            self.movement_client.wait_for_result()


            self.ps.header.stamp = rospy.Time.now()
            self.ps.pose.position.x = closest[0]
            self.ps.pose.position.y = closest[1]
            quaternion = self.quaternion_from_two_points(np.array(closest), np.array([self.currentFaceLocation.x, self.currentFaceLocation.y]))

            self.ps.pose.orientation.x = quaternion[0]
            self.ps.pose.orientation.y = quaternion[1]
            self.ps.pose.orientation.z = quaternion[2]
            self.ps.pose.orientation.w = quaternion[3]
            
            self.iterator -= 1
            goal = MoveBaseGoal()
            goal.target_pose = self.ps
            self.movement_client.send_goal_and_wait(goal)
            self.movement_client.wait_for_result()

            twist_msg = Twist()
            while self.distanceToFace > 0.8:
                try:
                    depth_image_message = rospy.wait_for_message("/camera/depth/image_raw", Image)
                except Exception as e:
                    print(e)
                    continue

                try:
                    depth_image = self.bridge.imgmsg_to_cv2(depth_image_message, "32FC1")
                except CvBridgeError as e:
                    print(e)

                r = 40
                center_x = depth_image.shape[0]//2
                center_y = depth_image.shape[1]//2
                distance = np.nanmean(depth_image[(center_x-r):(center_x+r), (center_y-r):(center_y+r)])

                if distance < 0.7:
                    # msg = Bool()
                    # msg.data = True
                    # rospy.Publisher('delete_last', Bool, queue_size=1).publish(msg)
                    print('u steno se bom zabiu')
                    break
                
                # if rospy.Time().now() > (self.last_face_detection + duration):
                #     msg = Bool()
                #     msg.data = True
                #     rospy.Publisher('delete_last', Bool, queue_size=1).publish(msg)
                #     self.state = self.states['exploring']
                #     print("aborted baby")
                #     return

                twist_msg.angular.z = self.angleToFace * 1.1
                twist_msg.linear.x = 0.2

                self.twist_pub.publish(twist_msg)
            
            # when get there
            isposter = False
            rospy.wait_for_service('isposter')
            try:
                is_poster = rospy.ServiceProxy('isposter', isIDPoster)
                resp = is_poster(self.faceID)
                isposter = resp.ok
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)

            time_now = rospy.Time().now()
            duration = rospy.Duration(1.0)

            while rospy.Time().now() < (time_now + duration):
                twist_msg.angular.z = 0
                twist_msg.linear.x = 0

                self.twist_pub.publish(twist_msg)

            if not isposter:
                self.detectedFaces += 1
                self.tts.speak('Where is the robber?')
                self.movement_client.cancel_all_goals()
                self.listen_command.publish("The cylinder is green or yellow.")
                rospy.wait_for_message("cylinder_colors", String)
                goal = MoveBaseGoal()
                goal.target_pose = self.ps
                self.movement_client.send_goal(goal)          

            if self.faceID >= self.maxFaces:
                self.state = self.states['done']
            else:
                self.state = self.states['idle']

            print('greeting', self.faceID)

        elif self.state == self.states['idle']: # drkanje kurca
            self.state = self.states['exploring']

        elif self.state == self.states['parking']:
            # print(self.prison_location)
            goals = np.vstack([self.xs, self.ys]).T
            print(goals)
            goals = sorted(goals, key=lambda potential: self.calculate_distance(potential, [self.prison_location.x, self.prison_location.y]))
            closest = goals[self.parking_try]
           
            self.movement_client.cancel_all_goals()
            self.movement_client.wait_for_result()


            self.ps.header.stamp = rospy.Time.now()
            self.ps.pose.position.x = closest[0]
            self.ps.pose.position.y = closest[1]
            quaternion = self.quaternion_from_two_points(np.array(closest), np.array([self.prison_location.x, self.prison_location.y]))

            self.ps.pose.orientation.x = quaternion[0]
            self.ps.pose.orientation.y = quaternion[1]
            self.ps.pose.orientation.z = quaternion[2]
            self.ps.pose.orientation.w = quaternion[3]
            
            goal = MoveBaseGoal()
            goal.target_pose = self.ps
            self.movement_client.send_goal_and_wait(goal)

            self.state = self.states['fine_parking']

        elif self.state == self.states['fine_parking']:
            print('sm v fine parking')

            self.arm_command_pub.publish('retract')
            self.arm_command_pub.publish('extend')

            dist = self.calculate_distance([self.prison_location.x,self.prison_location.y,self.prison_location.z], [self.robot_position.x, self.robot_position.y, self.robot_position.z])
            twist_msg = Twist()
            prev_dist = 999
            while dist > 0.2:
                
                try:
                    depth_image_message = rospy.wait_for_message("/camera/depth/image_raw", Image)
                except Exception as e:
                    print(e)
                    continue

                try:
                    depth_image = self.bridge.imgmsg_to_cv2(depth_image_message, "32FC1")
                except CvBridgeError as e:
                    print(e)

                r = 40
                center_x = depth_image.shape[0]//2
                center_y = depth_image.shape[1]//2
                distance = np.nanmean(depth_image[(center_x-r):(center_x+r), (center_y-r):(center_y+r)])

                if self.park_spot == None:
                    if distance < 0.5 or prev_dist < dist:
                        self.parking_try += 1
                        self.state = self.states['parking']
                        return
                    prev_dist = dist
                    dist = self.calculate_distance([self.prison_location.x,self.prison_location.y,self.prison_location.z], [self.robot_position.x, self.robot_position.y, self.robot_position.z])
                else:
                    prev_dist = dist
                    dist = self.calculate_distance([self.park_spot.x,self.park_spot.y,self.park_spot.z], [self.robot_position.x, self.robot_position.y, self.robot_position.z])
                    self.ps.header.stamp = rospy.Time.now()
                    self.ps.pose.position.x = self.park_spot.x
                    self.ps.pose.position.y = self.park_spot.y
                    goal = MoveBaseGoal()
                    goal.target_pose = self.ps
                    self.movement_client.send_goal_and_wait(goal)

                    # self.ps.header.stamp = rospy.Time.now()
                    # self.ps.pose.position.x = self.park_spot.x
                    # self.ps.pose.position.y = self.park_spot.y
                    # goal = MoveBaseGoal()
                    # goal.target_pose = self.ps
                    # self.movement_client.send_goal_and_wait(goal)
                    # twist_msg.linear.x = 0.1
                    # self.twist_pub.publish(twist_msg)
                    break



                #     park_spot = self.park_spot
                #     dist = self.calculate_distance([park_spot.x, park_spot.y, park_spot.z], [self.robot_position.x, self.robot_position.y, self.robot_position.z])
                #     direction = np.array([park_spot.x,park_spot.y]) - np.array([self.robot_position.x, self.robot_position.y])
                #     direction = direction/np.linalg.norm(direction)
                #     angle = math.atan2(direction[1], direction[0])
                #     robot_angle = euler_from_quaternion([self.robot_rotation.x, self.robot_rotation.y, self.robot_rotation.z, self.robot_rotation.w])[2]
                #     twist_msg.angular.z = (angle - robot_angle) * 0.1
                #     print("juhuhu", (angle - robot_angle))
                # print(dist)
               
                twist_msg.linear.x = 0.05
                self.twist_pub.publish(twist_msg)

            self.arm_command_pub.publish('wave')
            print('parkirou')
            self.state = self.states['done']

        elif self.state == self.states['cylinder']:

            print("go to cylinders: ", self.suspicious_locations)
            self.suspicious_locations = sorted(self.suspicious_locations, key=lambda x: x.x, reverse=True)
            # print("go to my assssss: ", self.suspicious_locations)

            for loc in self.suspicious_locations:
                closest = None
                min_dist = 99999
                i = 0
                closestIndex = 0
                goal = [loc.x, loc.y]
                for x, y in np.vstack([self.xs, self.ys]).T:
                    dist = self.calculate_distance([x, y], goal)
                    if dist < min_dist:
                        closest = [x, y]
                        min_dist = dist
                        closestIndex = i

                    i += 1

                self.movement_client.cancel_all_goals()
                self.movement_client.wait_for_result()

                self.ps.header.stamp = rospy.Time.now()
                self.ps.pose.position.x = closest[0]
                self.ps.pose.position.y = closest[1]
                quaternion = self.quaternion_from_two_points(np.array(closest), np.array([loc.x, loc.y]))

                self.ps.pose.orientation.x = quaternion[0]
                self.ps.pose.orientation.y = quaternion[1]
                self.ps.pose.orientation.z = quaternion[2]
                self.ps.pose.orientation.w = quaternion[3]
                
                goal = MoveBaseGoal()
                goal.target_pose = self.ps
                self.movement_client.send_goal_and_wait(goal)
                self.movement_client.wait_for_result()

                duration = rospy.Duration(3) # 1.5
                print("here i am, rock you like a hurricane")

                self.arm_command_pub.publish('cylinder')
                rospy.sleep(3)

                twist_msg = Twist()
                distance = 999
                while distance > 0.6:
                    print(distance)
                    try:
                        depth_image_message = rospy.wait_for_message("/camera/depth/image_raw", Image)
                    except Exception as e:
                        print(e)
                        continue

                    try:
                        depth_image = self.bridge.imgmsg_to_cv2(depth_image_message, "32FC1")
                    except CvBridgeError as e:
                        print(e)

                    r = 40
                    center_x = depth_image.shape[0]//2
                    center_y = depth_image.shape[1]//2
                    distance = np.nanmean(depth_image[(center_x-r):(center_x+r), (center_y-r):(center_y+r)])

                    # p1 = np.array([loc.x, loc.y])
                    # p2 = np.array([self.robot_position.x, self.robot_position.y])
                    # direction = p1 - p2
                    # direction = direction/np.linalg.norm(direction)
                    # angle = math.atan2(direction[1], direction[0])

                    # twist_msg.angular.z = angle / 10
                    twist_msg.linear.x = 0.05

                    self.twist_pub.publish(twist_msg)

                print('sem pri cylindru')

                self.movement_client.cancel_all_goals()
                self.compare_pub.publish(True)
                result = rospy.wait_for_message("compare_result", Bool)
                print(result)

                self.arm_command_pub.publish('stand')

                if result.data: #gremo v prison
                    if self.prison_location:
                        self.tts.speak('I am taking you to the prison now.')

                        self.state  = self.states['parking']
                        break
                    else:
                        self.havePrisoner = True
                        self.state = self.states['exploring']
                else: # gremo do druzga cilindra
                    print('drug cilinder')

                goal = MoveBaseGoal()
                goal.target_pose = self.ps
                self.movement_client.send_goal(goal)   

    def quaternion_from_two_points(self, p1, p2):
        direction = p2 - p1
        direction = direction/np.linalg.norm(direction)
        angle = math.atan2(direction[1], direction[0])
        return quaternion_from_euler(0,0,angle)

    def new_face_callback(self, msg):
        self.state = 1
        self.faceID = msg.id
        self.currentFaceLocation = msg.faceLocation

    def all_cylinders_detected(self, msg):
        self.cylinders = msg.data
        if self.rings:
            self.state = self.states['parking']

    def all_rings_detected(self, msg):
        self.rings = msg
        if self.cylinders:
            self.state = self.states['parking']


def main():
        robot = Robot()
        rate = rospy.Rate(10)
        
        robot.loop = 0
        robot.generate_goals(20, robot.loop)
        while not rospy.is_shutdown():
            robot.move()
            rate.sleep()


if __name__ == '__main__':
    main()
