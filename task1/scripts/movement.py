#!/usr/bin/python3

import rospy

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from std_msgs.msg import Int8
from task1.msg import FaceID_and_pose
from geometry_msgs.msg import Pose, Twist, Vector3
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
            0: 'exploring',
            1: 'greeting',
            2: 'idle'
        }

        self.state = 0

        self.ps = PoseStamped()
        self.ps.header.frame_id = "map"
        self.ps.pose.orientation.x = 0
        self.ps.pose.orientation.y = 0
        self.ps.pose.orientation.z = 0
        self.ps.pose.orientation.w = 1
        self.ps.pose.position.z = 0.0

        self.iterator = 0
        self.movestatus = 3

        self.faceID = 0
        self.facePos = PoseStamped()
        self.facePos.header.frame_id = "map"
        self.distanceToFace = 0.0


        # self.xs = [-0.75, -1.55, -0.7, -1.2, -1.2, -0.6, -0.05, 0.25, 1.2, 2.05, 2.75, 2.6,   1.8,  1.25, 1.1,  1.1,  2.1,   2.85, 3.35, 2.5, 1.75,  0.85, 0,   -0.25, -0.35, -0.1]
        # self.ys = [-0.45, -0.45,  0.3,  1.25, 2.2,  1.45, 1.6,  2.05, 1.9, 1.75, 1.25, 0.25, -0.25, 0.8, -0.3, -0.9, -0.65, -1.3, -1.7, -2,  -2.15, -2.1, -1.9, -1.4,  -0.7,  -0.25]
        self.xs = [-0.3, 1.85, 3.45, 1.1, 2.75, 0.45, -1.0, -1.05]
        self.ys = [-1.3, -1.5, -1.25, -0.04, 1.8, 2.35, 1.85, 0.15]
        self.tts = TextToSpeach()
        
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1000)
        self.twist_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=1000)
        # self.state_sub = rospy.Subscriber('move_base/status', GoalStatusArray, self.status_callback)
        self.new_face_sub = rospy.Subscriber("new_face_greet", FaceID_and_pose, self.new_face_callback)
        self.movement_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.movement_client.wait_for_server()   




    def move(self):

        if self.state == 0: # exploring
            state = self.movement_client.get_state()
            # print(state)
            if state == 3 or state == 9:
                self.ps.header.stamp = rospy.Time.now()
                self.ps.pose.position.x = self.xs[self.iterator]
                self.ps.pose.position.y = self.ys[self.iterator]
                
                goal = MoveBaseGoal()
                goal.target_pose = self.ps
                #print(goal)
                self.movement_client.send_goal(goal)
                # self.movement_client.wait_for_result()
                self.iterator += 1

            elif state == 4:
                self.ps.header.stamp = rospy.Time.now()
                self.ps.pose.position.x = self.xs[self.iterator]
                self.ps.pose.position.y = self.ys[self.iterator]
                
                goal = MoveBaseGoal()
                goal.target_pose = self.ps
                #print(goal)
                self.movement_client.send_goal(goal)
                self.iterator += 1

        elif self.state == 1: # greeting
            goal = MoveBaseGoal()
            goal.target_pose = self.facePos
            quat = self.facePos.pose.orientation
            (_,_, yaw) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
            
            start_time = rospy.Time().now()
            loop_duration = rospy.Duration(1.0 * self.distanceToFace)

            twist_msg = Twist()
            twist_msg.angular.z = yaw*2
            while rospy.Time().now() < start_time+loop_duration:
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = yaw

                self.twist_pub.publish(twist_msg)

            start_time = rospy.Time().now()
            while rospy.Time().now() < start_time+rospy.Duration(2.0 * self.distanceToFace):
                twist_msg.linear.x = 0.3
                twist_msg.angular.z = 0

                self.twist_pub.publish(twist_msg)

            print('sm pozdravu baje')
            
            #self.movement_client.cancel_all_goals()
            #self.movement_client.wait_for_result()
            # self.movement_client.send_goal_and_wait(goal)
            
            # self.movement_client.wait_for_result()
            # when get there
            self.tts.speak("Hello face number " + str(self.faceID))
            print('greeting', self.faceID)
            self.state = 2

        elif self.state == 2: # drkanje kurca
            self.state = 0

    def new_face_callback(self, msg):
        self.state = 1
        self.faceID = msg.id
        self.facePos = msg.pose
        self.distanceToFace = msg.distanceToFace

    # def status_callback(self, data):
    #     if len(data.status_list) > 0:
    #         self.movestatus = data.status_list[-1].status
    #     self.movestatus = 3 if self.movestatus == 0 or self.movestatus == 4 else self.movestatus


def main():
        robot = Robot()
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            robot.move()
            rate.sleep()


if __name__ == '__main__':
    main()