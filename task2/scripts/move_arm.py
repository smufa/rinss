#!/usr/bin/python3

import rospy
import time
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String

class Arm_Mover():
    def __init__(self):

        rospy.init_node('arm_mover', anonymous=True)
        
        self.arm_movement_pub = rospy.Publisher('/turtlebot_arm/arm_controller/command', JointTrajectory, queue_size=1)
        self.arm_user_command_sub = rospy.Subscriber("/arm_command", String, self.new_user_command)

        # Just for controlling wheter to set the new arm position
        self.user_command = None
        self.send_command = False

        # Pre-defined positions for the arm
        self.retract = JointTrajectory()
        self.retract.joint_names = ["arm_shoulder_pan_joint", "arm_shoulder_lift_joint", "arm_elbow_flex_joint", "arm_wrist_flex_joint"]
        self.retract.points = [JointTrajectoryPoint(positions=[0,-1.3,2.2,1],
                                                    time_from_start = rospy.Duration(1))]

        self.extend = JointTrajectory()
        self.extend.joint_names = ["arm_shoulder_pan_joint", "arm_shoulder_lift_joint", "arm_elbow_flex_joint", "arm_wrist_flex_joint"]
        self.extend.points = [JointTrajectoryPoint(positions=[0,0.3,1,0],
                                                    time_from_start = rospy.Duration(1))]
        
        self.right = JointTrajectory()
        self.right.joint_names = ["arm_shoulder_pan_joint", "arm_shoulder_lift_joint", "arm_elbow_flex_joint", "arm_wrist_flex_joint"]
        self.right.points = [JointTrajectoryPoint(positions=[-1.57,0.3,1,0],
                                                    time_from_start = rospy.Duration(1))]

        self.left = JointTrajectory()
        self.left.joint_names = ["arm_shoulder_pan_joint", "arm_shoulder_lift_joint", "arm_elbow_flex_joint", "arm_wrist_flex_joint"]
        self.left.points = [JointTrajectoryPoint(positions=[1.57,0.3,1,0],
                                                    time_from_start = rospy.Duration(1))]
                                                
        self.stand = JointTrajectory()
        self.stand.joint_names = ["arm_shoulder_pan_joint", "arm_shoulder_lift_joint", "arm_elbow_flex_joint", "arm_wrist_flex_joint"]
        self.stand.points = [JointTrajectoryPoint(positions=[0,0,0,0],
                                                    time_from_start = rospy.Duration(1))]

        self.cylinder = JointTrajectory()
        self.cylinder.joint_names = ["arm_shoulder_pan_joint", "arm_shoulder_lift_joint", "arm_elbow_flex_joint", "arm_wrist_flex_joint"]
        self.cylinder.points = [JointTrajectoryPoint(positions=[0,0.3,1,-0.4],
                                                    time_from_start = rospy.Duration(1))]

    def new_user_command(self, data):
        self.user_command = data.data.strip()
        self.send_command = True

    def update_position(self):
        # Only if we had a new command
        if self.send_command:
            if self.user_command == 'retract':
                self.arm_movement_pub.publish(self.retract)
                print('Retracted arm!')
            elif self.user_command == 'extend':
                self.arm_movement_pub.publish(self.extend)
                print('Extended arm!')
            elif self.user_command == 'right':
                self.arm_movement_pub.publish(self.right)
                print('Right-ed arm!')
            elif self.user_command == 'stand':
                self.arm_movement_pub.publish(self.stand)
                print('Stand arm!')
            elif self.user_command == 'wave':
                self.arm_movement_pub.publish(self.right)
                rospy.sleep(1.5)
                self.arm_movement_pub.publish(self.left)
                rospy.sleep(1.5)
                self.arm_movement_pub.publish(self.right)
                rospy.sleep(1.5)
                self.arm_movement_pub.publish(self.stand)
                print('Waved!')
            elif self.user_command == 'cylinder':
                self.arm_movement_pub.publish(self.cylinder)
                print('Cylinder!')
            else:
                print('Unknown instruction:', self.user_command)
                return(-1)
            self.send_command = False

if __name__ == "__main__":
    am = Arm_Mover()
    time.sleep(.5)
    
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        am.update_position()
        r.sleep()
