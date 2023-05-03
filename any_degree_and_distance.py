#!/usr/bin/env python
__author__ = "divagarn"
import rospy
import math
from geometry_msgs.msg import Twist, Quaternion
from tf.transformations import quaternion_from_euler
from tf import TransformListener
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

class RobotMovement:
    def __init__(self):
        rospy.init_node('robot_movement', anonymous=False)
        self.tf = TransformListener()
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

    def get_current_position(self):
        self.tf.waitForTransform('/odom', '/base_link', rospy.Time(), rospy.Duration(4.0))
        (trans, rot) = self.tf.lookupTransform('/odom', '/base_link', rospy.Time(0))
        return trans[0], trans[1], math.degrees(rot[2])

    def move(self, distance, degree):
        theta = math.radians(degree)
        x, y, _ = self.get_current_position()
        x += distance * math.cos(theta)
        y += distance * math.sin(theta)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        q = Quaternion(*quaternion_from_euler(0, 0, theta))
        goal.target_pose.pose.orientation = q
        self.client.send_goal(goal)
        self.client.wait_for_result()

    def main_run(self):
        while not rospy.is_shutdown():
            try:
                distance, degree = input("Enter distance and degree(ex: 0.5,0) : ").split(',')
                distance = float(distance)
                degree = float(degree)
                self.move(distance, degree)
            except (ValueError, KeyboardInterrupt) as error:
                rospy.logerr("Failed to send the goal: {}".format(str(error)))

if __name__ == '__main__':
    controller = RobotMovement()
    controller.main_run()
