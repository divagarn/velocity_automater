#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import mbf_msgs.msg as mbf_msgs
from math import cos, sin, radians
import tf

class RobotController:
    def __init__(self):
        rospy.init_node('move_base_flex_test')
        self.client = actionlib.SimpleActionClient('/move_base_flex/move_base', mbf_msgs.MoveBaseAction)
        self.client.wait_for_server()
        self.listener = tf.TransformListener()

    def move_forward(self, distance):
        # Get the current position and orientation of the robot
        self.listener.waitForTransform('/map', '/base_link', rospy.Time(), rospy.Duration(4.0))
        (position, orientation) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        current_x, current_y = position[0], position[1]
        _, _, current_yaw = tf.transformations.euler_from_quaternion(orientation)

        # Calculate the goal position for moving forward
        goal_x = current_x + distance * cos(current_yaw)
        goal_y = current_y + distance * sin(current_yaw)

        # Send the goal to move forward
        goal = mbf_msgs.MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.pose.position.x = goal_x
        goal.target_pose.pose.position.y = goal_y
        goal.target_pose.pose.orientation.w = 1.0
        self.client.send_goal(goal)
        self.client.wait_for_result()

        # Update current position after moving forward
        self.listener.waitForTransform('/map', '/base_link', rospy.Time(), rospy.Duration(4.0))
        (position, orientation) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        current_x, current_y = position[0], position[1]

    def rotate_right(self):
        # Get the current orientation of the robot
        self.listener.waitForTransform('/map', '/base_link', rospy.Time(), rospy.Duration(4.0))
        (position, orientation) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        _, _, current_yaw = tf.transformations.euler_from_quaternion(orientation)

        # Calculate the goal orientation for rotating right
        goal_yaw = current_yaw - radians(90)

        # Send the goal to rotate right
        goal = mbf_msgs.MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.pose.position.x = position[0]
        goal.target_pose.pose.position.y = position[1]
        goal.target_pose.pose.orientation.z = sin(goal_yaw / 2.0)
        goal.target_pose.pose.orientation.w = cos(goal_yaw / 2.0)
        self.client.send_goal(goal)
        self.client.wait_for_result()

if __name__ == '__main__':
    robot_controller = RobotController()
    robot_controller.move_forward(0.5)
    robot_controller.rotate_right()
