#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import actionlib
from geometry_msgs.msg import Twist, PoseStamped, Point
import mbf_msgs.msg as mbf_msgs
from actionlib_msgs.msg import GoalStatus
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

# Updated goal to move forward by 0.5m before turning right
GOAL = Point(0.5, 0, 0)

class SendGoal:
    def __init__(self):
        print("Inside Send Goal")
        rospy.init_node("send_goal_node")
        self.client = actionlib.SimpleActionClient("move_base_flex/move_base", mbf_msgs.MoveBaseAction)
        self.client.wait_for_server(rospy.Duration(20))
        print("Move detected")
        self.current_goal_idx = 0
        rospy.Timer(rospy.Duration(1), self.send_next_goal)

    def send_next_goal(self, event):
        if self.current_goal_idx == 0:
            # Update the goal to move forward by 0.5m
            self.current_goal_idx += 1
            quaternion = quaternion_from_euler(0, 0, 0)
            move_base_goal = mbf_msgs.MoveBaseGoal()
            move_base_goal.target_pose.header.frame_id = "map"
            move_base_goal.target_pose.header.stamp = rospy.Time.now()
            move_base_goal.target_pose.pose.position.x = GOAL.x
            move_base_goal.target_pose.pose.position.y = GOAL.y
            move_base_goal.target_pose.pose.position.z = 0
            move_base_goal.target_pose.pose.orientation.x = quaternion[0]
            move_base_goal.target_pose.pose.orientation.y = quaternion[1]
            move_base_goal.target_pose.pose.orientation.z = quaternion[2]
            move_base_goal.target_pose.pose.orientation.w = quaternion[3]
            self.client.send_goal(move_base_goal)
            self.client.wait_for_result()
            print("First goal reached")
        else:
            # Update the goal to turn right by 90 degrees
            quaternion = quaternion_from_euler(0, 0, math.pi/2)
            move_base_goal = mbf_msgs.MoveBaseGoal()
            move_base_goal.target_pose.header.frame_id = "map"
            move_base_goal.target_pose.header.stamp = rospy.Time.now()
            move_base_goal.target_pose.pose.position.x = 0
            move_base_goal.target_pose.pose.position.y = 0
            move_base_goal.target_pose.pose.position.z = 0
            move_base_goal.target_pose.pose.orientation.x = quaternion[0]
            move_base_goal.target_pose.pose.orientation.y = quaternion[1]
            move_base_goal.target_pose.pose.orientation.z = quaternion[2]
            move_base_goal.target_pose.pose.orientation.w = quaternion[3]
            self.client.send_goal(move_base_goal)
            self.client.wait_for_result()
            print("Second goal reached")

if __name__ == '__main__':
    SendGoalObj = SendGoal()
    rospy.spin()

