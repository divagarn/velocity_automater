#!/usr/bin/env python

import rospy
import actionlib
import math
from geometry_msgs.msg import PoseStamped
# from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import mbf_msgs.msg as mbf_msgs
from tf.transformations import euler_from_quaternion, quaternion_from_euler

GOALS_FILE = "direction.txt"


def create_move_base_goal(x, y, angle):
    quaternion = quaternion_from_euler(0, 0, angle)
    move_base_goal = mbf_msgs.MoveBaseGoal()
    move_base_goal.target_pose.header.frame_id = "map"
    move_base_goal.target_pose.header.stamp = rospy.Time.now()
    move_base_goal.target_pose.pose.position.x = x
    move_base_goal.target_pose.pose.position.y = y
    move_base_goal.target_pose.pose.position.z = 0
    move_base_goal.target_pose.pose.orientation.x = quaternion[0]
    move_base_goal.target_pose.pose.orientation.y = quaternion[1]
    move_base_goal.target_pose.pose.orientation.z = quaternion[2]
    move_base_goal.target_pose.pose.orientation.w = quaternion[3]
    return move_base_goal

class SendGoal:
    def __init__(self):
        print("Inside Send Person Follower Goal")
        rospy.init_node("person_follower_goal_node")
        self.client = actionlib.SimpleActionClient("move_base_flex/move_base", mbf_msgs.MoveBaseAction)
        self.client.wait_for_server(rospy.Duration(20))
        print("Move detected")
        self.goals = self.read_goals_from_file()
        self.current_goal_idx = 0
        rospy.Timer(rospy.Duration(1), self.send_next_goal)

    def read_goals_from_file(self):
        with open(GOALS_FILE, 'r') as f:
            return f.readlines()

    def send_next_goal(self, event):
        if self.current_goal_idx >= len(self.goals):
            print("Reached end of goals")
            rospy.signal_shutdown("Reached end of goals")
            return
        goal_str = self.goals[self.current_goal_idx]
        self.current_goal_idx += 1
        distance, angle_str = goal_str.strip().split(',')
        distance = float(distance)
        angle_str = angle_str.strip().lower()
        if angle_str == 'left':
            angle = math.radians(float(angle_str))
        elif angle_str == 'right':
            angle = -math.radians(float(angle_str))
        else:
            angle = 0
        self.client.send_goal(create_move_base_goal(distance, 0, angle))
        self.client.wait_for_result()


if __name__ == '__main__':
    SendGoalObj = SendGoal()
    rospy.spin()
