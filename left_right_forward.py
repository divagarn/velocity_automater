#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler

class MoveForward:
    def __init__(self):
        rospy.init_node('move_forward')
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
    def move_forward(self, num_moves, distance):
        for i in range(num_moves):
            goal = self.get_forward_goal(distance)
            self.client.send_goal(goal)
            self.client.wait_for_result()
    def move_right_rotation(self, num_moves, angle):
        for i in range(num_moves):
            goal = self.get_rotation_goal(angle)
            self.client.send_goal(goal)
            self.client.wait_for_result()
    def move_left_rotation(self, num_moves, angle):
        for i in range(num_moves):
            goal = self.get_rotation_goal(-angle)
            self.client.send_goal(goal)
            self.client.wait_for_result()
    def get_forward_goal(self, distance):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "base_link"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = distance
        goal.target_pose.pose.orientation.w = 1.0
        return goal
    def get_rotation_goal(self, angle):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "base_link"
        goal.target_pose.header.stamp = rospy.Time.now()
        q = quaternion_from_euler(0, 0, angle/180.0*3.14159) # convert angle from degrees to radians
        goal.target_pose.pose.orientation = Quaternion(*q)
        return goal
if __name__ == '__main__':
    move_forward = MoveForward()
    while True:
        input_str = input("Input Goal Type, goal type (F/R/L),(e.g. 0.5,F,5): ")
        input_list = input_str.split(',')
        if input_list[1] == 'F':
            goal_type = "forward"
            goal_amount = float(input_list[0])
            num_moves = int(input_list[2])
            move_forward.move_forward(num_moves, goal_amount)
        elif input_list[1] == 'R':
            goal_type = "right rotation"
            goal_amount = 90
            num_moves = int(input_list[2])
            move_forward.move_right_rotation(num_moves, goal_amount)
        elif input_list[1] == 'L':
            goal_type = "left rotation"
            goal_amount = 90
            num_moves = int(input_list[2])
            move_forward.move_left_rotation(num_moves, goal_amount)
        elif input_list[0] == 'END':
            break
        else:
            print("Invalid input.")
            continue
        print("Reached {} goal of {} {} for {} times.".format(goal_type, goal_amount, "meter" if goal_type == "forward" else "degrees", num_moves))
