#!/usr/bin/env python
__author__= 'divagarn'
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion
import tf
from tf.transformations import quaternion_from_euler
from dynamic_reconfigure.client import Client


class MoveForward:
    def __init__(self):
        rospy.init_node('move_forward')
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.tf_listener = tf.TransformListener()
        self.client.wait_for_server()
        self.dynamic_reconfigure_client = Client("move_base/DWAPlannerROS", timeout=30)


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

    def move_backward(self, num_moves, distance):
        self.dynamic_reconfigure_client.update_configuration({"min_vel_x": -0.1})
        for i in range(num_moves):
            goal = self.get_forward_goal(-distance)
            self.client.send_goal(goal)
            self.client.wait_for_result()
        self.dynamic_reconfigure_client.update_configuration({"min_vel_x": 0.0})

    def get_forward_goal(self, distance):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = self.get_current_x() + distance
        goal.target_pose.pose.position.y = self.get_current_y()
        goal.target_pose.pose.orientation.w = 1.0
        return goal

    # def get_rotation_goal(self, angle):
    #     goal = MoveBaseGoal()
    #     goal.target_pose.header.frame_id = "map"
    #     goal.target_pose.header.stamp = rospy.Time.now()
    #     q = quaternion_from_euler(0, 0, angle / 180.0 * 3.14159)  # convert angle from degrees to radians
    #     goal.target_pose.pose.orientation = Quaternion(*q)
    #     return goal

    def get_rotation_goal(self, angle):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "base_link"
        goal.target_pose.header.stamp = rospy.Time.now()
        q = quaternion_from_euler(0, 0, angle/180.0*3.14159)
        goal.target_pose.pose.orientation = Quaternion(*q)
        return goal

    def get_current_x(self):
        try:
            self.tf_listener.waitForTransform('map', 'base_link', rospy.Time(), rospy.Duration(1.0))
            (trans, rot) = self.tf_listener.lookupTransform('map', 'base_link', rospy.Time(0))
            return trans[0]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr('Unable to get current X position')
            return 0.0

    def get_current_y(self):
        try:
            self.tf_listener.waitForTransform('map', 'base_link', rospy.Time(), rospy.Duration(1.0))
            (trans, rot) = self.tf_listener.lookupTransform('map', 'base_link', rospy.Time(0))
            return trans[1]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr('Unable to get current Y position')
            return 0.0


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
        elif input_list[1] == 'B':
            goal_type = "backward"
            goal_amount = float(input_list[0])
            num_moves = int(input_list[2])
            move_forward.move_backward(num_moves, goal_amount)
        elif input_list[0] == 'END':
            break
        else:
            print("Invalid input.")
            continue
        print("Reached {} goal of {} {} for {} times.".format(goal_type, goal_amount, "meter" if goal_type == "forward" else "degrees", num_moves))
