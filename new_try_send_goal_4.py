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

GOAL = Point(0.5, 0, 0)

class SendGoal:
    def __init__(self):
        print("Inside Send Goal")
        rospy.init_node("send_goal_node")
        self.client = actionlib.SimpleActionClient("move_base_flex/move_base", mbf_msgs.MoveBaseAction)
        self.client.wait_for_server(rospy.Duration(20))
        print("Move detected")
        self.current_goal_idx = 0
        self.move_forward = True
        rospy.Timer(rospy.Duration(1), self.send_next_goal)

    def send_next_goal(self, event):
        if self.move_forward:
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
            print("Goal reached")

            # update the goal position to rotate
            self.move_forward = False
            self.rotation_goal = Twist()
            self.rotation_goal.angular.z = math.pi/2  # rotate 90 degrees to the right

        else:
            # rotate the robot from its current position
            rospy.loginfo("Rotating the robot")
            self.vel_publisher = rospy.Publisher("cmd_vel", Twist, queue_size=10)
            r = rospy.Rate(10)  # 10Hz
            time_to_rotate = math.pi / self.rotation_goal.angular.z
            start_time = rospy.Time.now()

            while (rospy.Time.now() - start_time).to_sec() < time_to_rotate:
                self.vel_publisher.publish(self.rotation_goal)
                r.sleep()

            # reset the move_forward flag to move the robot forward in the next iteration
            self.move_forward = True
            rospy.loginfo("Rotation completed, moving forward again")


if __name__ == '__main__':
    SendGoalObj = SendGoal()
    rospy.spin()

