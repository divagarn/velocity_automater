#!/usr/bin/env python

import math
import time
import rospy
import actionlib
from tf import TransformListener
from geometry_msgs.msg import Quaternion, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler


class RobotMovement:
    def __init__(self):
        rospy.init_node('robot_controller', anonymous=False)
        self.tf = TransformListener()
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

    def get_current_position(self):
        (trans, rot) = self.tf.lookupTransform('/map', '/map', rospy.Time(0))
        return trans[0], trans[1], rot[2]

    def move(self, distance, degree):
        x, y, current_degree = self.get_current_position()
        theta = math.radians(degree) + current_degree
        x += distance * math.cos(theta)
        y += distance * math.sin(theta)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "base_link"
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        q = Quaternion(*quaternion_from_euler(0, 0, theta))
        goal.target_pose.pose.orientation = q
        self.client.send_goal(goal)
        time.sleep(2)
        print("Current degree: {} degrees".format(current_degree))

    def main_run(self, file_path):
        with open(file_path, 'r') as f:
            for line in f:
                try:
                    distance, degree = map(float, line.strip().split(','))
                    self.move(distance, degree)
                    rospy.sleep(5)
                except (ValueError, KeyboardInterrupt) as error:
                    rospy.logerr("Failed to send the goal: {}".format(str(error)))


if __name__ == '__main__':
    controller = RobotMovement()
    path = '/home/divagar/mbf_tuning/final_src_dir/demo.txt'
    controller.main_run(path)
