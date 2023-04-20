#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import LaserScan


class MoveAndAlign:
    def __init__(self):
        rospy.init_node('move_and_align')
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        self.sub = rospy.Subscriber('/scan_filtered', LaserScan, self.scan_callback)
        self.angle_to_nearest_obstacle = None

    def scan_callback(self, data):
        # Find the angle to the nearest obstacle
        min_dist = min(data.ranges)
        min_dist_index = data.ranges.index(min_dist)
        self.angle_to_nearest_obstacle = min_dist_index * (data.angle_increment) + data.angle_min

    def rotate_to_nearest_obstacle(self):
        if self.angle_to_nearest_obstacle is None:
            print("No laser scan data received yet.")
            return

        goal = self.get_rotation_goal(self.angle_to_nearest_obstacle)
        self.client.send_goal(goal)
        self.client.wait_for_result()

    def get_rotation_goal(self, angle):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "base_link"
        goal.target_pose.header.stamp = rospy.Time.now()
        q = quaternion_from_euler(0, 0, angle)
        goal.target_pose.pose.orientation = Quaternion(*q)
        return goal

if __name__ == '__main__':
    move_and_align = MoveAndAlign()

    # Rotate to face the nearest obstacle
    move_and_align.rotate_to_nearest_obstacle()

