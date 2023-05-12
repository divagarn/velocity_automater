#!/usr/bin/env python
"""! @brief Example Python program with Doxygen style comments."""


##
# @file base_scratch.py
#
# @section todo_doxygen_example TODO
# - Add path folder run as main run
#
# @section author_doxygen_example Author(s)
# - Created by Divagar N on 10/05/2023.
# - Modified by Divagar N on 12/05/2023.


# Imports
import math
import rospy
import actionlib
from tf import TransformListener
from geometry_msgs.msg import Quaternion, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler


# File Path
PATH = '/home/divagar/mbf_tuning/final_src_dir/demo.txt'


class RobotMovement:
    """! The robot movement base class.

    This class is used to get the current position of the robot and processing the automatic goals.
    """

    def __init__(self):
        """! The Sensor base class initializer.
        """

        rospy.init_node('robot_controller', anonymous=False)
        self.tf = TransformListener()
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

    def get_current_position(self):
        """! Get the current position from the map.

        @return positions of x, y and yaw.
        """

        (trans, rot) = self.tf.lookupTransform('/map', '/map', rospy.Time.now())
        return trans[0], trans[1], rot[2]

    def move(self, distance, degree, wait_for_goal):
        """! Process the distance, degree and wait_for_goal.

        @param distance the distance unit in meters.
        @param degree the degree unit in degrees.
        @param wait_for_goal it was sets in string t or f which is wait for the current goal to end.
        """

        x, y, current_degree = self.get_current_position()
        theta = math.radians(degree)
        x += distance * math.cos(theta)
        y += distance * math.sin(theta)
        theta += current_degree
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "base_link"
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        q = Quaternion(*quaternion_from_euler(0, 0, theta))
        goal.target_pose.pose.orientation = q
        self.client.send_goal(goal)
        if wait_for_goal == "True":
            self.client.wait_for_result()

        print("Current degree: {} degrees".format(current_degree))

    def main_run(self, file_path):
        """! Process the file_path.

        @param file_path it gets the file path from the start to execute the goals from the txt files.
        """

        # This condition is for when the file_path is empty it asks the goals from the user.
        if file_path == "":
            while not rospy.is_shutdown():
                try:
                    input_str = input("Enter distance and degree: ")
                    # Get the inputs from the user as string.
                    distance, degree, wait_for_goal = map(str, input_str.split(','))
                    # Convert degree and distance as float.
                    self.move(float(distance), float(degree), wait_for_goal)
                except (ValueError, KeyboardInterrupt) as error:
                    rospy.logerr("Failed to send the goal: {}".format(str(error)))
        # This condition is run when the file_path is not empty.
        else:
            with open(file_path, 'r') as f:
                for line in f:
                    try:
                        distance, degree, wait_for_goal = map(str, line.strip().split(','))
                        self.move(float(distance), float(degree), wait_for_goal)
                        rospy.sleep(8)
                    except (ValueError, KeyboardInterrupt) as error:
                        rospy.logerr("Failed to send the goal: {}".format(str(error)))
