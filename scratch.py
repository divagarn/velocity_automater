#!/usr/bin/env python
"""! @brief This is the main file to the bot to do exploration, coverage of the room with the help of some other
python files. This file is the integration of all the used scripts.
"""


##
# @file scratch.py
#
# @section todo_doxygen_example TODO
# - To adjust the center of rotation of the robot.
# - Add code to exploration after rotation
# - Need to capture obstacles in the map.
#
# @section author_doxygen_example Author(s)
# - Created by Divagar N on 10/05/2023.
# - Modified by Divagar N on 12/05/2023.

# Imports
import rospy
from geometry_msgs.msg import PolygonStamped
from nav_msgs.msg import OccupancyGrid
import numpy as np
import cv2
from base_scratch import *


class MapExplorer:
    """! The MapExplorer base class.

    This class is used to get the current position from the robot footprint.
    """
    def __init__(self):
        """! This is the MapExplorer initializer.

        """
        # These are the inputs to the further script.
        self.ROBOT_X = 0
        self.ROBOT_Y = 0
        self.MAP_ORIGIN_POSITION_X = -1
        self.MAP_ORIGIN_POSITION_Y = -1
        self.MAP_RESOLUTION = -1
        self.EXPLORATION = '0,90,True'

    def footprint_callback(self, data):
        """! Process the data.

        @param data to get the poligin poins of the robot footprint
        """
        # Extract the x and y coordinates from the PolygonStamped message
        rob_x = [p.x for p in data.polygon.points]
        rob_y = [p.y for p in data.polygon.points]
        # To align the exact ubiquity center of rotation.
        robot_center = 0.25  # 25 cm
        # These are the works to calculate the center from the robot footprint.
        rob_center_x_coordinates = [x + robot_center for x in rob_x]
        rob_x_mean = sum(rob_center_x_coordinates) / len(rob_center_x_coordinates)
        rob_y_mean = sum(rob_y) / len(rob_y)
        if self.MAP_RESOLUTION != -1:
            self.ROBOT_X = (rob_x_mean - self.MAP_ORIGIN_POSITION_X) / self.MAP_RESOLUTION
            self.ROBOT_Y = (rob_y_mean - self.MAP_ORIGIN_POSITION_Y) / self.MAP_RESOLUTION
            print((int(self.ROBOT_X)), (int(self.ROBOT_Y)))

    def exploration(self):
        """! The script calls from the base_scratch.py file.

        """
        controller = RobotMovement()
        # To do 360 degree rotation of 90 degrees four times.
        for ir in range(0, 4):
            controller.main_run(self.EXPLORATION)

    def callback(self, msg):
        """! Process the msg to draw the map using numpy.

        """
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg.data)
        data_in = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        self.MAP_ORIGIN_POSITION_X = msg.info.origin.position.x
        self.MAP_RESOLUTION = msg.info.resolution
        self.MAP_ORIGIN_POSITION_Y = msg.info.origin.position.y
        data_in = cv2.merge([data_in, data_in, data_in])
        if self.ROBOT_X != 0 and self.ROBOT_Y != 0:
            data_in = cv2.circle(data_in, (int(self.ROBOT_X), int(self.ROBOT_Y)), radius=5, color=(0, 0, 255),
                                 thickness=-1)
        data_in = cv2.flip(data_in, 1)
        cv2.imshow('test', data_in)
        cv2.waitKey(1)

    def run(self):
        """! Subscribes all the data we need to process and run some previous function in the order.

        """
        self.exploration()
        rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, self.callback)
        rospy.Subscriber('/move_base/global_costmap/footprint', PolygonStamped, self.footprint_callback)
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('map_explorer', anonymous=True)
    map_explorer = MapExplorer()
    map_explorer.run()
