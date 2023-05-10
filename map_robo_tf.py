#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PolygonStamped
from nav_msgs.msg import OccupancyGrid
import numpy as np
import cv2


robot_x = 0
robot_y = 0
map_origin_position_x = -1
map_origin_position_y = -1
mapResolution_ = -1


def footprint_callback(data):
    global robot_x
    global robot_y
    global mapResolution_
    # Extract the x and y coordinates from the PolygonStamped message

    rob_x = [p.x for p in data.polygon.points]
    rob_y = [p.y for p in data.polygon.points]

    rob_x_mean = sum(rob_x) / len(rob_x)
    rob_y_mean = sum(rob_y) / len(rob_y)

    # center = (type(rob_x_mean), type(rob_y))
    # print("Center coordinates:", center)
    # round_off = np.column_stack((rob_x, rob_y))
    # print(round_off.shape)
    # print(rob_x, rob_y)
    if mapResolution_ != -1:
        robot_x = (rob_x_mean - map_origin_position_x) / mapResolution_
        robot_y = (rob_y_mean - map_origin_position_y) / mapResolution_
        print((int(robot_x)), (int(robot_y)))


def callback(msg):
    global map_origin_position_x
    global map_origin_position_y
    global mapResolution_
    """rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg.data)"""
    datain = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
    map_origin_position_x = msg.info.origin.position.x
    mapResolution_ = msg.info.resolution

    map_origin_position_y = msg.info.origin.position.y
    """#print(map_origin_position_x,map_origin_position_y,mapResolution_)

    #print(robotx, roboty)
    #print(datain.shape)"""
    datain = cv2.merge([datain, datain, datain])
    if robot_x != 0 and robot_y != 0:
        """#print(robotx[0],roboty[0])
        #input('cc')"""
        datain = cv2.circle(datain, (int(robot_x), int(robot_y)), radius=5, color=(0, 0, 255), thickness=-1)
    datain = cv2.flip(datain, 1)
    """#input('c')"""
    cv2.imshow('test', datain)
    cv2.waitKey(1)


def listener():
    rospy.init_node('listener', anonymous=True)

    """#rospy.Subscriber("/map", OccupancyGrid, callback)"""
    rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, callback)
    rospy.Subscriber('/move_base/global_costmap/footprint', PolygonStamped, footprint_callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
