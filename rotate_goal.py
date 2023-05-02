#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import tf2_ros
import tf2_geometry_msgs

def move_and_rotate():
    # Initialize the ROS node
    rospy.init_node('move_and_rotate', anonymous=True)

    # Create an action client for the move_base server
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    # Create a TransformStamped object to get the transform from the base_link frame to the map frame
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    transform_stamped = tf_buffer.lookup_transform('map', 'base_link', rospy.Time(), rospy.Duration(1.0))

    # Create a MoveBaseGoal message for the robot to rotate 90 degrees
    rotate_goal = MoveBaseGoal()
    rotate_goal.target_pose.header.frame_id = 'map'
    rotate_goal.target_pose.header.stamp = rospy.Time.now()

    # Set the orientation of the goal to rotate the robot by 90 degrees
    orientation_quat = tf2_geometry_msgs.transform_to_msg(transform_stamped.transform).rotation
    orientation_quat.z += 0.707
    orientation_quat.w += 0.707
    rotate_goal.target_pose.pose.orientation = orientation_quat

    # Transform the goal from the map frame to the base_link frame
    rotate_goal_in_base_link = tf_buffer.transform(rotate_goal.target_pose, 'base_link')

    # Send the rotation goal to the move_base server
    client.send_goal(rotate_goal_in_base_link)
    client.wait_for_result()

    # Create a MoveBaseGoal message for the robot to move 0.5 units forward
    move_goal = MoveBaseGoal()
    move_goal.target_pose.header.frame_id = 'map'
    move_goal.target_pose.header.stamp = rospy.Time.now()

    # Set the position of the goal to move the robot 0.5 units forward in the direction of the rotation
    position_in_base_link = PoseStamped()
    position_in_base_link.header.frame_id = 'base_link'
    position_in_base_link.header.stamp = rospy.Time.now()
    position_in_base_link.pose.position.x = 0.5
    position_in_map = tf_buffer.transform(position_in_base_link, 'map')
    move_goal.target_pose.pose.position = position_in_map.pose.position

    # Transform the goal from the map frame to the base_link frame
    move_goal_in_base_link = tf_buffer.transform(move_goal.target_pose, 'base_link')

    # Send the move goal to the move_base server
    client.send_goal(move_goal_in_base_link)
    client.wait_for_result()

if __name__ == '__main__':
    try:
        move_and_rotate()
    except rospy.ROSInterruptException:
        pass
