#! /usr/bin/env python3
import time
from geometry_msgs.msg import PoseStamped # Pose with ref frame and timestamp
from rclpy.duration import Duration # Handles time for ROS 2
import rclpy # Python client library for ROS 2
 
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult # Helper module
rclpy.init()
navigator = BasicNavigator()


init_pose = PoseStamped()
init_pose.header.frame_id = 'map'
init_pose.header.stamp = navigator.get_clock().now().to_msg()
init_pose.pose.position.x = 1.0
init_pose.pose.position.y = 1.0
init_pose.pose.position.z = 0.0
init_pose.pose.orientation.x = 0.0
init_pose.pose.orientation.y = 0.0
init_pose.pose.orientation.z = 0.0
init_pose.pose.orientation.w = 1.0

goal_pose = PoseStamped()
goal_pose.header.frame_id = 'map'
goal_pose.header.stamp = navigator.get_clock().now().to_msg()
goal_pose.pose.position.x = 0.0
goal_pose.pose.position.y = 0.0
goal_pose.pose.position.z = 0.0
goal_pose.pose.orientation.x = 0.0
goal_pose.pose.orientation.y = 0.0
goal_pose.pose.orientation.z = 0.0
goal_pose.pose.orientation.w = 1.0



navigator.setInitialPose(init_pose)
navigator.waitUntilNav2Active() # if autostarted, else use lifecycleStartup()
path = navigator.getPath(init_pose, goal_pose)
smoothed_path = navigator.smoothPath(path)

navigator.goToPose(goal_pose)
