#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
import time

def send_goal(x, y, yaw, frame_id="map"):
    """
    Sends a PoseStamped goal to the robot.
    
    :param x: X coordinate of the goal (meters)
    :param y: Y coordinate of the goal (meters)
    :param yaw: Orientation (yaw in radians)
    :param frame_id: Coordinate frame for the pose (default: "map")
    """
    # Create a PoseStamped message
    goal = PoseStamped()
    goal.header.frame_id = frame_id
    goal.header.stamp = rospy.Time.now()

    # Set position
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.position.z = 0.0

    # Convert yaw to quaternion
    quat = quaternion_from_euler(0, 0, yaw)  # roll, pitch, yaw
    goal.pose.orientation.x = quat[0]
    goal.pose.orientation.y = quat[1]
    goal.pose.orientation.z = quat[2]
    goal.pose.orientation.w = quat[3]

    # Publish the goal
    rospy.loginfo(f"Sending goal: x={x}, y={y}, yaw={yaw}")
    goal_pub.publish(goal)

if __name__ == "__main__":
    rospy.init_node("send_goal_script")

    # Publisher to send goals to move_base
    goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
    
    # Wait for the publisher to be ready
    rospy.sleep(2)

    # Define four target poses (x, y, yaw in radians)
    targets = [
        (1.0, 1.0, 0.0),  # Pose 1
        (2.0, 1.0, 1.57),  # Pose 2
        (2.0, 2.0, 3.14),  # Pose 3
        (1.0, 2.0, -1.57),  # Pose 4
    ]

    # Send the robot to each pose sequentially
    for target in targets:
        x, y, yaw = target
        send_goal(x, y, yaw)
        
        # Wait for the robot to reach the goal
        rospy.sleep(10)  # Adjust this time based on your robot's speed and goal distance
