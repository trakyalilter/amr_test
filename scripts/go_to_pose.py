#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
import time

class GoalSender(Node):
    def __init__(self):
        super().__init__('send_goal_script')

        # Publisher to send goals to /goal_pose topic in ROS 2
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.get_logger().info('GoalSender Node initialized.')

    def send_goal(self, x, y, yaw, frame_id='map'):
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
        goal.header.stamp = self.get_clock().now().to_msg()

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
        self.get_logger().info(f"Sending goal: x={x}, y={y}, yaw={yaw}")
        self.goal_publisher.publish(goal)


def main(args=None):
    rclpy.init(args=args)

    # Initialize the GoalSender node
    node = GoalSender()

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
        node.send_goal(x, y, yaw)

        # Wait for the robot to reach the goal
        time.sleep(10)  # Adjust this time based on your robot's speed and goal distance

    node.get_logger().info("All goals sent successfully!")

    # Shutdown the ROS 2 node
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
