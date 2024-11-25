#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import QApplication, QLabel, QMainWindow
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPixmap, QImage
import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler


class Nodeish(Node):
	def __init__(self):
		Node.__init__(self,'amr2')
class MapNavigationApp(QMainWindow, Nodeish):
    def __init__(self):
        # Initialize QMainWindow
        QMainWindow.__init__(self)
        Nodeish.__init__(self)
        self.map_sub = self.create_subscription(
            OccupancyGrid, "/map", self.map_callback, 10
        )
        self.get_logger().warn("Map is available.")
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)

        # Variables for map data
        self.map_image = None
        self.map_resolution = None
        self.map_origin = None

        # UI setup
        self.setWindowTitle("Map Navigation")
        self.setGeometry(100, 100, 800, 800)
        self.image_label = QLabel(self)
        self.image_label.setAlignment(Qt.AlignCenter)
        self.setCentralWidget(self.image_label)

        # Mouse click event
        self.image_label.mousePressEvent = self.on_map_click

    def map_callback(self, msg):
        """
        Callback for /map topic to update the map display.
        """
        self.map_resolution = msg.info.resolution
        self.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        width = msg.info.width
        height = msg.info.height

        # Convert occupancy grid to numpy array
        map_data = np.array(msg.data, dtype=np.int8).reshape(height, width)

        # Convert occupancy grid to grayscale image
        map_image = np.zeros((height, width), dtype=np.uint8)
        map_image[map_data == 0] = 255  # Free space
        map_image[map_data == -1] = 127  # Unknown space
        map_image[map_data > 0] = 0  # Occupied space

        # Convert to QImage for PyQt5
        print(map_image.data)
        self.map_image = QImage(map_image.data, width, height, QImage.Format_Grayscale8)
        self.map_image = self.map_image.scaled(800, 800, Qt.KeepAspectRatio)
        self.image_label.setPixmap(QPixmap.fromImage(self.map_image))

    def on_map_click(self, event):
        """
        Handle mouse click on the map to set a goal.
        """
        if self.map_image is None:
            self.get_logger().warn("Map is not available yet.")
            return

        # Get click position
        click_position = event.pos()
        x_click = click_position.x()
        y_click = click_position.y()

        # Scale click position to map coordinates
        map_width = self.map_image.width()
        map_height = self.map_image.height()
        x_normalized = x_click / map_width
        y_normalized = y_click / map_height

        # Convert to real-world coordinates
        x_real = self.map_origin[0] + (x_normalized * self.map_resolution * map_width)
        y_real = self.map_origin[1] + ((1 - y_normalized) * self.map_resolution * map_height)

        # Publish the goal
        self.send_goal(x_real, y_real, 0.0)  # Send with yaw=0.0 by default

    def send_goal(self, x, y, yaw):
        """
        Publish a PoseStamped goal to the /goal_pose topic.
        """
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()

        # Set position
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0

        # Set orientation (yaw to quaternion)
        quat = quaternion_from_euler(0, 0, yaw)
        goal.pose.orientation.x = quat[0]
        goal.pose.orientation.y = quat[1]
        goal.pose.orientation.z = quat[2]
        goal.pose.orientation.w = quat[3]

        self.get_logger().info(f"Sending goal: x={x}, y={y}, yaw={yaw}")
        self.goal_pub.publish(goal)


def main(args=None):
    # Initialize ROS 2 and PyQt5 application
    rclpy.init(args=args)
    app = QApplication(sys.argv)

    # Create and run the application
    window = MapNavigationApp()
    window.show()

    # Run PyQt5 event loop and ROS 2 spin in parallel
    try:
        app.exec_()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()

