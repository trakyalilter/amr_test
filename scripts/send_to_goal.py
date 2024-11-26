import sys
import threading  # To manage ROS2 in a separate thread
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid  # ROS2 specific import
from geometry_msgs.msg import PoseStamped  # To publish goal poses
import numpy as np
import cv2  # OpenCV for image processing
from PyQt5.QtCore import Qt, QPoint
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel


class MapViewer(Node):
    def __init__(self):
        super().__init__('map_viewer')

        # Initialize subscriber for the /map topic (OccupancyGrid type)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)

        # Initialize publisher for the goal pose
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # Initialize the PyQt application
        self.app = QApplication(sys.argv)

        # Create the MapNavigationApp widget and connect mouse click signals
        self.ui = MapNavigationApp()
        self.ui.map_clicked.connect(self.send_goal_pose)

        # Initialize map data
        self.map_data = None
        self.scale_factor = 4  # Fixed scale factor

    def map_callback(self, msg):
        """Callback function to process map data from the /map topic."""
        self.get_logger().info("Received map data")
        self.map_data = msg
        self.update_ui()

    def update_ui(self):
        """Update the UI with the new map data."""
        if self.map_data:
            # Extract map dimensions and resolution
            width = self.map_data.info.width
            height = self.map_data.info.height
            resolution = self.map_data.info.resolution
            origin_x = self.map_data.info.origin.position.x
            origin_y = self.map_data.info.origin.position.y

            self.get_logger().info(
                f"Map dimensions: {width}x{height}, resolution: {resolution} m/pixel, origin: ({origin_x}, {origin_y})"
            )

            # Convert OccupancyGrid data to numpy array
            map_array = np.array(self.map_data.data).reshape((height, width))

            # Flip the map vertically to align with PyQt's coordinate system
            map_array = np.flipud(map_array)

            # Map free (0) -> white, occupied (100) -> black, unknown (-1) -> gray
            display_map = np.zeros_like(map_array, dtype=np.uint8)
            display_map[map_array == 0] = 255  # Free space -> White
            display_map[map_array == 100] = 0  # Occupied -> Black
            display_map[map_array == -1] = 127  # Unknown -> Gray

            # Scale map based on resolution
            scaled_map = cv2.resize(
                display_map,
                (width * self.scale_factor, height * self.scale_factor),
                interpolation=cv2.INTER_NEAREST,
            )

            # Convert numpy array to QImage for display
            scaled_height, scaled_width = scaled_map.shape
            qimage = QImage(
                scaled_map.data,
                scaled_width,
                scaled_height,
                QImage.Format_Grayscale8,
            )
            pixmap = QPixmap.fromImage(qimage)

            # Update the UI with the new pixmap (map image)
            self.ui.set_map(pixmap)
            self.ui.map_metadata = {
                "resolution": resolution,
                "origin": (origin_x, origin_y),
                "scale_factor": self.scale_factor,
                "width": width,
                "height": height,
            }

    def send_goal_pose(self, clicked_point):
        """Convert clicked point to map coordinates and publish a goal pose."""
        if self.map_data:
            resolution = self.map_data.info.resolution
            origin_x, origin_y = self.map_data.info.origin.position.x, self.map_data.info.origin.position.y
            scale_factor = self.scale_factor

            # Convert clicked pixel to map coordinates
            map_x = (clicked_point.x() / scale_factor) * resolution + origin_x
            map_y = (self.ui.map_metadata["height"] - clicked_point.y() / scale_factor) * resolution + origin_y

            # Create a PoseStamped message for the goal
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = "map"  # Frame of the map
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose.position.x = map_x
            goal_pose.pose.position.y = map_y
            goal_pose.pose.position.z = 0.0
            goal_pose.pose.orientation.w = 1.0  # Neutral orientation

            # Publish the goal pose
            self.goal_publisher.publish(goal_pose)
            self.get_logger().info(f"Published goal pose: ({map_x}, {map_y})")


class MapNavigationApp(QWidget):
    map_clicked = Qt.pyqtSignal(QPoint)  # Signal to emit clicked point

    def __init__(self):
        """Initialize the UI for the Map Navigation App."""
        super().__init__()

        # Set up the window properties
        self.setWindowTitle("Map Viewer")
        self.setGeometry(100, 100, 800, 600)

        # Create a QLabel to display the map
        self.label = QLabel(self)
        self.label.setAlignment(Qt.AlignCenter)

        # Set up layout and add label
        layout = QVBoxLayout()
        layout.addWidget(self.label)
        self.setLayout(layout)

        # Metadata for the map
        self.map_metadata = {}

        # Show the UI initially
        self.show()

    def set_map(self, pixmap):
        """Update the displayed map in the UI."""
        self.label.setPixmap(pixmap)
        self.label.setAlignment(Qt.AlignCenter)
        self.label.adjustSize()  # Adjust the QLabel to fit the new image

    def mousePressEvent(self, event):
        """Handle mouse press events to detect map clicks."""
        if event.button() == Qt.LeftButton and self.label.pixmap():
            # Get the position of the click relative to the QLabel
            label_pos = self.label.pos()
            clicked_point = event.pos() - label_pos

            # Emit the clicked point signal
            self.map_clicked.emit(clicked_point)


def main():
    """Main function to run the application."""
    # Initialize ROS2
    rclpy.init(args=sys.argv)

    # Create the MapViewer node
    node = MapViewer()

    # Run ROS2 spin loop in a separate thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,))
    ros_thread.start()

    # Run the PyQt application loop
    sys.exit(node.app.exec_())

    # Clean shutdown
    ros_thread.join()
    node.stop()


if __name__ == '__main__':
    main()
