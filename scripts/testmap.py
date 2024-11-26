import sys
import threading  # To manage ROS2 in a separate thread
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid  # ROS2 specific import
import numpy as np
import cv2  # OpenCV for image processing
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel


class MapViewer(Node):
    def __init__(self):
        super().__init__('map_viewer')

        # Initialize subscriber for the /map topic (OccupancyGrid type)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)

        # Initialize the PyQt application
        self.app = QApplication(sys.argv)

        # Create the MapNavigationApp widget
        self.ui = MapNavigationApp()

        # Initialize map data
        self.map_data = None

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
            self.get_logger().info(f"Map dimensions: {width}x{height}, resolution: {resolution} m/pixel")

            # Convert OccupancyGrid data to numpy array
            map_array = np.array(self.map_data.data).reshape(( height,width))

            # Debug: Ensure reshaped map aligns with expected dimensions
            self.get_logger().info(f"Map array shape: {map_array.shape}")

            # Flip the map vertically to align with PyQt's coordinate system
            # map_array = np.flipud(map_array)

            # Map free (0) -> white, occupied (100) -> black, unknown (-1) -> gray
            display_map = np.zeros_like(map_array, dtype=np.uint8)
            display_map[map_array == 0] = 255  # Free space -> White
            display_map[map_array == 100] = 0  # Occupied -> Black
            display_map[map_array == -1] = 127  # Unknown -> Gray

            # Scale map based on resolution
            scale_factor = max(1, min(int(10 / resolution), 100))  # Clamp scale to [1, 100]
            scaled_map = cv2.resize(
                display_map,
                (width * 4, height * 4),
                interpolation=cv2.INTER_NEAREST
            )

            # Convert numpy array to QImage for display
            scaled_height, scaled_width = scaled_map.shape
            qimage = QImage(
                scaled_map.data, scaled_width,scaled_height, QImage.Format_Grayscale8
            )
            pixmap = QPixmap.fromImage(qimage)

            # Update the UI with the new pixmap (map image)
            self.ui.set_map(pixmap)

    def run(self):
        """Run the ROS2 node and PyQt application loop."""
        rclpy.spin(self)

    def stop(self):
        """Shut down the node and ROS2."""
        self.get_logger().info('Shutting down ROS2 node')
        self.destroy_node()
        rclpy.shutdown()


class MapNavigationApp(QWidget):
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

        # Show the UI initially
        self.show()

    def set_map(self, pixmap):
        """Update the displayed map in the UI."""
        self.label.setPixmap(pixmap)
        self.label.setAlignment(Qt.AlignCenter)
        self.label.adjustSize()  # Adjust the QLabel to fit the new image


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
