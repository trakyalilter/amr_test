import sys
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid  # ROS2 specific import
import numpy as np
from PyQt5.QtCore import Qt, QThread
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QApplication, QWidget, QGraphicsView, QGraphicsScene, QVBoxLayout, QLabel

class MapViewer(Node):
    def __init__(self):
        super().__init__('map_viewer')

        # Initialize subscriber for the /map topic (OccupancyGrid type)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)

        # Initialize the PyQt application
        self.app = QApplication(sys.argv)

        # Create the MapNavigationApp widget
        self.ui = MapNavigationApp()

    def map_callback(self, msg):
        """Callback function to process map data from the /map topic."""
        self.get_logger().info("Received map data")
        self.map_data = msg
        self.ui.show()
        # Process the map data to display
        self.update_ui()
	def update_ui(self):
        if self.map_data:
		    # Convert OccupancyGrid data to numpy array
		    width = self.map_data.info.width
		    height = self.map_data.info.height
		    map_array = np.array(self.map_data.data).reshape((height, width))

		    # Flip the map vertically (flip along the Y-axis)
		    map_array = np.flipud(map_array)

		    # Map free (0) -> white, occupied (100) -> black, unknown (-1) -> gray
		    display_map = np.zeros_like(map_array, dtype=np.uint8)
		    display_map[map_array == 0] = 255  # Free space -> White
		    display_map[map_array == 100] = 0  # Occupied -> Black
		    display_map[map_array == -1] = 127  # Unknown -> Gray

		    # Convert numpy array to QImage for display
		    height, width = display_map.shape
		    qimage = QImage(display_map.data, width, height, QImage.Format_Grayscale8)
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

    # Run the ROS2 spin loop in a separate thread
    ui_thread = QThread()
    ui_thread.run = node.run
    ui_thread.start()

    # Run the PyQt application loop
    sys.exit(node.app.exec_())

if __name__ == '__main__':
    main()

