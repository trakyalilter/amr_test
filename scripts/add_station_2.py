#!/usr/bin/env python3
import sys
import json
import os
import threading  # To manage ROS2 in a separate thread
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped  # For AMCL pose
import numpy as np
import cv2  # OpenCV for image processing
from PyQt5.QtCore import Qt, QPoint, pyqtSignal
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel
from PyQt5.QtGui import QPainter, QColor, QPen, QBrush
from PyQt5.QtWidgets import QPushButton
from PyQt5.QtWidgets import QInputDialog, QMessageBox
import numpy as np # Scientific computing library for Python
 
def quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]
class MapViewer(Node):
    def __init__(self):
        super().__init__('map_viewer')
        self.isDataLoaded = False
        # Initialize subscribers
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.create_subscription(Path, '/plan', self.plan_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)

        self.navigate_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Initialize publisher for the goal pose
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Initialize the PyQt application
        self.app = QApplication(sys.argv)

        # Create the MapNavigationApp widget and connect mouse click signals
        self.ui = MapNavigationApp()
        self.ui.add_station_signal.connect(self.add_station)
        self.ui.map_clicked_and_dragged.connect(self.send_goal_pose)

        self.robot_status = "Idle, waiting for a goal..."
        self.ui.update_status(self.robot_status)

        # Initialize map, path, and robot pose data
        self.map_data = None
        self.scale_factor = 4  # Fixed scale factor
        self.path = None  # Store the latest path
        self.robot_position = None  # Store the robot's current position
        self.stations = []
        # self.load_stations_from_json()
        import atexit
        atexit.register(self.save_stations_to_json)
    def add_station(self, position):
        """Handle adding a station to the map."""
        if not self.map_data:
            QMessageBox.warning(self.ui, "Error", "Map data is not available yet!")
            return

        # Ask the user for the station name
        station_name, ok = QInputDialog.getText(self.ui, "Set Station Name", "Enter station name:")
        if not ok or not station_name.strip():
            QMessageBox.warning(self.ui, "Error", "Station name cannot be empty!")
            return

        resolution = self.map_data.info.resolution
        origin_x, origin_y = self.map_data.info.origin.position.x, self.map_data.info.origin.position.y
        scale_factor = self.scale_factor

        # Convert pixel position to map coordinates
        map_x = (position.x() / scale_factor) * resolution + origin_x
        map_y = (self.ui.map_metadata["height"] - position.y() / scale_factor) * resolution + origin_y

        # Store station data (map position and UI button)
        station_data = {
            "map_position": (map_x, map_y),
            "name": station_name,  # Save station name
            "button": None  # Placeholder for button reference
        }

        # Add button to the UI overlay
        button = self.ui.add_station_button(position)
        button.setText(station_name)  # Set button text as station name
        button.clicked.connect(lambda: self.send_amr_to_station(map_x, map_y,station_name))

        # Save button reference in station data
        station_data["button"] = button
        self.stations.append(station_data)

        # Save stations to JSON
        self.save_stations_to_json()

        self.get_logger().info(f"Station '{station_name}' added at map coordinates: ({map_x}, {map_y})")

    def save_stations_to_json(self):
        """Save stations to a JSON file."""
        stations_data = []
        for station in self.stations:
            stations_data.append({
                "map_position": station["map_position"],
                "name": station.get("name", "Unnamed Station")  # Save the name
            })

        with open("stations.json", "w") as json_file:
            json.dump(stations_data, json_file, indent=4)
        self.get_logger().info("Stations saved to stations.json")

    def load_stations_from_json(self):
        """Load stations from a JSON file."""
        if not os.path.exists("stations.json"):
            self.get_logger().info("No stations.json file found. Starting with no stations.")
            return

        with open("stations.json", "r") as json_file:
            stations_data = json.load(json_file)

        for station_data in stations_data:
            map_x, map_y = station_data["map_position"]
            station_name = station_data.get("name", "Unnamed Station")  # Get the name or default to "Unnamed Station"

            # Convert map coordinates to pixel position for the UI
            resolution = self.map_data.info.resolution
            origin_x, origin_y = self.map_data.info.origin.position.x, self.map_data.info.origin.position.y
            scale_factor = self.scale_factor
            pixel_x = int((map_x - origin_x) / resolution * scale_factor)
            pixel_y = int((self.ui.map_metadata["height"] - (map_y - origin_y) / resolution) * scale_factor)

            # Add station button
            button = self.ui.add_station_button(QPoint(pixel_x, pixel_y))
            button.setText(station_name)  # Set the button text as the station name
            button.clicked.connect(lambda: self.send_amr_to_station(map_x, map_y,station_name))

            # Save station data to self.stations
            self.stations.append({
                "map_position": (map_x, map_y),
                "name": station_name,
                "button": button
            })

        self.get_logger().info("Stations loaded from stations.json")
        self.isDataLoaded = True

    def send_amr_to_station(self, map_x, map_y,station_name):
        """Send the AMR to the selected station."""
        quaternion = quaternion_from_euler(0, 0, 90) # Default orientation

        # Create a PoseStamped message for the goal
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"  # Frame of the map
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = map_x
        goal_pose.pose.position.y = map_y
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = quaternion[0]
        goal_pose.pose.orientation.y = quaternion[1]
        goal_pose.pose.orientation.z = quaternion[2]
        goal_pose.pose.orientation.w = quaternion[3]

        # self.send_nav2_goal(goal_pose)
        self.goal_publisher.publish(goal_pose)
        self.robot_status = f"Sending AMR to {station_name}"
        self.ui.update_status(self.robot_status)
        self.get_logger().info(f"AMR sent to station at: ({map_x}, {map_y})")
    def place_docking_station(self, x, y):
        """Place a docking station."""
        self.ui.add_docking_station(x, y)
        self.get_logger().info(f"Placed docking station at pixel coordinates: ({x}, {y})")
    def map_callback(self, msg):
        """Callback function to process map data from the /map topic."""
        self.get_logger().info("Received map data")
        self.map_data = msg
        if self.isDataLoaded == False:
            self.load_stations_from_json()

        
        self.update_ui()

    def plan_callback(self, msg):
        """Callback function to process path data from the /plan topic."""
        self.get_logger().info("Received path data")
        self.path = msg.poses  # Store the sequence of poses
        self.update_ui()

    def pose_callback(self, msg):
        """Callback function to process the robot's position from /amcl_pose."""
        self.get_logger().info("Received robot pose")
        self.robot_position = msg.pose.pose.position  # Store the robot's current position
        self.update_ui()

    def update_ui(self):
        """Update the UI with the new map, path, and robot position."""
        if self.map_data:
            # Extract map dimensions and resolution
            width = self.map_data.info.width
            height = self.map_data.info.height
            resolution = self.map_data.info.resolution
            origin_x = self.map_data.info.origin.position.x
            origin_y = self.map_data.info.origin.position.y

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

            # Draw the path if available
            if self.path:
                for i in range(len(self.path) - 1):
                    start = self.convert_to_pixel(
                        self.path[i].pose.position, resolution, origin_x, origin_y, height
                    )
                    end = self.convert_to_pixel(
                        self.path[i + 1].pose.position, resolution, origin_x, origin_y, height
                    )
                    # Draw the line on the scaled map
                    cv2.line(
                        scaled_map,
                        start,
                        end,
                        (0, 0, 255),  # Red color for the path
                        2,  # Line thickness
                    )

            # Draw the robot's current position
            if self.robot_position:
                robot_pixel = self.convert_to_pixel(
                    self.robot_position, resolution, origin_x, origin_y, height
                )
                cv2.circle(
                    scaled_map,
                    robot_pixel,
                    5,  # Radius of the circle
                    (255, 0, 0),  # Blue color for the robot
                    -1,  # Filled circle
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

    def convert_to_pixel(self, position, resolution, origin_x, origin_y, height):
        """Convert map coordinates to pixel coordinates."""
        pixel_x = int((position.x - origin_x) / resolution * self.scale_factor)
        pixel_y = int((height - (position.y - origin_y) / resolution) * self.scale_factor)
        return (pixel_x, pixel_y)

    def send_goal_pose(self, start_point, end_point):
        """Convert clicked and dragged points to map coordinates and publish a goal pose."""
        if self.map_data:
            resolution = self.map_data.info.resolution
            origin_x, origin_y = self.map_data.info.origin.position.x, self.map_data.info.origin.position.y
            scale_factor = self.scale_factor

            # Convert start point to map coordinates
            map_x = (start_point.x() / scale_factor) * resolution + origin_x
            map_y = (self.ui.map_metadata["height"] - start_point.y() / scale_factor) * resolution + origin_y

            # Convert drag direction to calculate orientation
            dx = end_point.x() - start_point.x()
            dy = end_point.y() - start_point.y()
            angle = np.arctan2(dy, dx)  # Compute the angle in radians
            quaternion = quaternion_from_euler(0, 0, angle)  # Convert angle to quaternion

            # Create a PoseStamped message for the goal
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = "map"  # Frame of the map
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose.position.x = map_x
            goal_pose.pose.position.y = map_y
            goal_pose.pose.position.z = 0.0
            goal_pose.pose.orientation.x = quaternion[0]
            goal_pose.pose.orientation.y = quaternion[1]
            goal_pose.pose.orientation.z = quaternion[2]
            goal_pose.pose.orientation.w = quaternion[3]

            # self.send_nav2_goal(goal_pose)
            # Publish the goal pose
            self.robot_status = f"Sending AMR to ({map_x}, {map_y}) with orientation: {angle} radians"
            self.ui.update_status(self.robot_status)
            self.goal_publisher.publish(goal_pose)
            self.get_logger().info(f"Published goal pose: ({map_x}, {map_y}) with orientation: {angle} radians")
    def send_nav2_goal(self, goal_pose):
        """Send the navigation goal to the Nav2 action server."""
        if not self.navigate_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("NavigateToPose action server not available!")
            self.robot_status = "Error: Nav2 server unavailable!"
            self.ui.update_status(self.robot_status)
            return

        # Create a goal message for NavigateToPose
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_pose

        # Send the goal asynchronously
        self.robot_status = "Sending goal to Nav2..."
        self.ui.update_status(self.robot_status)
        future = self.navigate_action_client.send_goal_async(
            nav_goal,
            feedback_callback=self.nav2_feedback_callback
        )
        future.add_done_callback(self.nav2_goal_response_callback)
    def nav2_goal_response_callback(self, future):
        """Callback for Nav2 goal response."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.robot_status = "Goal rejected by Nav2!"
            self.ui.update_status(self.robot_status)
            return

        self.robot_status = "Goal accepted, navigating..."
        self.ui.update_status(self.robot_status)

        # Monitor the result of the goal
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav2_result_callback)

    def nav2_feedback_callback(self, feedback_msg):
        """Callback to handle feedback from Nav2."""
        feedback = feedback_msg.feedback
        self.robot_status = f"Navigating... Estimated progress: {feedback.distance_remaining:.2f} meters remaining"
        self.ui.update_status(self.robot_status)
    def goal_status_callback(self, msg):
        """Callback to handle goal status updates."""
        if msg.status_list:
            # Get the last goal's status
            status = msg.status_list[-1].status
            if status == 1:
                self.robot_status = "Goal received, robot is moving..."
            elif status == 3:
                self.robot_status = "Goal reached successfully!"
            elif status == 4:
                self.robot_status = "Goal was aborted."
            elif status == 5:
                self.robot_status = "Goal was rejected."
            else:
                self.robot_status = "Unknown goal status."

        else:
            self.robot_status = "No active goals."

        # Update the status in the UI
        self.ui.update_status(self.robot_status)
    def nav2_result_callback(self, future):
        """Callback for Nav2 goal result."""
        result = future.result()
        if result.status == 4:  # SUCCEEDED
            self.robot_status = "Goal reached successfully!"
        elif result.status == 5:  # ABORTED
            self.robot_status = "Goal aborted!"
        elif result.status == 6:  # CANCELED
            self.robot_status = "Goal canceled!"
        else:
            self.robot_status = "Unknown result status!"

        self.ui.update_status(self.robot_status)
class MapNavigationApp(QWidget):
    map_clicked_and_dragged = pyqtSignal(QPoint, QPoint)  # Signal to emit start and end points
    add_station_signal = pyqtSignal(QPoint)
    def __init__(self):
        """Initialize the UI for the Map Navigation App."""
        super().__init__()

        # Set up the window properties
        self.setWindowTitle("Map Viewer")
        self.setGeometry(100, 100, 800, 600)

        # Create a QLabel to display the map
        self.label = QLabel(self)
        self.label.setAlignment(Qt.AlignCenter)
        # Create a QLabel for the robot's status
        self.status_label = QLabel(self)
        self.status_label.setText("Robot Status: Waiting for goal...")
        self.status_label.setAlignment(Qt.AlignCenter)
        # Set up layout and add label
        layout = QVBoxLayout()
        layout.addWidget(self.label)
        layout.addWidget(self.status_label)
        self.setLayout(layout)

        # Metadata for the map
        self.map_metadata = {}

        # Variables to track mouse interaction
        self.start_point = None
        self.end_point = None
        # Store docking station and stations (in pixel coordinates)

        self.docking_station = None
        self.stations = []  # List of stations as (x, y) tuples

        # Show the UI initially
        self.show()

    def set_map(self, pixmap):
        """Update the displayed map in the UI."""
        self.label.setPixmap(pixmap)
        self.label.setAlignment(Qt.AlignCenter)
        self.label.adjustSize()  # Adjust the QLabel to fit the new image
    def update_status(self, status_text):
        """Update the robot's status in the UI."""
        self.status_label.setText(f"Robot Status: {status_text}")
    def add_docking_station(self, x, y):
        """Add a docking station at the given pixel coordinates."""
        self.docking_station = (x, y)
        self.update()

    def add_station(self, x, y):
        """Add a station at the given pixel coordinates."""
        self.stations.append((x, y))
        self.update()
    def add_station_button(self, position):
        """Add a button to the map to represent a station."""
        button = QPushButton(self)
        button.setGeometry(position.x(), position.y(), 50, 50)  # Position and size of the button
        button.show()
        return button
    def mousePressEvent(self, event):
        """Handle mouse press events to detect map clicks."""
        if event.button() == Qt.RightButton and self.label.pixmap():
            # Get the position of the click relative to the QLabel
            label_pos = self.label.pos()
            map_pos = event.pos() - label_pos
            self.add_station_signal.emit(map_pos)  # Emit signal to add a station

        if event.button() == Qt.LeftButton and self.label.pixmap():
            label_pos = self.label.pos()
            self.start_point = event.pos() - label_pos  # Store the start point

    def mouseReleaseEvent(self, event):
        """Handle mouse release events to detect drag and set orientation."""
        if event.button() == Qt.LeftButton and self.start_point:
            # Get the position of the release relative to the QLabel
            label_pos = self.label.pos()
            self.end_point = event.pos() - label_pos  # Store the end point

            # Emit the signal with start and end points
            self.map_clicked_and_dragged.emit(self.start_point, self.end_point)

            # Reset start and end points
            self.start_point = None
            self.end_point = None
    def mouseDoubleClickEvent(self, event):
        """Handle mouse double-clicks to add a station."""
        if event.button() == Qt.LeftButton and self.label.pixmap():
            label_pos = self.label.pos()
            click_point = event.pos() - label_pos

            # Emit signal to add a station at map coordinates
            self.station_added.emit(click_point.x(), click_point.y())
    def paintEvent(self, event):
        """Custom paint event to draw the overlay and stations."""
        super().paintEvent(event)
        painter = QPainter(self)

        # Draw the semi-transparent mask
        painter.setBrush(QColor(0, 0, 0, 100))  # Black with transparency
        painter.setPen(Qt.NoPen)
        painter.drawRect(self.rect())

        # Draw the docking station (if any)
        if self.docking_station:
            painter.setBrush(QColor(0, 255, 0, 200))  # Green
            painter.setPen(QPen(QColor(0, 255, 0), 2))
            x, y = self.docking_station
            painter.drawEllipse(QPoint(x, y), 10, 10)  # Draw circle (radius 10)

        # Draw the stations
        painter.setBrush(QColor(255, 0, 0, 200))  # Red
        painter.setPen(QPen(QColor(255, 0, 0), 2))
        for x, y in self.stations:
            painter.drawRect(x - 5, y - 5, 10, 10)  # Draw rectangle (10x10 pixels)
def stop(self):
    self.destroy_node()
    rclpy.shutdown()
def main():
    rclpy.init(args=sys.argv)
    node = MapViewer()
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,))
    ros_thread.start()
    try:
        sys.exit(node.app.exec_())
    finally:
        node.stop()
        ros_thread.join()


if __name__ == '__main__':
    main()
