import sys
import json
import os
import threading  # To manage ROS2 in a separate thread
import cv2  # OpenCV for image processing
from PyQt5.QtCore import Qt, QPoint, pyqtSignal,QRectF
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel
from PyQt5.QtGui import QPainter, QColor, QPen, QBrush
from PyQt5.QtWidgets import QPushButton
from PyQt5.QtWidgets import QInputDialog, QMessageBox
import numpy as np # Scientific computing library for Python
import paho.mqtt.client as mqtt
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
class MapViewer():
    def __init__(self):
        super().__init__()
        self.isDataLoaded = False
        self.mapExists = False
        self.map = None
        self.costmapExists = False
        self.costmap = None
        self.mapdata = None
        self.costmapdata = None
        self.app = QApplication(sys.argv)
        self.ui = MapNavigationApp()
        self.ui.add_station_signal.connect(self.add_station)
        self.ui.map_clicked_and_dragged.connect(self.send_goal_pose)
        self.robot_status = "Idle, waiting for a goal..."
        self.ui.update_status(self.robot_status)

        # MQTT Setup
        self.mqtt_client = mqtt.Client()
        self.mqtt_broker = "localhost"  # Replace with your broker address
        self.mqtt_port = 1883  # Default MQTT port
        self.mqtt_topic = "robot/map"  # Topic for map data
        self.mqtt_pose = "robot/pose"
        self.mqtt_path = "robot/path"
        self.mqtt_costmap = "robot/costmap"
        self.setup_mqtt()

        # Other attributes
        self.map_data = None
        self.scale_factor = 4
        self.path = None
        self.robot_position = None
        self.stations = []

        # Load and save station data
        import atexit
        atexit.register(self.save_stations_to_json)
    def setup_mqtt(self):
        """Setup the MQTT client and connect to the broker."""
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        try:
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port)
            
            threading.Thread(target=self.mqtt_client.loop_forever, daemon=True).start()
            print("Connected to MQTT broker")
        except Exception as e:
            pass

    def on_mqtt_connect(self, client, userdata, flags, rc):
        """Callback when the MQTT client connects to the broker."""
        if rc == 0:
            print(f"MQTT connected successfully. Subscribing to topic: {self.mqtt_topic}")
            client.subscribe(self.mqtt_topic,qos=0)
            client.subscribe(self.mqtt_pose,qos=0)
            client.subscribe(self.mqtt_path,qos=0)
            client.subscribe(self.mqtt_costmap,qos=0)
            print(f"MQTT connected successfully. Subscribing to topic: {self.mqtt_pose}")
            print(f"MQTT connected successfully. Subscribing to topic: {self.mqtt_path}")
            print(f"MQTT connected successfully. Subscribing to topic: {self.mqtt_costmap}")
        else:
            self.get_logger().error(f"MQTT connection failed with code {rc}")

    def on_mqtt_message(self, client, userdata, message):

        payload = message.payload.decode("utf-8")
        data = json.loads(payload)
        self.validator(data)



    def validator(self,data):
        if 'map_info' in data:
            self.mapdata = data
            self.map = data.get('data',[])
            self.mapExists = True
            self.process_mqtt_map()
        if 'costmap_info' in data:
            self.costmapdata = data
            self.costmap = data.get('data',[])
            self.costmapExists = True
            self.process_mqtt_map()
        if 'AMR_POSE' in data:
            self.process_mqtt_pose(data['AMR_POSE'])
        if 'path' in data:
            self.path = data
            print(f"Path Length:{len(self.path)}")
    def validate_map_data(self, map_data):
        # Ensure 'info' and 'data' keys exist at the top level
        if 'info' not in map_data or 'data' not in map_data:
            print("Missing 'info' or 'data' in map_data")
            return

        # Check for required keys inside 'info'
        required_info_keys = ['resolution', 'width', 'height', 'origin']
        for key in required_info_keys:
            if key not in map_data['info']:
                print(f"Missing key in 'info': {key}")
                return

        # Ensure 'origin' contains 'x' and 'y'
        if 'x' not in map_data['info']['origin'] or 'y' not in map_data['info']['origin']:
            print("Missing 'x' or 'y' in 'origin'")
            return

        self.process_mqtt_map(map_data)
    def validate_pose_data(self, pose_data):
        # Check for top-level keys
        required_top_keys = ['position', 'orientation']
        for key in required_top_keys:
            if key not in pose_data:
                print(f"Missing key in pose_data: {key}")
                return

        # Check for required keys in 'position'
        required_position_keys = ['x', 'y', 'z']
        for key in required_position_keys:
            if key not in pose_data['position']:
                print(f"Missing key in 'position': {key}")
                return

        # Check for required keys in 'orientation'
        required_orientation_keys = ['x', 'y', 'z', 'w']
        for key in required_orientation_keys:
            if key not in pose_data['orientation']:
                print(f"Missing key in 'orientation': {key}")
                return

        self.process_mqtt_pose(pose_data)
    def process_mqtt_pose(self,pose_data):
        print("Processing received pose data...")
        try:
            
            self.robot_position = pose_data.get('position',{})
            self.robot_orientation = pose_data.get('orientation')
            self.robot_x = self.robot_position.get('x')
            self.robot_y = self.robot_position.get('y')
            self.robot_z = self.robot_position.get('z')

            print(f"x:{self.robot_x}, y:{self.robot_y}, orientation:{self.robot_orientation}")    
        except KeyError as e:
            print(f"Error processing MQTT message: {e}")
    def process_mqtt_map(self):
        self.update_ui()
    def update_ui(self):
        print("Updating UI.")
        """Update the UI with the new map, path, and robot position."""
        pixmap = None
        if self.mapExists:
            if not self.costmapExists and self.mapdata:
                pixmap = self.map_ui(self.mapdata)
                
            elif self.costmapExists:
                if self.mapdata and self.costmapdata:
                    overlayed_map = self.overlay_costmap_on_map(self.mapdata, self.costmapdata)
                    pixmap = self.convert_to_qimage(overlayed_map)
            
            if pixmap is not None:
                self.ui.set_map(pixmap)
        print("UI updated.")
    def map_ui(self,map_data):
        if self.mapExists:
            # Extract map dimensions and resolution
            info = map_data.get("map_info", {})
            resolution = info.get("resolution")
            width = info.get("width")
            height = info.get("height")
            self.origin = info.get("origin", {})
            origin_x = self.origin.get("x")
            origin_y = self.origin.get("y")
            self.data = map_data.get("data", [])

            # Convert OccupancyGrid data to numpy array
            map_array = np.array(self.data).reshape((height, width))
            map_array = np.flipud(map_array)

            # Map free (0) -> white, occupied (100) -> black, unknown (-1) -> gray
            display_map = np.zeros_like(map_array, dtype=np.uint8)
            display_map[map_array == 0] = 255  # Free space -> White
            display_map[map_array == 100] = 0  # Occupied -> Black
            display_map[map_array == -1] = 127  # Unknown -> Gray

            # Scale the map
            scaled_map = cv2.resize(
                display_map,
                (width * self.scale_factor, height * self.scale_factor),
                interpolation=cv2.INTER_NEAREST,
            )

            # Convert the grayscale map to an RGB mask
            rgb_mask = cv2.cvtColor(scaled_map, cv2.COLOR_GRAY2RGB)

            # Draw the robot's rectangle on the mask
            if self.robot_position:
                robot_pixel = self.convert_to_pixel(
                    resolution, origin_x, origin_y, height
                )

                # Define the robot rectangle (centered around robot_pixel)
                top_left = (robot_pixel[0] - 20, robot_pixel[1] - 20)
                bottom_right = (robot_pixel[0] + 20, robot_pixel[1] + 20)
                print(top_left)
                print(bottom_right)
                # Draw the rectangle in blue (BGR format)
                cv2.rectangle(
                    rgb_mask,
                    top_left,
                    bottom_right,
                    (0, 0, 255),  # Blue
                    -1,  # Filled rectangle
                )

            # Optionally, draw the path if available
            if self.path:
                for i in range(len(self.path) - 1):
                    start = self.convert_to_pixel(
                        self.path[i].pose.position, resolution, origin_x, origin_y, height
                    )
                    end = self.convert_to_pixel(
                        self.path[i + 1].pose.position, resolution, origin_x, origin_y, height
                    )
                    # Draw the path in red
                    cv2.line(
                        rgb_mask,
                        start,
                        end,
                        (0, 0, 255),  # Red
                        2,  # Line thickness
                    )

            # Convert the RGB mask to QImage for display
            scaled_height, scaled_width, _ = rgb_mask.shape
            qimage = QImage(
                rgb_mask.data,
                scaled_width,
                scaled_height,
                QImage.Format_RGB888,
            )
            pixmap = QPixmap.fromImage(qimage)
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

        print(f"Station '{station_name}' added at map coordinates: ({map_x}, {map_y})")
    def overlay_costmap_on_map(self, map_data, costmap_data):
        """
        Overlay costmap on the map and return an RGB image.
        """
        # Extract map details
        map_resolution = map_data.get('map_info').get('resolution')
        map_width = map_data.get('map_info').get('width')
        map_height = map_data.get('map_info').get('height')

        # Extract costmap details
        costmap_resolution = costmap_data.get('costmap_info').get('resolution')
        costmap_width = costmap_data.get('costmap_info').get('width')
        costmap_height = costmap_data.get('costmap_info').get('height')

        # Convert map and costmap data to numpy arrays
        
        map_array = np.array(map_data.get('data', [])).reshape(map_height, map_width)
        
        costmap_array = np.array(costmap_data.get('data', [])).reshape(costmap_height, costmap_width)

        # Debug original dimensions
        print(f"Original Map dimensions: {map_array.shape}, Original Costmap dimensions: {costmap_array.shape}")

        # Resize costmap based on self.scale_factor
        new_costmap_width = int(costmap_width * self.scale_factor)
        new_costmap_height = int(costmap_height * self.scale_factor)

        costmap_array = cv2.resize(
            costmap_array,
            (new_costmap_width, new_costmap_height),
            interpolation=cv2.INTER_NEAREST
        )

        # Debug resized dimensions
        print(f"Resized Costmap dimensions: {costmap_array.shape}")

        # Convert map to grayscale (values: 0 = black, 100 = white, -1 = gray)
        map_grayscale = np.zeros_like(map_array, dtype=np.uint8)
        map_grayscale[map_array == 0] = 255  # Free space -> white
        map_grayscale[map_array == 100] = 0  # Occupied -> black
        map_grayscale[map_array == -1] = 127  # Unknown -> gray

        # Normalize costmap values to 0-255 range
        costmap_normalized = cv2.normalize(costmap_array, None, 0, 255, cv2.NORM_MINMAX)

        # Create RGB map
        rgb_map = cv2.cvtColor(map_grayscale, cv2.COLOR_GRAY2RGB)
        if self.path:
            path = self.path.get('path', [])  # Ensure we get a list of path points
            print("PATH EXISTS", len(path))  # Debug the length of the path
            
            if len(path) > 1:  # At least two points are needed to draw a line
                for i in range(len(path) - 1):
                    # Convert start and end points to pixel coordinates
                    start = self.convert_to_pixel_path(
                        path[i]['position'],  # Assuming path[i] is a dict with 'position'
                        map_resolution,
                        map_data.get('map_info').get('origin').get('x'),
                        map_data.get('map_info').get('origin').get('y'),
                        map_height
                    )
                    end = self.convert_to_pixel_path(
                        path[i + 1]['position'],
                        map_resolution,
                        map_data.get('map_info').get('origin').get('x'),
                        map_data.get('map_info').get('origin').get('y'),
                        map_height
                    )
                    
                    # Ensure the points are within the bounds of the map
                    if (0 <= start[0] < rgb_map.shape[1] and 0 <= start[1] < rgb_map.shape[0] and
                        0 <= end[0] < rgb_map.shape[1] and 0 <= end[1] < rgb_map.shape[0]):
                        # Draw the path in red
                        cv2.line(
                            rgb_map,
                            start,
                            end,
                            (30, 227, 46),  # Red color
                            1,  # Line thickness
                        )
                    else:
                        print(f"Path segment out of bounds: start={start}, end={end}")
        # Overlay costmap on the map
        # If resizing causes a mismatch, crop or pad the costmap to match the map
        if rgb_map.shape[:2] != costmap_normalized.shape:
            resized_costmap = cv2.resize(
                costmap_normalized, 
                (map_width, map_height), 
                interpolation=cv2.INTER_NEAREST
            )
        else:
            resized_costmap = costmap_normalized

        # Create heatmap from resized costmap
        heatmap = cv2.applyColorMap(resized_costmap.astype(np.uint8), cv2.COLORMAP_JET)

        # Debug final array shapes
        print(f"RGB Map dimensions: {rgb_map.shape}, Heatmap dimensions: {heatmap.shape}")

        # Ensure dimensions match
        assert rgb_map.shape == heatmap.shape, "RGB map and heatmap dimensions must match!"

        # Overlay costmap as a heatmap
        overlayed_map = cv2.addWeighted(rgb_map, 0.8, heatmap, 0.2, 0)

        return overlayed_map
    def convert_to_pixel_path(self, position, resolution, origin_x, origin_y, map_height):
        """
        Convert a world position (x, y) to pixel coordinates.
        """
        x_pixel = int((position['x'] - origin_x) / resolution)
        y_pixel = int((position['y'] - origin_y) / resolution)
        
        # Flip y-axis because image origin is at top-left
        y_pixel = map_height - y_pixel - 1
        
        return (x_pixel, y_pixel)
    def convert_to_qimage(self, overlayed_map, scale_factor=4):
        """
        Convert an OpenCV RGB image to a QImage and scale it for the UI.
        """
        # Scale the map using OpenCV
        new_width = int(overlayed_map.shape[1] * scale_factor)
        new_height = int(overlayed_map.shape[0] * scale_factor)
        
        resized_map = cv2.resize(overlayed_map, (new_width, new_height), interpolation=cv2.INTER_LINEAR)

        # Convert resized OpenCV image to QImage
        height, width, channel = resized_map.shape
        bytes_per_line = 3 * width
        qimage = QImage(resized_map.data, width, height, bytes_per_line, QImage.Format_RGB888)
        return QPixmap.fromImage(qimage)
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
        print("Stations saved to stations.json")

    def load_stations_from_json(self):
        """Load stations from a JSON file."""
        if not os.path.exists("stations.json"):
            print("No stations.json file found. Starting with no stations.")
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

        print("Stations loaded from stations.json")
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
        print(f"AMR sent to station at: ({map_x}, {map_y})")
    def place_docking_station(self, x, y):
        """Place a docking station."""
        self.ui.add_docking_station(x, y)
        print(f"Placed docking station at pixel coordinates: ({x}, {y})")
    def map_callback(self, msg):
        """Callback function to process map data from the /map topic."""
        print("Received map data")
        self.map_data = msg
        if self.isDataLoaded == False:
            self.load_stations_from_json()

        
        self.update_ui()

    def plan_callback(self, msg):
        """Callback function to process path data from the /plan topic."""
        print("Received path data")
        self.path = msg.poses  # Store the sequence of poses
        self.update_ui()

    def pose_callback(self, msg):
        """Callback function to process the robot's position from /amcl_pose."""
        print("Received robot pose")
        self.robot_position = msg.pose.pose.position  # Store the robot's current position
        self.update_ui()



    def convert_to_pixel(self, resolution, origin_x, origin_y, height):
        """Convert map coordinates to pixel coordinates."""
        # X-coordinate conversion
        pixel_x = int((self.robot_x - origin_x) / resolution * self.scale_factor)
        # Y-coordinate conversion (account for flipped Y-axis in image coordinates)
        pixel_y = int((height - (self.robot_y - origin_y) / resolution) * self.scale_factor)
        return (pixel_x, pixel_y)

    def send_goal_pose(self, start_point, end_point):
        """Convert clicked and dragged points to map coordinates and publish a goal pose."""
        if self.map_data:
            resolution = self.map_data.info.resolution
            origin_x, origin_y = self.robot_x, self.robot_y
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
            print(f"Published goal pose: ({map_x}, {map_y}) with orientation: {angle} radians")
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
    
def main():
    
    node = MapViewer()
    ros_thread = threading.Thread(args=(node,))
    ros_thread.start()
    try:
        sys.exit(node.app.exec_())
    finally:
        node.stop()
        ros_thread.join()


if __name__ == '__main__':
    main()