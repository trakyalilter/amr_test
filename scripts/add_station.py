import sys
import time
import json
import math
import os
import threading
import cv2
from helper import Helper
from AMRButton import AMRButton
from WaypointButton import WaypointButton
import numpy as np
import paho.mqtt.client as mqtt
from Station import Station
from DbOperations import StationsDB
from Waypoint import Waypoint,WaypointManager
from PyQt5.QtCore import Qt, QPoint,QPointF, pyqtSignal,QRectF
from PyQt5.QtGui import QImage, QPixmap,QIcon,QPolygonF,QPainter, QColor, QPen, QBrush,QTransform,QCursor,QPainterPath
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout,QHBoxLayout, QLabel,QPushButton,QListWidget,QInputDialog, QMessageBox,QListWidgetItem
class MapOverlay(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAttribute(Qt.WA_TransparentForMouseEvents)
        self.setStyleSheet("background: transparent;")
        self.lines = []
        self.paths = []
        self.previous_paths = []
    def convert_to_pixel_path(self, position, resolution, origin_x, origin_y, map_height):
        """
        Convert a world position (x, y) to pixel coordinates.
        """
        scale_factor = 4
        x_pixel = int((position['x'] - origin_x) / resolution)*scale_factor
        y_pixel = int((map_height - (position['y'] - origin_y) / resolution))*scale_factor
    
        
        return (x_pixel, y_pixel)
    def add_line(self, start_point, end_point):
        self.lines.append((start_point, end_point))
        self.update()
    def draw_path(self, pathdata, mapdata):
        print("DRAWING PATH!")
        map_info = mapdata['map_info']
        map_resolution = map_info['resolution']
        map_height = map_info['height']
        map_origin_x = map_info['origin']['x']
        map_origin_y = map_info['origin']['y']
        path = pathdata.get('path', [])

        print("PrevLeng:",len(self.previous_paths))
        if len(path) <= 1:
            self.paths = self.previous_paths
        if len(path) > 1:
            raw_paths = []  # Temporarily store unsmoothed paths
            for i in range(len(path) - 1):
                start = self.convert_to_pixel_path(
                    path[i]['position'],
                    map_resolution,
                    map_origin_x,
                    map_origin_y,
                    map_height
                )
                end = self.convert_to_pixel_path(
                    path[i + 1]['position'],
                    map_resolution,
                    map_origin_x,
                    map_origin_y,
                    map_height
                )
                raw_paths.append((QPoint(*start), QPoint(*end)))

            # Smooth the paths
            self.paths = self.smooth_path(raw_paths)
            self.previous_paths = self.paths

        self.update()
    def clear_lines(self):
        self.lines.clear()
        self.update()

    def paintEvent(self, event):
                painter = QPainter(self)
                painter.setRenderHint(QPainter.Antialiasing)
                pen = QPen(Qt.blue, 2)
                pen.setStyle(Qt.DashLine)
                painter.setPen(pen)
                for line in self.lines:
                    start_point, end_point = line
                    painter.drawLine(start_point, end_point)
                    self._draw_arrowhead(painter, start_point, end_point)
                pen = QPen(QColor(38, 235, 67), 4)
                pen.setStyle(Qt.SolidLine)
                pen.setCapStyle(Qt.RoundCap)
                pen.setJoinStyle(Qt.RoundJoin)
                painter.setPen(pen)
                for path in self.paths:
                    start_point, end_point = path
                    painter.drawLine(start_point, end_point)
                self.paths = []
                painter.end()
    def smooth_path(self, path, window_size=7):
        """
        Apply a moving average filter to smooth the path.
        :param path: List of tuples (start_point, end_point), where each point is a QPoint.
        :param window_size: The size of the moving window (odd number).
        :return: Smoothed path as a list of tuples (start_point, end_point).
        """
        if len(path) < 2:
            return path  # Not enough segments to smooth

        # Flatten the list of tuples into a single list of points
        points = [point for segment in path for point in segment]

        smoothed_points = []
        half_window = window_size // 2

        for i in range(len(points)):
            x_sum = 0
            y_sum = 0
            count = 0

            for j in range(-half_window, half_window + 1):
                index = i + j
                if 0 <= index < len(points):  # Check bounds
                    x_sum += points[index].x()
                    y_sum += points[index].y()
                    count += 1

            avg_x = x_sum / count
            avg_y = y_sum / count
            smoothed_points.append(QPointF(avg_x, avg_y))

        # Reconstruct the path as pairs of points (start, end)
        smoothed_path = [
            (smoothed_points[i], smoothed_points[i + 1]) for i in range(len(smoothed_points) - 1)
        ]

        return smoothed_path
    def _draw_arrowhead(self, painter, start_point, end_point):
        arrow_size = 10
        dx = end_point.x() - start_point.x()
        dy = end_point.y() - start_point.y()
        angle = math.atan2(dy, dx)
        p1 = QPointF(
            end_point.x() - arrow_size * math.cos(angle - math.pi / 6),
            end_point.y() - arrow_size * math.sin(angle - math.pi / 6),
        )
        p2 = QPointF(
            end_point.x() - arrow_size * math.cos(angle + math.pi / 6),
            end_point.y() - arrow_size * math.sin(angle + math.pi / 6),
        )
        arrowhead = QPolygonF([end_point, p1, p2])
        painter.setBrush(Qt.blue)
        painter.drawPolygon(arrowhead)
class MapViewer():
    def __init__(self):
        super().__init__()
        self.helper = Helper()
        self.stationsDB = StationsDB()
        self.wpManager = WaypointManager()
        self.wpFile = 'wayPoints.json'
        self.waypoints =self.wpManager.load_waypoints_from_json(self.wpFile)
        # self.wp = Waypoint("wp1", [1, 2, 3])
        # self.wpManager.add_waypoint(self.wp)
        self.ui = None
        self.isDataLoaded = False
        self.mapExists = False
        self.map = None
        self.costmapExists = False
        self.costmap = None
        self.mapdata = None
        self.costmapdata = None
        self.app = QApplication(sys.argv)
        # self.ui.map_clicked_and_dragged.connect(self.send_goal_pose)
        self.isStationButtonsOnUI = False
        self.robot_status = None
        # MQTT Setup
        self.mqtt_client = mqtt.Client("Windows")
        self.mqtt_broker = "localhost"  
        self.mqtt_port = 1883
        self.mqtt_topic = "robot/map"
        self.mqtt_pose = "robot/pose"
        self.mqtt_path = "robot/path"
        self.mqtt_costmap = "robot/costmap"
        self.mqtt_topic_send_robot = "robot/sendrobot"
        self.mqtt_topic_nav_status = "robot/navstatus"
        self.mqtt_topic_initial_pose = "robot/initialPose"

        self.mqtt_topic_pathing = "robot/Pathing"
        self.setup_mqtt()
        # Other attributes
        self.map_data = None
        self.scale_factor = 4
        self.path = None
        self.robot_position = None
        self.stations = []

        time.sleep(5)
        self.ui = MapNavigationApp()
        self.ui.add_station_signal.connect(self.add_station)
        self.ui.set_initial_pose_signal.connect(self.set_initial_pose)
        self.ui.publish_path_signal.connect(self.publish_path)
        stationDB = StationsDB()
        if self.mapdata is not None and self.isStationButtonsOnUI == False:
            for station in stationDB.get_stations():
                self.add_station_from_db(station)
            self.isStationButtonsOnUI = True
        import atexit
        atexit.register(self.save_stations_to_json)
    def setup_mqtt(self):
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        try:
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port)
            threading.Thread(target=self.mqtt_client.loop_forever, daemon=True).start()
        except Exception as e:
            pass

    def on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            client.subscribe(self.mqtt_topic,qos=1)
            client.subscribe(self.mqtt_pose,qos=1)
            client.subscribe(self.mqtt_path,qos=1)
            client.subscribe(self.mqtt_costmap,qos=1)
            client.subscribe(self.mqtt_topic_nav_status,qos=1)
        else:
            pass

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
        if 'status_message' in data:
            self.robot_status = data['status_message']
            print("STATUSSS MESSAGE:",self.robot_status)
            #self.ui.status_update_signal.emit(self.robot_status)

    # def validate_map_data(self, map_data):
    #     # Ensure 'info' and 'data' keys exist at the top level
    #     if 'info' not in map_data or 'data' not in map_data:
    #         print("Missing 'info' or 'data' in map_data")
    #         return

    #     # Check for required keys inside 'info'
    #     required_info_keys = ['resolution', 'width', 'height', 'origin']
    #     for key in required_info_keys:
    #         if key not in map_data['info']:
    #             print(f"Missing key in 'info': {key}")
    #             return

    #     # Ensure 'origin' contains 'x' and 'y'
    #     if 'x' not in map_data['info']['origin'] or 'y' not in map_data['info']['origin']:
    #         print("Missing 'x' or 'y' in 'origin'")
    #         return

    #     self.process_mqtt_map(map_data)
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
        try:
            self.robot_position = pose_data.get('position',{})
            self.robot_orientation = pose_data.get('orientation')
            self.robot_x = self.robot_position.get('x')
            self.robot_y = self.robot_position.get('y')
            self.robot_z = self.robot_position.get('z')
        except KeyError as e:
            print(f"Error processing MQTT message: {e}")
    def process_mqtt_map(self):
        self.update_ui()
    def update_ui(self):
        """Update the UI with the new map, path, and robot position."""
        pixmap = None
        if self.mapExists and self.ui is not None:
            if not self.costmapExists and self.mapdata:
                pass
            elif self.costmapExists:
                if self.mapdata and self.costmapdata:
                    overlayed_map = self.overlay_costmap_on_map(self.mapdata, self.costmapdata)
                    if self.path:
                        print(f"TYPE:{type(self.path)} {type(self.mapdata)}")
                        self.ui.draw_path_signal.emit(self.path,self.mapdata)
                    pixmap = self.convert_to_qimage(overlayed_map)
            
            if pixmap is not None:
                self.ui.set_map(pixmap)
    def add_station_from_db(self, station:Station):
        info = self.mapdata["map_info"]
        resolution = info["resolution"]
        self.origin = info["origin"]
        origin_x = self.origin["x"]
        origin_y = self.origin["y"]
        scale_factor = self.scale_factor
        pixel_x = int((station.x - origin_x) / resolution * scale_factor)
        pixel_y = int((self.mapdata['map_info']["height"] - (station.y - origin_y) / resolution) * scale_factor)
        button = self.ui.add_station_button(QPoint(pixel_x, pixel_y),station_name=station.name)
        button.clicked.connect(lambda: self.send_amr_to_station(station.x, station.y,station.name))
    def add_station(self, position):
        """Handle adding a station to the map."""
        if not self.mapdata:
            QMessageBox.warning(self.ui, "Error", "Map data is not available yet!")
            return

        # Ask the user for the station name
        station_name, ok = QInputDialog.getText(self.ui, "Set Station Name", "Enter station name:")
        if not ok or not station_name.strip():
            QMessageBox.warning(self.ui, "Error", "Station name cannot be empty!")
            return

        info = self.mapdata.get("map_info", {})
        resolution = info.get("resolution")
        width = info.get("width")
        height = info.get("height")
        self.origin = info.get("origin", {})
        origin_x = self.origin.get("x")
        origin_y = self.origin.get("y")
    
        scale_factor = self.scale_factor

        # Convert pixel position to map coordinates
        map_x = ((position.x()+12) / scale_factor) * resolution + origin_x
        map_y = (height - (position.y()+12) / scale_factor) * resolution + origin_y
        # Store station data (map position and UI button)
        station_data = {
            "map_position": (map_x, map_y),
            "name": station_name,  # Save station name
            "button": None  # Placeholder for button reference
        }

        # Add button to the UI overlay
        button = self.ui.add_station_button(position,station_name=station_name)
        station = Station(name=station_name, x=map_x, y=map_y)
        self.stationsDB.add_station(station)
        # button.setText(station_name)  # Set button text as station name
        button.clicked.connect(lambda: self.send_amr_to_station(map_x, map_y,station_name))

        # Save button reference in station data
        station_data["button"] = button
        self.stations.append(station_data)

        # Save stations to JSON
        self.save_stations_to_json()
    def set_initial_pose(self,x,y,theta):
        info = self.mapdata.get("map_info", {})
        resolution = info.get("resolution")
        width = info.get("width")
        height = info.get("height")
        self.origin = info.get("origin", {})
        origin_x = self.origin.get("x")
        origin_y = self.origin.get("y")
    
        scale_factor = self.scale_factor
        quaternion = self.helper.quaternion_from_euler(0, 0, theta)
        print(theta)
        print(quaternion)
        # Convert pixel position to map coordinates
        map_x = (x / scale_factor) * resolution + origin_x
        map_y = (height - y / scale_factor) * resolution + origin_y

        initial_pose ={"initialPose":
                            {
                                "x": map_x,
                                "y": map_y,
                                "orientation":{
                                    "x":quaternion[0],
                                    "y":quaternion[1],
                                    "z":quaternion[2],
                                    "w":quaternion[3]
                                }
                            }
                        }
        self.mqtt_client.publish(self.mqtt_topic_initial_pose,json.dumps(initial_pose))
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

        # Flip the map_array and costmap_array along the x-axis
        # map_array = np.flipud(map_array)
        costmap_array = np.flipud(costmap_array)

        # Debug original dimensions
        # print(f"Original Map dimensions: {map_array.shape}, Original Costmap dimensions: {costmap_array.shape}")

        # Resize costmap based on self.scale_factor
        new_costmap_width = int(costmap_width * self.scale_factor)
        new_costmap_height = int(costmap_height * self.scale_factor)

        costmap_array = cv2.resize(
            costmap_array,
            (new_costmap_width, new_costmap_height),
            interpolation=cv2.INTER_NEAREST
        )

        # Debug resized dimensions
        # print(f"Resized Costmap dimensions: {costmap_array.shape}")

        # Convert map to grayscale (values: 0 = black, 100 = white, -1 = gray)
        map_grayscale = np.zeros_like(map_array, dtype=np.uint8)
        map_grayscale[map_array == 0] = 255  # Free space -> white
        map_grayscale[map_array == 100] = 0  # Occupied -> black
        map_grayscale[map_array == -1] = 127  # Unknown -> gray

        # Normalize costmap values to 0-255 range
        costmap_normalized = cv2.normalize(costmap_array, None, 0, 255, cv2.NORM_MINMAX)

        # Flip the map along the x-axis and ensure it is contiguous
        rgb_map = cv2.cvtColor(map_grayscale, cv2.COLOR_GRAY2RGB)
        rgb_map = np.flipud(rgb_map)  # Flip vertically
        rgb_map = np.ascontiguousarray(rgb_map)  # Ensure compatible layout


        # if self.path:
        #     path = self.path.get('path', [])
        #     if len(path) > 1:
        #         for i in range(len(path) - 1):
        #             start = self.convert_to_pixel_path(
        #                 path[i]['position'],
        #                 map_resolution,
        #                 map_data.get('map_info').get('origin').get('x'),
        #                 map_data.get('map_info').get('origin').get('y'),
        #                 map_height
        #             )
        #             end = self.convert_to_pixel_path(
        #                 path[i + 1]['position'],
        #                 map_resolution,
        #                 map_data.get('map_info').get('origin').get('x'),
        #                 map_data.get('map_info').get('origin').get('y'),
        #                 map_height
        #             )

        #             # Ensure the points are within bounds
        #             if (0 <= start[0] < rgb_map.shape[1] and 0 <= start[1] < rgb_map.shape[0] and
        #                 0 <= end[0] < rgb_map.shape[1] and 0 <= end[1] < rgb_map.shape[0]):
        #                 cv2.line(
        #                     rgb_map,
        #                     start,
        #                     end,
        #                     (30, 227, 46),  # Green color
        #                     1,  # Line thickness
        #                     lineType=cv2.LINE_AA,
        #                 )
        #             else:
        #                 print(f"Path segment out of bounds: start={start}, end={end}")
        if self.robot_position:
            robot_pixel = self.convert_to_pixel(
                map_resolution,
                map_data.get('map_info').get('origin').get('x'),
                map_data.get('map_info').get('origin').get('y'),
                map_height
            )
            orientation_angle = self.helper.quaternion_to_yaw(self.robot_orientation)
            orientation_angle_deg = np.degrees(orientation_angle)*-1
            self.ui.update_amr_signal.emit(robot_pixel, orientation_angle_deg)
        
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
        new_width = int(overlayed_map.shape[1] * scale_factor)
        new_height = int(overlayed_map.shape[0] * scale_factor)
        resized_map = cv2.resize(overlayed_map, (new_width, new_height), interpolation=cv2.INTER_LINEAR)
        height, width, channel = resized_map.shape
        bytes_per_line = 3 * width
        qimage = QImage(resized_map.data, width, height, bytes_per_line, QImage.Format_RGB888)
        return QPixmap.fromImage(qimage)
    def save_stations_to_json(self):        
        stations_data = []
        for station in self.stations:
            stations_data.append({
                "map_position": station["map_position"],
                "name": station.get("name", "Unnamed Station")
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
            station_name = station_data.get("name", "Unnamed Station")
            resolution = self.mapdata['info']['resolution']
            origin_x, origin_y = self.mapdata['info']['origin']['position']['x'],self.mapdata['info']['origin']['position']['y']
            scale_factor = self.scale_factor
            pixel_x = int((map_x - origin_x) / resolution * scale_factor)
            pixel_y = int((self.mapdata['info']["height"] - (map_y - origin_y) / resolution) * scale_factor)
            button = self.ui.add_station_button(QPoint(pixel_x, pixel_y),station_name)
            button.clicked.connect(lambda: self.send_amr_to_station(map_x, map_y,station_name))
            self.stations.append({
                "map_position": (map_x, map_y),
                "name": station_name,
                "button": button
            })
        self.isDataLoaded = True
    def publish_path(self, pathing):
        self.mqtt_client.publish(self.mqtt_topic_pathing, json.dumps(pathing))
    def send_amr_to_station(self, map_x, map_y,station_name):
        """Send the AMR to the selected station."""
        quaternion = self.helper.quaternion_from_euler(0, 0, 90)
        pose = {
            "sendRobot":{
                "position":{
                    "x":map_x,
                    "y":map_y
                },
                "orientation":{
                    "x":quaternion[0],
                    "y":quaternion[1],
                    "z":quaternion[2],
                    "w":quaternion[3]
                }
            }
        }
        self.mqtt_client.publish(self.mqtt_topic_send_robot,json.dumps(pose))
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
        pixel_x = int((self.robot_x - origin_x) / resolution)*self.scale_factor
        # Y-coordinate conversion (account for flipped Y-axis in image coordinates)
        pixel_y = int((height - (self.robot_y - origin_y) / resolution))*self.scale_factor
        return (pixel_x, pixel_y)
class MapNavigationApp(QWidget):
    map_clicked_and_dragged = pyqtSignal(QPoint, QPoint)
    add_station_signal = pyqtSignal(QPoint)
    update_amr_signal = pyqtSignal(tuple, float)
    station_added = pyqtSignal(float, float)
    status_update_signal = pyqtSignal(str)

    set_initial_pose_signal = pyqtSignal(float, float,float)
    draw_path_signal = pyqtSignal(dict,dict)
    publish_path_signal = pyqtSignal(dict)
    def __init__(self):
        super().__init__()
        self.wpManager= WaypointManager()
        self.sdb = StationsDB()
        self.mapdata = None
        # self.isStationsOnUI = False
        self.selected_waypoints = []  # Track selected waypoints
        self.lines = []  # Store lines between waypoints
        self.update_amr_signal.connect(self.update_amr_position)
        self.status_update_signal.connect(self.update_status)
        self.amr_button = None
        self.selectedPath = None
        self.isSettingInitialPose = False
        # Set up the window properties
        self.setWindowTitle("Y-Navigation")
        self.setGeometry(100, 100, 800, 600)

        # Create a QLabel to display the map
        self.label = QLabel(self)
        self.label.setAlignment(Qt.AlignCenter)

        self.overlay = MapOverlay(self)
        self.draw_path_signal.connect(self.overlay.draw_path)
        self.overlay.setGeometry(self.label.geometry())
        self.overlay.show()
        
        self.start_waypoint = None
        # Create a QLabel for the robot's status
        self.status_label = QLabel(self)
        self.status_label.setText("Robot Status: Waiting for goal...")
        self.status_label.setAlignment(Qt.AlignCenter)
        # Set up layout and add label
        self.layout = QVBoxLayout()
        self.layout.addWidget(self.label)
        self.layout.addWidget(self.status_label)

        self.bottomHorizontalLayout = QHBoxLayout()
        self.layout.addLayout(self.bottomHorizontalLayout)
        self.setInitialPoseButton = QPushButton("Set Initial Pose", self)
        self.button_style = """
        QPushButton {
            background: rgba(94, 65, 222, 0.2); /* Equivalent to #5e41de33 */
            border: none;
            border-radius: 20px;
            padding: 6px 12px;
            min-height: 36px;
            min-width: 120px;
            font-size: 17px;
            font-family: sans-serif;
            color: #5D41DE;
            letter-spacing: 1px;
        }

        QPushButton:hover {
            background: rgba(94, 65, 222, 0.3); /* Equivalent to #5e41de4d */
        }
        """
        self.setInitialPoseButton.setStyleSheet(self.button_style)
        self.setInitialPoseButton.clicked.connect(self.initialPoseButtonClicked)
        self.bottomHorizontalLayout.addWidget(self.setInitialPoseButton)
        self.loopWayPoints = QPushButton("Loop Waypoints", self)
        self.loopWayPoints.setStyleSheet(self.button_style)
        self.loopWayPoints.clicked.connect(self.looping)
        self.bottomHorizontalLayout.addWidget(self.loopWayPoints)

        self.MainLayout = QHBoxLayout()
        self.MainLayout.addLayout(self.layout)
        self.rightLO = QVBoxLayout() 
        self.pathsListWidget = QListWidget()
        self.rightLO.addWidget(self.pathsListWidget)
        self.MainLayout.addLayout(self.rightLO)
        self.setLayout(self.MainLayout)
        self.map_metadata = {}
        self.start_point = None
        self.end_point = None
        self.docking_station = None
        self.show()
    def initialPoseButtonClicked(self):
        initialPoseCursor = QCursor(QPixmap('map-pin.png'))
        self.label.setCursor(initialPoseCursor)
        self.isSettingInitialPose = True

    def sett_amr_pose(self, position, angle):
        if self.amr_button is None:
            self.amr_button = QPushButton(self)
            self.amr_button.setFixedSize(50, 50)

        # Load the AMR image
        pixmap = QPixmap("AMR.png")
        if pixmap.isNull():
            raise FileNotFoundError("Robot image (AMR.png) not found!")

        # Rotate the pixmap based on the angle
        transform = QTransform().rotate(angle)
        rotated_pixmap = pixmap.transformed(transform, mode=Qt.SmoothTransformation)

        # Set the rotated pixmap as the button's icon
        self.amr_button.setIcon(rotated_pixmap)
        self.amr_button.setIconSize(rotated_pixmap.size())

        # Update button position on the UI
        self.amr_button.move(position.x(), position.y())
        self.amr_button.show()

    def update_amr_position(self, robot_pixel,orientation_angle_deg):
        self.sett_amr_pose(robot_pixel[0], robot_pixel[1], orientation_angle_deg)
    def set_map(self, pixmap):
        """Update the displayed map in the UI."""
        self.label.setPixmap(pixmap)
        self.label.setAlignment(Qt.AlignCenter)
        self.label.adjustSize()  # Adjust the QLabel to fit the new image
        self.overlay.setGeometry(self.label.geometry())
    def connect_waypoints(self, start_point, end_point):
        """Draw a line between two waypoints."""
        self.overlay.add_line(start_point, end_point)
    
    def draw_path(self, path,mapdata):
        self.overlay.draw_path(path,mapdata)
    def clear_connections(self):
        """Clear all waypoint connections."""
        self.overlay.clear_lines()
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
    def add_station_button(self, position,station_name):
        """Add a custom button to the map to represent a station."""
        button = WaypointButton(station_name,self)  # Use CustomButton instead of QPushButton
        button.clicked_with_ctrl.connect(self.handle_waypoint_click)
        button.ctrl_released.connect(self.createWayPointPath)
        button.setGeometry(position.x()+12, position.y()+12, 24, 24)
        button.show()
        return button
    def sett_amr_pose(self, x, y, angle):
        if not hasattr(self, 'amr_button') or self.amr_button is None:
            self.amr_button = AMRButton("Robot.png", self)
        self.amr_button.set_image_angle(angle)
        self.amr_button.move(x, y)
        self.amr_button.show()
    def mousePressEvent(self, event):
        if event.button() == Qt.RightButton and self.label.pixmap():
            label_pos = self.label.pos()
            map_pos = event.pos() - label_pos
            self.add_station_signal.emit(map_pos)
        if event.button() == Qt.LeftButton and self.label.pixmap():
            label_pos = self.label.pos()
            self.start_point = event.pos() - label_pos
            if self.isSettingInitialPose:
                self.initialPose = event.pos() - self.label.pos()
                
                self.label.setCursor(QCursor(Qt.ArrowCursor))
                
    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton and self.start_point and self.isSettingInitialPose:
            label_pos = self.label.pos()
            self.end_point = event.pos() - label_pos
            theta = math.atan2(self.start_point.y()-self.end_point.y()  , self.start_point.x()- self.end_point.x()) + math.pi/2
            self.set_initial_pose_signal.emit(self.initialPose.x(), self.initialPose.y(),theta)
            self.isSettingInitialPose = False
            self.map_clicked_and_dragged.emit(self.start_point, self.end_point)
            self.start_point = None
            self.end_point = None
    def mouseDoubleClickEvent(self, event):
        if event.button() == Qt.LeftButton and self.label.pixmap():
            label_pos = self.label.pos()
            click_point = event.pos() - label_pos
            self.station_added.emit(click_point.x(), click_point.y())

    def handle_waypoint_click(self, waypoint_button):
        """Handle waypoint click for drawing sequential lines."""
        # wpManager = WaypointManager()
        if waypoint_button not in self.selected_waypoints:
            self.selected_waypoints.append(waypoint_button)
        # If more than one waypoint is selected, draw lines sequentially
        if len(self.selected_waypoints) > 1:
            for i in range(len(self.selected_waypoints) - 1):
                start_button = self.selected_waypoints[i]
                end_button = self.selected_waypoints[i + 1]
                start_point = start_button.pos() + QPoint(
                    start_button.width() // 2, start_button.height() // 2
                )
                end_point = end_button.pos() + QPoint(
                    end_button.width() // 2, end_button.height() // 2
                )
                self.connect_waypoints(start_point, end_point)
    def createWayPointPath(self):
        sdb = StationsDB()
        
        wp_name, ok = QInputDialog.getText(self, "Set Path Name", "Enter path name:")
        if not ok or not wp_name.strip():
            QMessageBox.warning(self, "Error", "Path name cannot be empty!")
            return
        if len(self.selected_waypoints) >= 2:
            wp = Waypoint(wp_name,[self.sdb.get_station(x.station_name).id for x in self.selected_waypoints])
            self.wpManager.add_waypoint(wp)
            QListWidgetItem(wp_name,self.pathsListWidget)
            self.selected_waypoints.clear()
    def looping(self):
        _ = self.pathsListWidget.currentItem().text()
        id_list = [x for x in self.wpManager.load_waypoints_from_json('wayPoints.json') if x.name == _][0].id_list
        stationsList = [station for station in self.sdb.get_stations() if station.id in id_list]
        pathing = {
            "Pathing":[{
                   "x":station.x,
                   "y":station.y 
            }for station in stationsList
            ]
        }
        self.publish_path_signal.emit(pathing)
        print(pathing)
        print("LOOOOOOOOOOOOOOOOOOOOOOOOP")
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
