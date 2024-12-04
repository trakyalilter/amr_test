import queue
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped  # Assuming PoseWithCovarianceStamped type for robot pose
import paho.mqtt.client as mqtt
import json
import threading
from nav2_simple_commander.robot_navigator import BasicNavigator


class MapMQTTPublisher(Node):
    def __init__(self):
        super().__init__('map_mqtt_publisher')

        # Initialize navigator
        self.nav = BasicNavigator()

        # Thread-safe queue for MQTT data
        self.mqtt_queue = queue.Queue()

        # MQTT Broker Settings
        self.mqtt_client = mqtt.Client("ROS2_Map_Publisher")
        self.mqtt_broker = "host.docker.internal"
        self.mqtt_port = 1883
        self.mqtt_topic_map = "robot/map"
        self.mqtt_topic_pose = "robot/pose"
        self.mqtt_topic_path = "robot/path"
        self.mqtt_topic_costmap = "robot/costmap"
        self.mqtt_topic_send_robot = "robot/sendrobot"

        # Setup MQTT
        self.setup_mqtt()

        # ROS Subscriptions
        self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self.costmap_callback, 10
        )
        self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10
        )
        self.create_subscription(
            PoseWithCovarianceStamped, '/pose', self.pose_callback, 10
        )
        self.create_subscription(
            Path, '/plan', self.path_callback, 10
        )

        # Timer to process MQTT messages from the queue
        self.create_timer(0.1, self.process_mqtt_queue)

    def setup_mqtt(self):
        """Setup the MQTT client and connect to the broker."""
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        try:
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port)
            threading.Thread(target=self.mqtt_client.loop_forever, daemon=True).start()
            self.get_logger().info("Connected to MQTT broker")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to MQTT broker: {e}")

    def on_mqtt_connect(self, client, userdata, flags, rc):
        """Callback when the MQTT client connects to the broker."""
        if rc == 0:
            client.subscribe(self.mqtt_topic_send_robot, qos=0)
            self.get_logger().info(f"MQTT connected successfully. Subscribing to topic: {self.mqtt_topic_send_robot}")
        else:
            self.get_logger().error(f"MQTT connection failed with code {rc}")

    def on_mqtt_message(self, client, userdata, message):
        """Callback for when an MQTT message is received."""
        payload = message.payload.decode("utf-8")
        data = json.loads(payload)
        self.mqtt_queue.put(data)  # Add the data to the queue for processing

    def process_mqtt_queue(self):
        """Process MQTT messages from the queue."""
        while not self.mqtt_queue.empty():
            data = self.mqtt_queue.get()
            if "sendRobot" in data:
                self.send_robot_to_pose(data)

    def send_robot_to_pose(self, data):
        """Send the robot to the specified pose."""
        self.get_logger().info(f"Sending Robot To Pose!!!")
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.get_clock().now().to_msg()

        goal_pose.pose.position.x = data['sendRobot']['position']['x']
        goal_pose.pose.position.y = data['sendRobot']['position']['y']
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = data['sendRobot']['orientation']['x']
        goal_pose.pose.orientation.y = data['sendRobot']['orientation']['y']
        goal_pose.pose.orientation.z = data['sendRobot']['orientation']['z']
        goal_pose.pose.orientation.w = data['sendRobot']['orientation']['w']

        self.nav.goToPose(goal_pose)

    def costmap_callback(self, msg):
        costmap_data = {
            "costmap_info": {
                "resolution": msg.info.resolution,
                "width": msg.info.width,
                "height": msg.info.height,
                "origin": {
                    "x": msg.info.origin.position.x,
                    "y": msg.info.origin.position.y,
                    "z": msg.info.origin.position.z,
                },
            },
            "data": list(msg.data),
        }
        self.mqtt_client.publish(self.mqtt_topic_costmap, json.dumps(costmap_data), qos=0)
        self.get_logger().info(f"Published map to MQTT: {self.mqtt_topic_costmap}")

    def map_callback(self, msg):
        map_data = {
            "map_info": {
                "resolution": msg.info.resolution,
                "width": msg.info.width,
                "height": msg.info.height,
                "origin": {
                    "x": msg.info.origin.position.x,
                    "y": msg.info.origin.position.y,
                    "z": msg.info.origin.position.z,
                },
            },
            "data": list(msg.data),
        }
        self.mqtt_client.publish(self.mqtt_topic_map, json.dumps(map_data), qos=0)
        self.get_logger().info(f"Published map to MQTT: {self.mqtt_topic_map}")

    def pose_callback(self, msg):
        pose_data = {
            "AMR_POSE": {
                "position": {
                    "x": msg.pose.pose.position.x,
                    "y": msg.pose.pose.position.y,
                    "z": msg.pose.pose.position.z,
                },
                "orientation": {
                    "x": msg.pose.pose.orientation.x,
                    "y": msg.pose.pose.orientation.y,
                    "z": msg.pose.pose.orientation.z,
                    "w": msg.pose.pose.orientation.w,
                },
            }
        }
        self.mqtt_client.publish(self.mqtt_topic_pose, json.dumps(pose_data), qos=0)
        self.get_logger().info(f"Published pose to MQTT: {self.mqtt_topic_pose}")

    def path_callback(self, msg):
        path_data = {
            "path": [
                {
                    "position": {
                        "x": pose.pose.position.x,
                        "y": pose.pose.position.y,
                        "z": pose.pose.position.z,
                    },
                    "orientation": {
                        "x": pose.pose.orientation.x,
                        "y": pose.pose.orientation.y,
                        "z": pose.pose.orientation.z,
                        "w": pose.pose.orientation.w,
                    },
                }
                for pose in msg.poses
            ]
        }
        self.mqtt_client.publish(self.mqtt_topic_path, json.dumps(path_data), qos=0)
        self.get_logger().info(f"Published path to MQTT: {self.mqtt_topic_path}")


def main(args=None):
    rclpy.init(args=args)
    node = MapMQTTPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
