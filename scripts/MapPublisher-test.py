import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped  # Assuming PoseWithCovarianceStamped type for robot pose
import paho.mqtt.client as mqtt
import json

class MapMQTTPublisher(Node):
    def __init__(self):
        super().__init__('map_mqtt_publisher')
        
        # MQTT Broker Settings
        self.mqtt_client = mqtt.Client("ROS2_Map_Publisher")
        self.mqtt_client.connect("host.docker.internal", 1883)  # Replace with your broker IP
        self.mqtt_topic_map = "robot/map"
        self.mqtt_topic_pose = "robot/pose"  # Separate topic for the robot's pose

        # Subscribe to OccupancyGrid topic
        self.subscription_map = self.create_subscription(
            OccupancyGrid,
            '/map',  # Map topic
            self.map_callback,
            10
        )

        # Subscribe to Pose topic (e.g., /amcl_pose or /robot_pose)
        self.subscription_pose = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',  # Replace with your pose topic if it's different
            self.pose_callback,
            10
            )

    def map_callback(self, msg):
        map_data = {
            "map_info": {
                "resolution": msg.info.resolution,
                "width": msg.info.width,
                "height": msg.info.height,
                "origin": {
                    "x": msg.info.origin.position.x,
                    "y": msg.info.origin.position.y,
                    "z": msg.info.origin.position.z
                }
            },
            "data": list(msg.data)
        }
        self.mqtt_client.publish(self.mqtt_topic_map, json.dumps(map_data),qos=0)
        self.get_logger().info(f"Published map to MQTT: {self.mqtt_topic_map}")

    def pose_callback(self, msg):
        # Extract position and orientation from PoseWithCovarianceStamped
        pose_data = { 
            "AMR_POSE":{
            "position": {
                "x": msg.pose.pose.position.x,
                "y": msg.pose.pose.position.y,
                "z": msg.pose.pose.position.z
            },
            "orientation": {
                "x": msg.pose.pose.orientation.x,
                "y": msg.pose.pose.orientation.y,
                "z": msg.pose.pose.orientation.z,
                "w": msg.pose.pose.orientation.w
            }
            }
        }
        self.mqtt_client.publish(self.mqtt_topic_pose, json.dumps(pose_data),qos=0)
        self.get_logger().info(f"Published pose to MQTT: {self.mqtt_topic_pose}")


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


if __name__ == '__main__':
    main()
