import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import paho.mqtt.client as mqtt
import json


class MapMQTTPublisher(Node):
    def __init__(self):
        super().__init__('map_mqtt_publisher')
        
        # MQTT Broker Settings
        self.mqtt_client = mqtt.Client("ROS2_Map_Publisher")
        self.mqtt_client.connect("host.docker.internal", 1883)  # Replace with your broker IP
        self.mqtt_topic = "robot/map"

        # Subscribe to OccupancyGrid topic
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

    def map_callback(self, msg):
        # Convert OccupancyGrid to JSON
        map_data = {
            "info": {
                "resolution": msg.info.resolution,
                "width": msg.info.width,
                "height": msg.info.height,
                "origin": {
                    "x": msg.info.origin.position.x,
                    "y": msg.info.origin.position.y,
                    "z": msg.info.origin.position.z
                }
            },
            "data": msg.data  # List of occupancy grid values
        }

        # Publish map data to MQTT broker
        self.mqtt_client.publish(self.mqtt_topic, json.dumps(map_data))
        self.get_logger().info(f"Published map to MQTT: {self.mqtt_topic}")


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
