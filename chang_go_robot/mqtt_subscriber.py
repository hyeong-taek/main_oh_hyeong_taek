#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
import random
from paho.mqtt import client as mqtt_client
from rclpy.qos import QoSProfile
# =====================
# MQTT 설정
# =====================
broker = 'c41c91fd.ala.eu-central-1.emqxsl.com'
port = 8883
username = 'taek'
password = '991031'
topic = "mqtt_empty_shelf"
client_id = f'python-mqtt-{random.randint(0, 100)}'
class MQTTSubscriberNode(Node):
    def __init__(self):
        super().__init__('mqtt_subscriber_node')
        # ROS2 퍼블리셔: MQTT 메시지를 퍼블리시할 토픽

        qos = QoSProfile(depth=200)

        self.ros_pub = self.create_publisher(Int8, '/empty_shelf', qos)
        
        # MQTT 연결
        self.mqtt_client = self.connect_mqtt()
        self.mqtt_client.loop_start()  # MQTT 비동기 루프 시작
        self.get_logger().info("MQTT Subscriber Node started!")
    def connect_mqtt(self) -> mqtt_client.Client:
        def on_connect(client, userdata, flags, rc):
            if rc == 0:
                self.get_logger().info("Connected to MQTT Broker!")
                client.subscribe(topic)
            else:
                self.get_logger().error(f"Failed to connect, return code {rc}")
        def on_message(client, userdata, msg):
            decoded_msg = msg.payload.decode()
            self.get_logger().info(f"Received `{decoded_msg}` from `{msg.topic}`")
            # ROS2 토픽으로 퍼블리시
            ros_msg = Int8()
            try:
                if decoded_msg.lower() in ['true', 'false']:
                    ros_msg.data = int(decoded_msg.lower() == 'true')  # true → 1, false → 0
                else:
                    ros_msg.data = int(decoded_msg)  # 숫자로 들어오면 그거 씀

                self.ros_pub.publish(ros_msg)

            except ValueError:
                self.get_logger().error(f"❌ MQTT 메시지 파싱 실패: `{decoded_msg}` → int 변환 불가")

        
            # ros_msg.data = int(decoded_msg)
            # self.ros_pub.publish(ros_msg)
            
        client = mqtt_client.Client(client_id=client_id, protocol=mqtt_client.MQTTv311)
        client.tls_set()  # TLS 활성화
        client.username_pw_set(username, password)
        client.on_connect = on_connect
        client.on_message = on_message
        client.connect(broker, port)
        return client
def main(args=None):
    rclpy.init(args=args)
    node = MQTTSubscriberNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("MQTT Subscriber stopped by user.")
    finally:
        node.mqtt_client.loop_stop()
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()