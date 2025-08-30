import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo
from visualization_msgs.msg import Marker  # 추가
import tf2_ros
from tf2_geometry_msgs import do_transform_point
from rclpy.duration import Duration
class ObjectXYZ(Node):
    def __init__(self):
        super().__init__('object_xyz')
        self.camera_intrinsics = None
        self.intrinsics_received = False
        self.camera_frame_id = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/robot0/oakd/rgb/camera_info',
            self.camera_info_callback,
            10)
        self.detection_sub = self.create_subscription(
            PointStamped,
            '/object_detection_info',
            self.detection_callback,
            10)
        self.map_point_pub = self.create_publisher(PointStamped, '/map_point', 10)
        # Marker 퍼블리셔 추가
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)
        self.get_logger().info("객체 좌표 변환 노드 시작. 카메라 정보를 기다립니다...")
    def camera_info_callback(self, msg):
        if not self.intrinsics_received:
            self.camera_intrinsics = {
                'fx': msg.k[0],
                'fy': msg.k[4],
                'cx': msg.k[2],
                'cy': msg.k[5]
            }
            self.camera_frame_id = msg.header.frame_id
            self.intrinsics_received = True
            self.get_logger().info(f"카메라 내부 파라미터 수신 완료: {self.camera_intrinsics}")
            self.destroy_subscription(self.camera_info_sub)
    def detection_callback(self, msg):
        if not self.intrinsics_received:
            self.get_logger().warn("아직 카메라 정보를 수신하지 못했습니다.")
            return
        u = msg.point.x
        v = msg.point.y
        depth = msg.point.z
        if depth <= 0:
            self.get_logger().warn(f"유효하지 않은 깊이 값({depth})")
            return
        fx = self.camera_intrinsics['fx']
        fy = self.camera_intrinsics['fy']
        cx = self.camera_intrinsics['cx']
        cy = self.camera_intrinsics['cy']
        x_cam = (u - cx) * depth / fx
        y_cam = (v - cy) * depth / fy
        z_cam = depth
        point_in_camera_frame = PointStamped()
        point_in_camera_frame.header.stamp = msg.header.stamp
        point_in_camera_frame.header.frame_id = self.camera_frame_id
        point_in_camera_frame.point.x = x_cam
        point_in_camera_frame.point.y = y_cam
        point_in_camera_frame.point.z = z_cam
        try:
            target_frame = 'map'
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                point_in_camera_frame.header.frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0)
            )
            point_in_map_frame = do_transform_point(point_in_camera_frame, transform)
            self.map_point_pub.publish(point_in_map_frame)
            self.get_logger().info(f"맵 좌표계로 변환된 좌표 발행: {point_in_map_frame.point}")
            # Marker 메시지 생성 및 발행
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'object_marker'
            marker.id = 0
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = point_in_map_frame.point
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.15  # 크기 설정
            marker.scale.y = 0.15
            marker.scale.z = 0.15
            marker.color.a = 1.0  # 투명도
            marker.color.r = 1.0
            marker.color.g = 0.2
            marker.color.b = 0.2
            self.marker_pub.publish(marker)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f"좌표 변환 실패: {e}")
def main(args=None):
    rclpy.init(args=args)
    node = ObjectXYZ()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()