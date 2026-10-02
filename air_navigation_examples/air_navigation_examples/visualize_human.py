import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped
from std_msgs.msg import ColorRGBA
from builtin_interfaces.msg import Duration
from air_simple_sim_msgs.msg import SemanticObservation
from tf2_ros import TransformException, Buffer, TransformListener
import tf2_geometry_msgs
from rclpy.time import Time
from rclpy.duration import Duration as RclpyDuration


def create_color(r, g, b, a):
    c = ColorRGBA()
    c.r, c.g, c.b, c.a = r, g, b, a
    return c


class VisualizeHumansFromTopic(Node):
    def __init__(self):
        super().__init__('visualize_humans_from_topic')
        self.timer_period = 0.1

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.human_positions = []  # list of tuples: (uuid, x, y, z)

        self.create_subscription(
            SemanticObservation,
            '/semantic_sensor_hf',
            self.semantic_callback,
            10
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            'semantic_sensor_visualisation',
            10
        )

        self.create_timer(self.timer_period, self._on_timer)
        self.get_logger().info('VisualizeHumansFromTopic (simplified) started.')

    def semantic_callback(self, msg: SemanticObservation):
        if 'human' not in msg.klass.lower():
            return

        point_stamped = PointStamped()
        point_stamped.header.stamp = Time(seconds=0).to_msg()  # use latest available
        point_stamped.header.frame_id = msg.point.header.frame_id
        point_stamped.point = msg.point.point

        try:
            transformed = self.tf_buffer.transform(
                point_stamped,
                target_frame="map",
                timeout=RclpyDuration(seconds=0.5)
            )
            x, y, z = transformed.point.x, transformed.point.y, transformed.point.z
            self.get_logger().info(f"Detected human: {msg.uuid} at ({x:.2f}, {y:.2f})")
            if not any(h[0] == msg.uuid for h in self.human_positions):
                self.human_positions.append((msg.uuid, x, y, z))
                self.get_logger().info(f"New human: {msg.uuid} at ({x:.2f}, {y:.2f})")
        except TransformException as ex:
            self.get_logger().warn(f"TF failed for {msg.uuid}: {ex}")

    def _on_timer(self):
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        for idx, (uid, x, y, z) in enumerate(self.human_positions):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = now
            marker.id = idx
            marker.ns = "humans"
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = z
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.5
            marker.color = create_color(1.0, 0.0, 0.0, 1.0)
            marker.lifetime = Duration(sec=1)  # short lifetime so old ones fade
            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)
        self.get_logger().info(f"Published {len(marker_array.markers)} human marker(s)")
        self.human_positions.clear()  # clear for next round


def main():
    rclpy.init()
    node = VisualizeHumansFromTopic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
