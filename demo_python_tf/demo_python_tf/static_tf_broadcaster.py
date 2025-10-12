import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from tf_transformations import quaternion_from_euler

class StaticTFBroadcaster(Node):
    def __init__(self):
        super().__init__('static_tf2_broadcaster')
        self.static_broadcaster_ = StaticTransformBroadcaster(self)
        self.publish_static_tf()

    def publish_static_tf(self):
        static_transform_stamped = TransformStamped()

        static_transform_stamped.header.stamp = self.get_clock().now().to_msg()
        static_transform_stamped.header.frame_id = 'base_link'
        static_transform_stamped.child_frame_id = 'camera_link'

        static_transform_stamped.transform.translation.x = 0.5
        static_transform_stamped.transform.translation.y = 0.3
        static_transform_stamped.transform.translation.z = 0.6

        quat = quaternion_from_euler(math.radians(180), 0, 0)
        static_transform_stamped.transform.rotation.x = quat[0]
        static_transform_stamped.transform.rotation.y = quat[1]
        static_transform_stamped.transform.rotation.z = quat[2]
        static_transform_stamped.transform.rotation.w = quat[3]

        self.static_broadcaster_.sendTransform(static_transform_stamped)
        self.get_logger().info('Publishing Static Transform from "base_link" to "camera_link"')

def main(args=None):
    rclpy.init(args=args)
    node = StaticTFBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()
