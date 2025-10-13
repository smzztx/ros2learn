import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from tf_transformations import euler_from_quaternion

class TFListener(Node):
    def __init__(self):
        super().__init__('tf_listener')
        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_, self)
        self.timer_ = self.create_timer(0.5, self.lookup_transform)

    def lookup_transform(self):
        try:
            result = self.tf_buffer_.lookup_transform(
                'base_link',
                'bottle_link',
                rclpy.time.Time(seconds=0),
                rclpy.time.Duration(seconds=1))
            transform = result.transform
            euler = euler_from_quaternion([
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w
            ])
            self.get_logger().info(
                f'Translation: ={transform.translation} | '
                f'quaternion: {transform.rotation} | '
                f'rpy: {euler}'
                
            )
        except Exception as e:
            self.get_logger().warn(f'Could not transform: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = TFListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()