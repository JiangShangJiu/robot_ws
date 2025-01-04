import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformException

class TransformRelayNode(Node):
    def __init__(self):
        super().__init__('transform_relay_node')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(0.1, self.relay_transform)

    def relay_transform(self):
        try:
            # 获取当前时间
            current_time = self.get_clock().now()

            # 获取 fr3_link0 到 d435_depth_frame 的变换
            transform = self.buffer.lookup_transform('fr3_link0', 'd435_depth_frame', rclpy.time.Time())

            # 修改变换的时间戳为当前时间
            transform.header.stamp = current_time.to_msg()
            
            # 广播获取的变换
            self.broadcaster.sendTransform(transform)
            self.get_logger().info('Broadcasting transform from fr3_link0 to d435_depth_frame with updated timestamp')
        except TransformException as ex:
            self.get_logger().warn(f'Could not transform: {ex}')

def main(args=None):
    rclpy.init(args=args)
    transform_relay_node = TransformRelayNode()
    rclpy.spin(transform_relay_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
