import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

class PointCloudFrameModifier(Node):
    def __init__(self):
        super().__init__('pointcloud_frame_modifier')
        
        # 订阅原始点云话题
        self.subscription = self.create_subscription(
            PointCloud2,
            '/rgbd_camera/points',  # 原始点云话题
            self.listener_callback,
            10
        )
        self.subscription  # 避免未被使用的警告

        # 发布修改后的点云话题
        self.publisher = self.create_publisher(
            PointCloud2,
            '/rgbd_camera/points_modified',  # 修改后的点云话题
            10
        )
        self.get_logger().info('Node initialized and ready to modify frame_id')

    def listener_callback(self, msg):
        # 修改 frame_id
        modified_msg = msg
        modified_msg.header.frame_id = 'd435_depth_frame'
        
        # 发布修改后的消息
        self.publisher.publish(modified_msg)
        self.get_logger().info('Published modified PointCloud2 with frame_id: d435_depth_frame')

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudFrameModifier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
