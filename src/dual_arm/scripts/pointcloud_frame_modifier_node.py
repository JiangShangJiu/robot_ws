import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

class PointCloudFrameModifier(Node):
    def __init__(self):
        super().__init__('pointcloud_frame_modifier')
        
        # 订阅原始点云话题 (FP3)
        self.subscription_fp3 = self.create_subscription(
            PointCloud2,
            '/fp3_camera/points',  # 原始点云话题 (FP3)
            self.listener_callback_fp3,
            10
        )
        self.subscription_fp3  # 避免未被使用的警告

        # 订阅原始点云话题 (FR3)
        self.subscription_fr3 = self.create_subscription(
            PointCloud2,
            '/fr3_camera/points',  # 原始点云话题 (FR3)
            self.listener_callback_fr3,
            10
        )
        self.subscription_fr3  # 避免未被使用的警告

        # 发布修改后的点云话题 (FP3)
        self.publisher_fp3 = self.create_publisher(
            PointCloud2,
            '/fp3_camera/points_modified',  # 修改后的点云话题 (FP3)
            10
        )

        # 发布修改后的点云话题 (FR3)
        self.publisher_fr3 = self.create_publisher(
            PointCloud2,
            '/fr3_camera/points_modified',  # 修改后的点云话题 (FR3)
            10
        )

        self.get_logger().info('Node initialized and ready to modify frame_id')

    def listener_callback_fp3(self, msg):
        # 修改 FP3 点云的 frame_id
        modified_msg_fp3 = msg
        modified_msg_fp3.header.frame_id = 'fp3_camera_depth_frame'
        
        # 发布修改后的消息 (FP3)
        self.publisher_fp3.publish(modified_msg_fp3)
        self.get_logger().info('Published modified PointCloud2 with frame_id: fp3_depth_frame')

    def listener_callback_fr3(self, msg):
        # 修改 FR3 点云的 frame_id
        modified_msg_fr3 = msg
        modified_msg_fr3.header.frame_id = 'fr3_camera_depth_frame'
        
        # 发布修改后的消息 (FR3)
        self.publisher_fr3.publish(modified_msg_fr3)
        self.get_logger().info('Published modified PointCloud2 with frame_id: fr3_depth_frame')

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
