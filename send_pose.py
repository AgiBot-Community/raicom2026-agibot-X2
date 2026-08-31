#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped

class ContinuousPoseSender(Node):
    def __init__(self):
        super().__init__('continuous_pose_sender_node')
        
        # 1. 声明发布者：持续发给你们自己的导航包
        self.pub = self.create_publisher(
            PoseStamped, 
            '/map_tf_distribution/localization_pose', 
            10
        )
        
        # 2. 声明订阅者：获取 3D 雷达的实时精准定位
        self.sub = self.create_subscription(
            PoseWithCovarianceStamped, 
            '/pcl_pose', 
            self.pose_callback, 
            10
        )
        self.get_logger().info("🚀 实时坐标转发节点已启动！正在持续转发...")

    def pose_callback(self, msg):
        # 只要收到雷达定位坐标，就立刻转换格式
        out_msg = PoseStamped()
        
        # 剥离出 header 和 pose，丢弃不需要的协方差数据
        out_msg.header = msg.header
        out_msg.pose = msg.pose.pose 
        
        # 持续自动发送消息
        self.pub.publish(out_msg)
        
        # 注意：这里去掉了 sys.exit(0)
        # 为了防止终端被日志淹没，这里也去掉了每次发送都打印坐标的 print

def main(args=None):
    rclpy.init(args=args)
    node = ContinuousPoseSender()
    try:
        # spin 会让节点一直保持运行状态，直到你按 Ctrl+C
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("停止转发。")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()