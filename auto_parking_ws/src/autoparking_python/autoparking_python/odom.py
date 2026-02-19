import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import tf2_geometry_msgs
from autoparking_interface.msg import ParkingPose
from geometry_msgs.msg import Pose2D


def vectorized_rotate(points, angle_deg):
    """批量处理实现"""
    theta = np.radians(angle_deg)
    c, s = np.cos(theta), np.sin(theta)
    R = np.array([[c, -s], [s, c]])
    return points @ R.T

class OdomSubscriber(Node):
    def __init__(self):
        super().__init__('odom_subscriber')
        
        # 创建TF2缓冲器和转换器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 创建odom订阅器
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.odom_sub  # 避免pylint警告
        
        # 初始化坐标存储
        self.last_odom = None
        self.get_logger().info("Intializedodom订阅器已启动")

        self.car_name = 'car1'
        # 发布二维位姿信息
        self.publisher_ = self.create_publisher(ParkingPose, f'/{self.car_name}/my_odom', 10)   # 创建发布者对象（消息类型、话题名、队列长度）
        self.pose_timer = self.create_timer(0.05, self.odom_pub)            # 创建一个定时器（单位为秒的周期，定时执行的回调函数）
        self.x = 0.
        self.y = 0.

    def odom_pub(self):
        msg = ParkingPose()
        msg.car_name = self.car_name
        pose = Pose2D()
        pose.x = self.x
        pose.y = self.y
        pose.theta = 0.
        msg.current_pose = pose
        self.publisher_.publish(msg)    
    
    def odom_callback(self, msg):

        # 提取位置信息
        x = msg.pose.pose.position.x 
        y = msg.pose.pose.position.y 
        # 打印x和y
        # self.get_logger().info(f'Position:  x={x}, y={y}')

        points = np.array([[x, y]])
        # rotated = vectorized_rotate(points, 180)            # 旋转了一百八十度 TODO:
        rotated = vectorized_rotate(points, 270)            # 旋转了一百八十度 TODO:
        self.x = rotated[0][0]
        self.y = rotated[0][1]

def main(args=None):

    # 示例
    # points = np.array([[1, 2]])
    # # points = np.array([[1, 2], [3, 4], [5, 6]])
    # rotated = vectorized_rotate(points, 180)
    # print(f"批量旋转:\n{rotated}")

    rclpy.init(args=args)
    node = OdomSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


    # x = 0.1
    # y = 0.2
    # # 存储最新坐标
    # last_odom = {
    #     'x': x,
    #     'y': y,
    # }
    
    # # 变换
    # x = last_odom['x']
    # y = last_odom['y']
    # points = np.array([[x, y]])
    # rotated = vectorized_rotate(points, 180)
    # x = rotated[0][0]
    # y = rotated[0][1]

    # print(x,y)
    

