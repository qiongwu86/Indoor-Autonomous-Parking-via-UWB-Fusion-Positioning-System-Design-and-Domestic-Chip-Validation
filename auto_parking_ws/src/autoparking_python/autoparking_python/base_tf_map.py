import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster  # 坐标发布器
from geometry_msgs.msg import TransformStamped  # 消息接口
from tf_transformations import quaternion_from_euler    # 欧拉角转四元数
import math
from geometry_msgs.msg import Pose2D
from autoparking_interface.msg import ParkingPose

"""
用于发布TF
"""

class TFBroadcaster(Node):
    def __init__(self):
        super().__init__('tf_broadcaster')
        self.broadcaster_ = TransformBroadcaster(self)      # 创建广播器
        # self.publish_static_tf()
        # self.timers_ = self.create_timer(1,self.publish_tf)
        
        self.sub = self.create_subscription(                        # 可能需要改话题格式
            ParkingPose, '/car1/parking', self.listener_callback, 10)     # 创建订阅者对象（消息类型、话题名、订阅者回调函数、队列长度）

    def listener_callback(self, msg):
        pose = Pose2D()
        pose.x = msg.current_pose.x
        pose.y = msg.current_pose.y
        pose.theta = msg.current_pose.theta
        self.get_logger().info(f'Receiving data:{pose}')         # 输出日志信息，提示已进入回调函数
        self.publish_tf(pose)

    def publish_tf(self, msg):
        """
        发布 TF 从 map 到 base_link 之间的坐标关系
        """
        transform = TransformStamped()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'base_link'
        transform.header.stamp = self.get_clock().now().to_msg()

        transform.transform.translation.x = msg.x
        transform.transform.translation.y = msg.y
        transform.transform.translation.z = 0.0
        # 欧拉角，返回一个四元数元组 x,y,z,w
        q = quaternion_from_euler(msg.theta,0,0)
        # 旋转部分进行赋值
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]
        # 坐标发布出去
        self.broadcaster_.sendTransform(transform)
        # self.get_logger().info(f'发布TF:{transform}')

def main():

    rclpy.init()
    node = TFBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()
 
