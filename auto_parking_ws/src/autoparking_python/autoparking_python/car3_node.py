
from .car1_node import CarTestNode
import rclpy                                     # ROS2 Python接口库
from rclpy.node import Node                      # ROS2 节点类
from geometry_msgs.msg import Pose2D
'''
模拟的车辆节点3
'''
def main(args=None):                                 # ROS2节点主入口main函数
    pose = Pose2D()
    pose.x = 14.30                                    # 初始化的位姿
    pose.y = 45.57                                    # 初始化的位姿
    pose.theta = 0.0                                    # 初始化的位姿
    uwb_id = 3
    rclpy.init(args=args)                            # ROS2 Python接口初始化
    node = CarTestNode("car3",pose,uwb_id)     # 创建ROS2节点对象并进行初始化
    # node.send_request()                          # 发送服务请求

    rclpy.spin(node)
    node.destroy_node()                          # 销毁节点对象
    rclpy.shutdown()                             # 关闭ROS2 Python接口

if __name__ == '__main__':
    main()