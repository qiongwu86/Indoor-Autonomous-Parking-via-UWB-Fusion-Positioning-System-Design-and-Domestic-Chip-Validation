from .car1_start import UserlNode
import rclpy
from rclpy.node import Node

def main(args=None):
    rclpy.init(args=args)
    node = UserlNode(name="user3_node",car_name="car3")
    rclpy.spin(node=node)
    node.destroy_node()                          # 销毁节点对象
    rclpy.shutdown()

if __name__ == '__main__':
    main()

