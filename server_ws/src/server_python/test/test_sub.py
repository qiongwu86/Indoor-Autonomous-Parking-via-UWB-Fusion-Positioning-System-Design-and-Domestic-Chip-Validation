#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
注册用的服务 服务端 ，用于返回规划好的路径, 并创建订阅者
"""


import rclpy                      # ROS2 Python接口库
from rclpy.node   import Node     # ROS2 节点类
from std_msgs.msg import String   # ROS2标准定义的String消息

from autoparking_interface.srv import RegisterCar
from geometry_msgs.msg import Pose2D
from nav_2d_msgs.msg import Path2D
from test2 import Vehicle
from autoparking_interface.msg import ParkingPose

class SubscriberNode(Node):

    def __init__(self, name):
        super().__init__(name)                             # ROS2节点父类初始化

        self.curren_num_car = 0                             # 统计车辆数
        self.parking_car = {}                               # 定义一个空字典来存储车辆对象 

        self.srv = self.create_service(RegisterCar, 'RegisterCar', self.adder_callback)  # 创建服务器对象（接口类型、服务名、服务器回调函数）

    def adder_callback(self, request, response):   # 创建回调函数，执行收到请求后对数据的处理
        """
        受到请求数据后,需返回规划好的路径,以及当前车辆在该停车场的id,以便于发布者发布
        """

        self.curren_num_car = self.curren_num_car + 1           # 注册的车辆数目加1
        response.car_id = self.curren_num_car

        # 接收数据
        car_name = request.car_name
        start_time = request.start_time
        initial_pose = request.initial_pose
        self.get_logger().info(f'汽车的名字是{request.car_name}') 

        # 创建车辆对象并添加到停车场列表中 
        vehicle = Vehicle(car_name, str(start_time), initial_pose) 
        self.parking_car[request.car_name] = vehicle             # 每次注册，将车辆记录在字典中

        topic_name = f'/{request.car_name}/parking'             # 设置订阅者的话题名字
        # self.get_logger().info(f'topic_name:{topic_name}') 
        # 创建订阅者
        subscription = self.create_subscription(ParkingPose,topic_name, self.subscriber_callback,1)

        # 路径规划部分：先 假设
        path = Path2D()
        pose1 = Pose2D()
        pose2 = Pose2D()
        pose1.x = 1.0
        pose2.y = 2.0
        path.poses.append(pose2) 
        path.poses.append(pose2)  
        response.path = path
        self.get_logger().info(f'路径点为{response.path.poses}')   # 输出日志信息

        response.parking_id = 1
        return response                          # 反馈应答信息
    # 订阅车俩的位置信息
    def subscriber_callback(self,msg):
        car_name = msg.car_name
        self.parking_car[car_name].update(msg.current_pose.x, msg.current_pose.y)       # 更新对应车辆的二维位姿

        self.get_logger().info(f'车辆{car_name}的位置为{msg.current_pose.x,msg.current_pose.y}') 

def main(args=None):                               # ROS2节点主入口main函数
    rclpy.init(args=args)                          # ROS2 Python接口初始化
    node = SubscriberNode("topic_helloworld_sub")  # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                               # 循环等待ROS2退出
    node.destroy_node()                            # 销毁节点对象
    rclpy.shutdown()                               # 关闭ROS2 Python接口

if __name__ == '__main__':
    main()