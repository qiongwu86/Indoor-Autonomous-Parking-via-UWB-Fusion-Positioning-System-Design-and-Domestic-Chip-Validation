#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
整合进系统
"""

import math
import rclpy                        # ROS2 Python接口库
from rclpy.node import Node         # ROS2 节点类
import random
from geometry_msgs.msg import Pose2D
from nav_2d_msgs.msg import Path2D
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from sensor_msgs.msg import Imu
import tf_transformations
from autoparking_interface.msg import ParkingPose
from autoparking_interface.srv import SetParking
from autoparking_interface.srv import RegisterCar

import numpy as np




class CarNode(Node):

    def __init__(self, name):
        super().__init__(name)                                          # ROS2节点父类初始化
        self.max_speed_ = 0.5                                           # 最大线速度
        self.max_yaw_ = 0.3027                                          # 最大角速度
        # 创建速度发布者 控制指令周期20ms
        self.vel_publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.vel_timer_ = self.create_timer(0.02, self.vel_publish)            # 创建一个定时器（单位为秒的周期，定时执行的回调函数）

    def vel_publish(self):
        message = Twist()
        # v=w*r
        message.linear.x = 0.0
        message.angular.z = np.deg2rad(30)
        # message.angular.z = 0.2

        message.linear.x *= 1.0
        message.angular.z *= 1.0

        # 限制最大速度
        if math.fabs(message.linear.x) > self.max_speed_:
            if message.linear.x < 0:  
                message.linear.x = -self.max_speed_
            else:
                message.linear.x = self.max_speed_
        if math.fabs(message.angular.z) > self.max_yaw_:
            if message.angular.z < 0:  
                message.angular.z = -self.max_yaw_
            else:
                message.angular.z = self.max_yaw_


        self.vel_publisher_.publish(message)
        self.get_logger().info(f'发布线速度:{message.linear.x}')
        self.get_logger().info(f'发布角速度:{message.angular.z}')

def main(args=None):                                 # ROS2节点主入口main函数
    # 3.启动节点
    rclpy.init(args=args)                            # ROS2 Python接口初始化
    node = CarNode("car1")        # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                                 # 循环等待ROS2退出
    node.destroy_node()                              # 销毁节点对象
    rclpy.shutdown()                                 # 关闭ROS2 Python接口

if __name__ == '__main__':
    main()


