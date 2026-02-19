#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
模拟的车辆节点1
'''

import rclpy                                     # ROS2 Python接口库
from rclpy.node import Node                      # ROS2 节点类
from std_msgs.msg import String                  # 字符串消息类型
from autoparking_interface.srv import RegisterCar
from geometry_msgs.msg import Pose2D
from nav_2d_msgs.msg import Path2D
from rclpy.executors import MultiThreadedExecutor
from autoparking_interface.msg import ParkingPose
from autoparking_interface.srv import SetParking
import time
import math

class CarTestNode(Node):

    def __init__(self, name, pose, uwb_id):
        super().__init__(name)                                    # ROS2节点父类初始化
        # self.pose = Pose2D()
        self.car_name = name
        self.pose = pose
        self.current_x = pose.x                                      # 假设当前坐标
        self.current_y = pose.y
        # self.current_yaw = 3/2 * math.pi 
        self.current_yaw = pose.theta
        self.on_parking = False
        self.uwb_id = uwb_id
        self.parking_id = -2
        self.topic_name = f'/{self.car_name}/parking'

        self.client = self.create_client(RegisterCar, 'RegisterCar') # 创建服务客户端对象（服务接口类型，服务名）

        self.srv = self.create_service(SetParking, f'{self.car_name}/SetParking', self.adder_callback)  # 创建服务器对象（接口类型、服务名、服务器回调函数）
        # 3.创建发布者
        self.publisher_ = self.create_publisher(ParkingPose, self.topic_name, 10)                     # 创建发布者对象（消息类型、话题名、队列长度）
        self.publisher_timer = self.create_timer(0.05, self.publisher_timer_callback)          # 创建一个定时器（单位为秒的周期，定时执行的回调函数）  

        # 数据打印定时器    1s
        self.info_timer_ = self.create_timer(1, self.info_callback)

    def info_callback(self):
        # self.get_logger().info(f'当前位姿：{(self.current_x,self.current_y)}') 
        # self.get_logger().info(f'所选车位为{self.parking_id}') 
        self.get_logger().info(f'Current pose:{(self.current_x,self.current_y)}') 
        self.get_logger().info(f'The selected parking space is {self.parking_id}') 

    # 接受设置开始泊车的服务
    def adder_callback(self, request, response):   # 创建回调函数，执行收到请求后对数据的处理

        # 接收数据
        self.on_parking = request.on_parking                        # 设置泊车标志位 完成后需要重新设置为False
        self.parking_id_hope = request.parking_id_hope              # 接受预期车位id
        # self.get_logger().info(f'是否开始泊车：{self.on_parking}') 
        self.get_logger().info(f'Start parking:{self.on_parking}') 
        if self.on_parking == True:
            self.send_request()                                     # 向场端发送注册请求
            response.success = True
        else:
            response.success = False
        return response                          # 反馈应答信息
    
    def send_request(self):                                          # 创建一个发送服务请求的函数 向场端发送

        # 1.判断服务是否上线
        while self.client.wait_for_service(timeout_sec=1.0) is False:
            # self.get_logger().info(f'等待服务端上线....')
            self.get_logger().info(f'Waiting for the sever to go online....')
        # 2.构造 Request
        request = RegisterCar.Request()
        """
        1/车辆的名称；2/发送请求的时间辍/3；车辆的初始位姿态
        """
        request.car_name = self.car_name
        request.start_time = self.get_clock().now().to_msg()
        request.initial_pose = self.pose
        request.uwb_id = int(self.uwb_id)
        request.parking_id_hope = self.parking_id_hope
        
        # topic_name = f'/{request.car_name}/parking'                    # 发布的话题名称为 /车辆名称/parking
        # self.get_logger().info(f'topic_name:{topic_name}') 

        # 4.发送并 spin 等待服务处理完成
        future = self.client.call_async(request)
        future.add_done_callback(self.timer_callback_for_future)                    # 非阻塞地等待响应
        # rclpy.spin_until_future_complete(self, self.future)                       # 用这个会有点问题
        # self.timer_ = self.create_timer(0.01, self.timer_callback_for_future)     # 用定时器来查询，用这个也会有点问题

    # 发布当前车辆的二维位姿与车辆的名字
    def publisher_timer_callback(self):
        self.current_x = self.current_x
        self.current_y = self.current_y
        msg = ParkingPose()
        msg.car_name = self.car_name
        pose = Pose2D()
        pose.x = self.current_x
        pose.y = self.current_y
        pose.theta = self.current_yaw
        msg.current_pose = pose
        # self.get_logger().info(f'发布当前位姿{msg}')
        self.publisher_.publish(msg)  

    # 场端服务响应后的回调函数
    def timer_callback_for_future(self,future):

        #请求已经完成则处理，否则继续循环等待
        if future.done():
            try:
                #从响应结果中取响应数据
                response = future.result()
            except Exception as e:
                #如果发生异常，记录异常错误信息
                self.get_logger().error('Service call failed %r' % (e,))
            else:
                #正常响应
                self.car_id = response.car_id                           # 该车在停车场中的id
                # self.get_logger().info(f'该车在停车场中的id:{self.car_id}')
                self.get_logger().info(f'The ID of this vehicle in the parking lot:{self.car_id}')
                # self.get_logger().info(f'所规划好的泊车路径：{response.path}')              # 所规划好的泊车路径
                self.parking_id = response.parking_id
                # self.get_logger().info(f'所选择的车位为：{self.parking_id}')                # 所选择的车位为
                self.get_logger().info(f'The selected parking space is {self.parking_id}')                # 所选择的车位为

            #响应处理完毕，关闭定时器，与上面的开启定时器来查询组合
            # self.timer_.cancel()

def main(args=None):                                 # ROS2节点主入口main函数
    pose = Pose2D()
    # 26.6, 35.0
    # 14.23, 45.57
    pose.x = 13.10                                  # 初始化的位姿
    pose.y = 45.57                                    # 初始化的位姿
    pose.theta = 0.                                    # 初始化的位姿
    uwb_id = 1
    

    rclpy.init(args=args)                            # ROS2 Python接口初始化
    node = CarTestNode("car1",pose, uwb_id)     # 创建ROS2节点对象并进行初始化
    # node.send_request()                          # 发送服务请求
    executor = MultiThreadedExecutor()
    rclpy.spin(node,executor=executor)
    node.destroy_node()                          # 销毁节点对象
    rclpy.shutdown()                             # 关闭ROS2 Python接口

if __name__ == '__main__':
    main()