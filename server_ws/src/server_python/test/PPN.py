#!/usr/bin/env python3
# -*- coding: utf-8 -*-

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

import socket
import ast

from control import Car_Dynamics, MPC_Controller, Linear_MPC_Controller
import parkpath_scout
import path_theta
import numpy as np
import plath_test

# 创建一个udp套件字
import time

"""
创建一个发布者节点
"""

# 基站A0的偏置
offset_x = 13.559322
offset_y = 26.271186

def getxy():            # 每28ms 输出一个标签的测距数据
    # 等待接收方发送数据
    # rs中存储的是一个元组（接收到的数据，（发送方的ip，port））

    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # 绑定本地相关信息，如果不绑定，则系统会随机分配，必须绑定本电脑的ip和port
    local_addr = ('', 8080)  # 元组的第一个参数为本机IP，可以为空字符串，会自动生成本机IP
    udp_socket.bind(local_addr)

    rs_data = udp_socket.recvfrom(1024)
    rs_masg = rs_data[0]
    # rs_addr = rs_data[1]
    # print(rs_data)
    # 接收到的数据解码展示
    msg = rs_masg.decode('utf-8')
    # print(msg)
    # print(type(msg))
    # print("数据长度:",len(msg))

    x = ''
    y = ''
    z = ''

    j = 0

    for i in msg:           # 第40位：标签ID

        j = j+1
        if i == 'X':        # 第49位：X
            n = j + 3
            while msg[n] != ',':
                x = x + msg[n]
                n = n + 1

        if i == 'Y':        # 第49位：X
            n = j + 3
            while msg[n] != ',':
                y = y + msg[n]
                n = n + 1
        if i == 'Z':        # 第49位：X
            n = j + 3
            while msg[n] != ',':
                z = z + msg[n]
                n = n + 1

    # print(rs_addr)
    # 关闭套件字
    id = int(msg[40])
    x = ast.literal_eval(x)
    y = ast.literal_eval(y)
    z = ast.literal_eval(z)

    # print(f'标签ID为{id}')
    # print(f'X在消息的第{j}位,该位显示数据{i},X的坐标值位为{x}')
    # print(f'Y在消息的第{j}位,该位显示数据{i},Y的坐标值位为{y}')
    # print(f'Z在消息的第{j}位,该位显示数据{i},Z的坐标值位为{z}')

    udp_socket.close()

    x = x + offset_x
    y = y + offset_y
    return id, x, y, z

# 计算角度 0～2pi
def calculate_angle(x1, y1, x2, y2):
    delta_x = x2 - x1
    delta_y = y2 - y1
    angle_radians = math.atan2(delta_y, delta_x)
    angle_degrees = angle_radians * (180 / math.pi)
    # 如果需要将角度限制在0到360度之间（通常不是必需的）
    if angle_degrees < 0:
        angle_degrees += 360
    if angle_radians < 0:
        angle_radians += 2*math.pi
    return angle_radians

# 输入起始点 和 所选 path（岔路点和停车位点）， 输出 预期路径点
def getpathpoint(initial_x, initial_y, path):
    
    len_path = len(path)

    target_y = path[len_path -2].y+0.3      # 终点坐标只与 上一个路口的y轴 和 目标车位的x轴有关
    target_x = path[len_path -1].x-0.4

    # if len(path) == 2:                      # 如果路径长度为2,那么path[1]存储的就是停车位点，修改target的xy
    #     target_y = path[0].y+0.3
    #     target_x = path[1].x-0.4

    # 计算初始点与下一个目标的角度，如果目标在下方
    if 225.0 * 0.0174533 < calculate_angle(initial_x, initial_y, path[0].x, path[0].y) < 315.0 * 0.0174533:        
        point1 = [path[0].x, path[0].y + 2.5]
        point2 = [path[0].x + 1.1, path[0].y + 0.3]
    ######### 下面都没改好
    # # 目标在上方
    # if 225.0 * 0.0174533 < calculate_angle(initial_x, initial_y, path[0].x, path[0].y) < 315.0 * 0.0174533:        
    #     point1 = [path[0].x, path[0].y + 2.5]
    #     point2 = [path[0].x + 1.1, path[0].y + 0.3]
    # # 目标在 左
    # if 225.0 * 0.0174533 < calculate_angle(initial_x, initial_y, path[0].x, path[0].y) < 315.0 * 0.0174533:        
    #     point1 = [path[0].x, path[0].y + 2.5]
    #     point2 = [path[0].x + 1.1, path[0].y + 0.3]
    # # 目标在 右
    # if 225.0 * 0.0174533 < calculate_angle(initial_x, initial_y, path[0].x, path[0].y) < 315.0 * 0.0174533:        
    #     point1 = [path[0].x, path[0].y + 2.5]
    #     point2 = [path[0].x + 1.1, path[0].y + 0.3]

    # return [point1, point2, [target_x,target_y]]
    return [point1, point2, [target_x-2.53,target_y+0.1]]


# 用于发布测试
def posi():
    x = '14.75'
    y = '50.0'
    x = ast.literal_eval(x)
    y = ast.literal_eval(y)
    x = float(x)
    y = float(y)
    return x,y

# 自定义岔路点 + 停车位点 -》 并根据初始点产生新的路径点
def make_newpath(initial_x, initial_y):
    
    # 选岔路点 与 车位 点，把theta==1.0 作为是否为车位点，或者最后一个路径点就是车位
    pose1 = Pose2D()
    pose1.x = 15.50               # 14.80    offset_x = 13.559322
    pose1.y = 29.04                   # 29.04
    pose1.theta = 0.0               #

    pose2 = Pose2D()                # 车位
    pose2.x = 25.1                  # 24.4
    pose2.y = 35.0                  # 35
    pose2.theta = 1.0               #

    path = Path2D()
    # path.header = header
    path.poses = [pose1, pose2]  # You can add more poses to form a complete path

    newpath = getpathpoint(initial_x, initial_y,path.poses) 
    newpath = np.array(newpath)
    return newpath
    
class PositionPubNode(Node):

    def __init__(self, name, initial_x, initial_y, initial_yaw, path, my_car, controller, MPC_HORIZON, park_path):
        super().__init__(name)                                          # ROS2节点父类初始化
        self.initial_x = initial_x                                      # 初始 x 坐标
        self.initial_y = initial_y                                      # 初始 y 坐标
        self.current_x = initial_x                                      # 当前 x 坐标
        self.current_y = initial_y                                      # 当前 y 坐标
        self.initial_yaw = initial_yaw                                  # 初始航向角
        self.initial_imu = 0.0                                          # 初始IMU角
        self.current_yaw = 0.0                                          # 当前航行角   = 初始航向角 + （当前IMU角 - 初始IMU角）
        self.path = path                                                # 路径
        self.max_speed_ = 0.5                                           # 最大线速度
        self.max_yaw_ = 0.3027                                          # 最大角速度
        self.k_ = 1.0                                                   # 线性调节系数
        self.current_point_id = 0                                       # 当前路径的点的id
        self.current_parkpoint_id = 0                                   # 当前泊车路径的点的id
        self.target_x = path[0][0]                                      # 目标点坐标
        self.target_y = path[0][1]                                      # 目标点坐标
        self.initial_a_ = 0                                             # 0：记录初始IMU角度

        self.my_car_ = my_car
        self.controller_ = controller
        self.MPC_HORIZON = MPC_HORIZON
        self.park_path = park_path

        # 订阅 N100
        self.yaw_subscriber_ = self.create_subscription(Imu,'/imu',self.yaw_callback, 10)    # 创建订阅者对象（消息类型、话题名、回调函数、队列长度）
        
        # 发布二维位姿信息 100ms 并且更新二维坐标
        self.publisher_ = self.create_publisher(Pose2D, 'random_num', 10)   # 创建发布者对象（消息类型、话题名、队列长度）
        self.timer = self.create_timer(0.1, self.timer_callback)            # 创建一个定时器（单位为秒的周期，定时执行的回调函数）

        # # 创建发布路径点的发布者 包含选中的岔路口坐标列表，和最后 停车位的坐标（区分）
        # self.parkingplace_publisher_ = self.create_publisher(Path2D, 'path_topic', 10)           
        # self.timer = self.create_timer(3, self.pub_newpath)            # 创建一个定时器（单位为秒的周期，定时执行的回调函数）

        # 创建速度发布者 控制指令周期20ms
        self.vel_publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.vel_timer_ = self.create_timer(0.02, self.vel_publish)            # 创建一个定时器（单位为秒的周期，定时执行的回调函数）

        # # 测试，订阅乌龟的位置
        # self.pose_subscription_ = self.create_subscription(
        #     Pose,
        #     '/turtle1/pose',
        #     self.on_pose_received_,
        #     10
        # )
        self.get_logger().info(f'mypath:{self.path}')

    # 订阅N100发送的角度信息，并赋值给yaw 弧度制
    def yaw_callback(self, msg):
        # 读取四元数信息
        orientation = msg.orientation
        q_w = orientation.w
        q_x = orientation.x
        q_y = orientation.y
        q_z = orientation.z
        # 四元数转欧拉角
        yaw, _, _ = tf_transformations.euler_from_quaternion([q_w, q_x, q_y, q_z], axes='sxyz')  # 'sxyz'表示静态坐标系下的Z-Y-X旋转顺序

        # self.get_logger().info(
        #     f'Received IMU message:'
        #     f'euler: [{yaw}]'                   # -pi ~ pi
        # )

        if yaw < 0:                                 # 0 ~ 2pi
            yaw += 2*math.pi

        if self.initial_a_ == 0:            # 第一次读到角度信息
            # self.initial_yaw = 3/2*math.pi
            self.initial_a_ += 1
            self.current_yaw = self.initial_yaw
            self.initial_imu = yaw

        else:
            current_yaw = self.initial_yaw  - (yaw - self.initial_imu)      # 初始值加变化值，因为变化方向相反 用负号相加
            if current_yaw < 0:
                current_yaw += 2*math.pi
            if current_yaw > 3.1415926*2:
                current_yaw = current_yaw - 3.1415926*2
            self.current_yaw = current_yaw

        self.get_logger().info(
            f'Received IMU message:'
            f'euler: [{yaw}]'                   # 0~2pi
        )
    
        # self.current_yaw = msg.theta * 0.0174533        # 更新当前的yaw 弧度制
        # self.yaw = msg.theta

    # 接收UWB定位信息，yaw，并  发布二维位姿信息
    def timer_callback(self):
        id, x, y, z = getxy()
        # x, y = posi()                   # 测试用
        msg = Pose2D()
        msg.x = x
        msg.y = y
        msg.theta = self.current_yaw
        self.current_x = x              # 更新xy的坐标
        self.current_y = y
        self.publisher_.publish(msg)    
        # self.get_logger().info(f"publishing:{msg}")


    def vel_publish(self):

        message = Twist()

        if self.current_point_id >= len(self.path):
            # 执行垂直泊车路径跟踪
            self.get_logger().info(f'正在进行垂直泊车')
            linear, angular = self.controller_.optimize(self.my_car_, self.park_path[self.current_parkpoint_id:self.current_parkpoint_id + self.MPC_HORIZON])        # 差速
            # self.my_car_.update_state(self.my_car_.move(linear, angular))
            self.my_car_.update_state_pose(self.current_x, self.current_y, linear, self.current_yaw)
            message.linear.x = linear
            message.angular.z = angular
            self.current_parkpoint_id += 1

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

            message.linear.x = message.linear.x * 0.8
            message.angular.z = message.angular.z * 0.8

        else:
            self.get_logger().info(f'正在导航至车位')
            self.get_logger().info(f'当前位置:{self.current_x, self.current_y}')
            self.get_logger().info(f'下一个目标位置:{self.target_x, self.target_y}')
            
            distance = math.sqrt((self.target_x - self.current_x)**2 + (self.target_y - self.current_y)**2)
            # self.get_logger().info(f'距离差:{distance}')
            angle = calculate_angle(self.current_x, self.current_y, self.target_x, self.target_y) - self.current_yaw
            if angle > 3.1415926:
                angle -= 2*3.1415926
            if  angle < -3.1415926:
                angle += 2*3.1415926
            # self.get_logger().info(f'角度差:{calculate_angle(self.current_x, self.current_y, self.target_x, self.target_y)}')
            # self.get_logger().info(f'当前航向角为:{self.current_yaw}')

            if self.current_point_id % 2 == 0:          # 选择应该走直线的目标点 
                if distance > 0.3:                      # 距离比较大的时候
                # if distance > 0.4:                      # 试试target_y再+0.4
                    if math.fabs(angle) > 0.4:          # 先调航向角
                        message.angular.z = angle
                    else:                               # 再前进
                        message.linear.x = self.k_ * distance   # 距离越远速度越快
                else:
                    self.current_point_id += 1          # 更新至下一个目标点
                    if self.current_point_id >= len(self.path):
                        pass
                    else:
                        self.target_x = self.path[self.current_point_id][0]
                        self.target_y = self.path[self.current_point_id][1]


            else:                                       # 转弯口，走曲线
                if distance > 0.4: 
                    message.linear.x = 0.5
                    message.angular.z = 0.25
                else:
                    self.current_point_id += 1
                    self.target_x = self.path[self.current_point_id][0]
                    self.target_y = self.path[self.current_point_id][1]

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

            message.linear.x = message.linear.x * 0.7
            message.angular.z = message.angular.z * 0.7
        # message.linear.x = 0.0
        # message.angular.z = 0.314
        self.vel_publisher_.publish(message)
        self.get_logger().info(f'发布线速度:{message.linear.x}')
        self.get_logger().info(f'发布角速度:{message.angular.z}')

    # # 测试 订阅乌龟的pose
    # def on_pose_received_(self, msg: Pose):
    #     self.current_x = msg.x
    #     self.current_y = msg.y

    #     # if msg.theta < 0:
    #     #     self.current_yaw = msg.theta + 2*math.pi
    #     # else:
    #     #     self.current_yaw = msg.theta

def main(args=None):                                 # ROS2节点主入口main函数
    # 1.初始点获取
    _, initial_x, initial_y, _ = getxy()
    # initial_x , initial_y = posi()
    initial_yaw = 3/2*math.pi
    # initial_yaw = 0.0

    # 2.产生寻车路径
    path = make_newpath(initial_x , initial_y)
    # path = [[0., 0.]]

    # 3.产生跟踪路径
    parkstart_x = path[len(path)-1][0]
    parkstart_y = path[len(path)-1][1]
    park_path = parkpath_scout.parkpath(parkstart_x, parkstart_y)
    # park_path = parkpath_scout.down(initial_x, initial_y)       # 从当前点开始测试，初始角需要3/2*math.pi
    # park_path = parkpath_scout.parkpath(initial_x, initial_y)   # 从当前点开始测试，初始角需要0.0

    # list_t = []
    # for i in range(0,150,2):
    #     list_t.append((i,0))
    # list_t.reverse()
    # park_path = np.array(list_t)
    # park_path = park_path / 10.0
    # park_path = park_path + [initial_x,initial_y]

    
    # 4.路径跟踪参数配置
    my_car = Car_Dynamics(initial_x, initial_y, 0, np.deg2rad(90), length=1, dt=0.2)
    MPC_HORIZON = 5  # MPC区间
    controller = MPC_Controller()

    # 5.启动节点
    rclpy.init(args=args)                            # ROS2 Python接口初始化
    node = PositionPubNode("PositionPubNode", initial_x , initial_y, initial_yaw, path, my_car, controller, MPC_HORIZON, park_path)        # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                                 # 循环等待ROS2退出
    node.destroy_node()                              # 销毁节点对象
    rclpy.shutdown()                                 # 关闭ROS2 Python接口

if __name__ == '__main__':
    # main()

    # 1.初始点获取
    # _, initial_x, initial_y, _ = getxy()
    initial_x , initial_y = posi()
    initial_x = initial_x + 0.8
    print(initial_x)

    # 2.产生寻车路径
    path = make_newpath(initial_x , initial_y)

    # 3.产生泊车跟踪路径
    parkstart_x = path[len(path)-1][0]
    parkstart_y = path[len(path)-1][1]
    # park_path = parkpath_scout.parkpath(parkstart_x, parkstart_y)
    park_path = parkpath_scout.parkpath(initial_x, initial_y)

    # list_t = []
    # for i in range(0,150,5):
    #     list_t.append((i,0))
    # list_t.reverse()
    # park_path = np.array(list_t)
    # park_path = park_path / 10.0
    # park_path = park_path + [initial_x,initial_y]

    # 4.路径叠加测试
    vstack_arr = np.vstack((path, park_path))
    print(len(path))
    print(len(park_path))
    print(len(vstack_arr))
    print(path[len(path)-1],park_path[0])

    plath_test.show_polt(vstack_arr, 1)
