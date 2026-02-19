#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
car1 注册用的服务 客户端 ，请求注册车辆信息,并创建发布者
并完成泊车控制

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
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import socket
import ast
from std_msgs.msg  import Float32MultiArray
from .control import Car_Dynamics, MPC_Controller, Linear_MPC_Controller
from .utils import *
import numpy as np

################
from .my_ekf import *
################

# 创建一个udp套件字
import time

# # 基站A0的偏置 小
# offset_x = 13.559322
# offset_y = 26.271186

use = 0         # 0:DWM1000             1:MK8000
if use ==0:
    # da
    offset_x = 13.559322
    offset_y = 10.153
else:
    offset_x = 13.559322
    offset_y = 26.271186

# 获取UDP定位数据
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

# Path2D to path
def to_path(path_2d:Path2D):
    path = []
    for point in path_2d.poses:
        x = point.x
        y = point.y
        path.append((x,y))
    path = np.array(path)
    return path

class CarNode(Node):

    def __init__(self, name, initial_x, initial_y, initial_yaw, my_car, controller, uwb_id):
        super().__init__(name)                                          # ROS2节点父类初始化
        self.car_name = name                                            # 车辆的名称
        self.initial_x = initial_x                                      # 初始 x 坐标
        self.initial_y = initial_y                                      # 初始 y 坐标
        self.current_x = initial_x                                      # 当前 x 坐标
        self.current_y = initial_y                                      # 当前 y 坐标
        self.initial_yaw = initial_yaw                                  # 初始航向角
        self.initial_imu = 0.0                                          # 初始IMU角
        self.current_yaw = 0.0                                          # 当前航行角   = 初始航向角 + （当前IMU角 - 初始IMU角）
        self.max_speed_ = 0.5                                           # 最大线速度
        self.max_yaw_ = 0.3027                                          # 最大角速度
        self.k_ = 1.0                                                   # 线性调节系数
        self.current_point_id = 0                                       # 当前路径的点的id
        self.current_parkpoint_id = 0                                   # 当前泊车路径的点的id
        self.initial_a_ = 0                                             # 0：记录初始IMU角度
        self.on_parking = False                                         # 是否在泊车的标志位
        self.my_car_ = my_car                                           # 创建车辆运动学模型
        self.controller_ = controller                                   # 配置控制器
        self.MPC_HORIZON = 5                                            # 配置预测空间
        self.topic_name = f'/{self.car_name}/parking'                   # 定义发布的话题名称 /车辆名称/parking
        self.logger = DataLogger()                                      # 日志管理
        self.uwb_id = uwb_id                                            # uwb编号
        self.linear = 0.0                                               # 当前车辆的线速度
        self.angular = 0.0                                              # 当前车辆的角速度      
        self.info_id = 0                                                # 用于延长打印间隔
        
        self.mpc_step = 1
        self.linaer_scale = 0.90                                        # 0.95 -》0.90
        self.angular_scale = 0.95                                        # 1.0->0.95
        self.ekf_use = True                                            # TODO:测试MK8000时要改     # 不使用的情况下 可以后续用其他滤波算法进行对比
        
        self.odom_x = 0.
        self.odom_y = 0.
        self.uwb_x = 0.
        self.uwb_y = 0.

        self.offset_x = 13.559322
        self.use = 0         # 0:DWM1000             1:MK8000
        if self.use == 0:
            self.offset_y = 10.153
        else:
            self.offset_y = 26.271186               # MK8000 小 # TODO:

        ################
        # 增加ekf
        # 初始化滤波器
        self.current_ekf_x = initial_x
        self.current_ekf_y = initial_y
        self.current_ekf_yaw = initial_yaw    
        self.ekf = ExtendedKalmanFilter(
            process_noise=np.array([0.00,0.00,0.000]),  # 运动噪声
            sensor_noise=np.array([0.00,0.00,0.00]),    # 传感器噪声
            Q_scale=1.0,                                # 模型协方差
            R_scale=1.0,                                 # 传感器协方差TODO:调整参数   5;3
            x=initial_x,y=initial_y,yaw=initial_yaw
        )
        self.linear_k_minus_1 = 0.0
        self.angular_k_minus_1 = 0.0

        self.uwb_t = 0.
        self.uwb_t_minus_1 = 0.

        self.nlos = True               # NLOS标志位
        self.last_control = [0.,0.]              # 记录上一时刻的控制参数
        ################

        self.current_x_0 = initial_x                                      # 当前 x 坐标
        self.current_y_0 = initial_y                                      # 当前 y 坐标
        self.current_x_1 = initial_x                                      # 当前 x 坐标
        self.current_y_1 = initial_y                                      # 当前 y 坐标
        self.current_x_2 = initial_x                                      # 当前 x 坐标
        self.current_y_2 = initial_y                                      # 当前 y 坐标        
        self.current_x_3 = initial_x                                      # 当前 x 坐标
        self.current_y_3 = initial_y                                      # 当前 y 坐标

        ################
        # 订阅 N100
        self.yaw_subscriber_ = self.create_subscription(Imu,'/imu',self.yaw_callback, 10)    # 创建订阅者对象（消息类型、话题名、回调函数、队列长度）
        
        # 订阅 UWB
        self.uwb_subscriber_ = self.create_subscription(Float32MultiArray,'/uwb_data',self.uwb_callback, 10)    # 创建订阅者对象（消息类型、话题名、回调函数、队列长度）
        # 150ms 四标签融合
        # self.uwbs_timer = self.create_timer(0.15, self.uwbs_callback)

        # 创建注册服务的客户端
        self.client = self.create_client(RegisterCar, 'RegisterCar') # 创建服务客户端对象（服务接口类型，服务名）

        # 创建开始泊车服务的服务端
        self.srv = self.create_service(SetParking, f'{self.car_name}/SetParking', self.adder_callback)  # 创建服务器对象（接口类型、服务名、服务器回调函数）
        
        # 发布二维位姿信息 50ms 同时接受UWB信息，并更新二维坐标
        self.publisher_ = self.create_publisher(ParkingPose, self.topic_name, 10)   # 创建发布者对象（消息类型、话题名、队列长度）
        self.pose_timer = self.create_timer(0.05, self.timer_callback)            # 创建一个定时器（单位为秒的周期，定时执行的回调函数）
        
        # 创建速度发布者 控制指令周期50ms
        self.vel_publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.vel_timer_ = self.create_timer(0.05, self.vel_publish)            # 创建一个定时器（单位为秒的周期，定时执行的回调函数）
        # MPC更新周期200ms
        self.mpc_timer = self.create_timer(0.2, self.vel_change)            # 创建一个定时器（单位为秒的周期，定时执行的回调函数）
        # 改成400ms TODO:
        # self.mpc_timer = self.create_timer(0.4, self.vel_change)            # 创建一个定时器（单位为秒的周期，定时执行的回调函数）

        # 订阅odom坐标并更新
        self.odom_sub_ = self.create_subscription(ParkingPose,'/car1/my_odom',self.odom_callback, 10)
    
    def odom_callback(self,msg):
        # 使用odom              # TODO:如果修改初始角度了，需要修改odom中的旋转角度
        points = Pose2D()
        points.x = msg.current_pose.x
        points.y = msg.current_pose.y

        self.odom_x = points.x + self.initial_x
        self.odom_y = points.y + self.initial_y

    # 150ms 四标签融合
    def uwbs_callback(self):
        
        # 假设四个标签的二维坐标
        coords = np.array([
            [self.current_x_0, self.current_y_0],
            [self.current_x_1, self.current_y_1],
            [self.current_x_2, self.current_y_2],
            [self.current_x_3, self.current_y_3]
        ])

        # 核心逻辑判断 
        if np.all(np.abs(coords)  > 1e-4 ):
            
            # 计算质心
            fused_point = coords.mean(axis=0)
            self.current_x = fused_point[0]
            self.current_y = fused_point[1]
            self.get_logger().info(f"融合后坐标：({self.current_x}, {self.current_y})")

        else:
            self.get_logger().info("非四标签融合状态")

    def uwb_callback(self, msg):
        # self.uwb_t = time.perf_counter()
        # delta_tt = self.uwb_t - self.uwb_t_minus_1
        # self.get_logger().info(f'uwb更新时间:{delta_tt}s')

        # self.uwb_t_minus_1 = self.uwb_t
        id = int(msg.data[0])
        if id == 0:
            # 更新x，y位置信息
            self.current_x = msg.data[1]+self.offset_x
            self.current_y = msg.data[2]+self.offset_y
            self.uwb_x = self.current_x                 # UWB坐标
            self.uwb_y = self.current_y

        # self.get_logger().info(f"receive:{id,msg.data[1]}") 
        ################
        # # ekf滤波
        # control_inputs = np.array([self.linear_k_minus_1,  self.angular_k_minus_1])     # 取上一时刻的控制量
        # self.linear_k_minus_1 = self.linear
        # self.angular_k_minus_1 = self.angular
        # model_estimate_history = self.ekf.predict(control_inputs,  delta_t=delta_tt)         # 由运动模型计算出的状态    # TODO:修改delta_t为uwb定位间隔
        # z = np.asarray([self.current_x,self.current_y,self.current_yaw])    # 输入传感器的观测值
        # state_estimate_ekf = self.ekf.update(z)                                         # 由ekf融合后的状态
        # self.current_ekf_x = state_estimate_ekf[0]
        # self.current_ekf_y = state_estimate_ekf[1]
        # self.current_ekf_yaw = state_estimate_ekf[2]
        
        ################################
        # # TODO:多标签
        
        # self.uwb_t = time.perf_counter()
        # delta_t = self.uwb_t - self.uwb_t_minus_1
        # self.get_logger().info(f'uwb更新时间:{delta_t}s')

        # self.uwb_t_minus_1 = self.uwb_t

        # id = int(msg.data[0])
        # self.get_logger().info(f"id:({id})")
        # if id == 0:
        #     self.get_logger().info(f"执行了id0")
        #     self.current_x_0 = msg.data[1]+offset_x
        #     self.current_y_0 = msg.data[2]+offset_y
        # elif id == 1:
        #     self.get_logger().info(f"执行了id1")
        #     self.current_x_1 = msg.data[1]+offset_x
        #     self.current_y_1 = msg.data[2]+offset_y
        # elif id == 2:
        #     self.get_logger().info(f"执行了id2")
        #     self.current_x_2 = msg.data[1]+offset_x
        #     self.current_y_2 = msg.data[2]+offset_y
        # elif id == 3:
        #     self.get_logger().info(f"执行了id3")
        #     self.current_x_3 = msg.data[1]+offset_x
        #     self.current_y_3 = msg.data[2]+offset_y

        # # 假设四个标签的二维坐标
        # coords = np.array([
        #     [self.current_x_0, self.current_y_0],
        #     [self.current_x_1, self.current_y_1],
        #     [self.current_x_2, self.current_y_2],
        #     [self.current_x_3, self.current_y_3]
        # ])

        # # 计算质心
        # fused_point = coords.mean(axis=0)
        # self.current_x = fused_point[0]
        # self.current_y = fused_point[1]
        # self.get_logger().info(f"融合后坐标：({self.current_x}, {self.current_y})")
        ################################

    # 接受设置开始泊车的服务
    def adder_callback(self, request, response):   # 创建回调函数，执行收到请求后对数据的处理

        # 接收数据
        if request.on_parking == True:                              # 泊车
            self.parking_id_hope = request.parking_id_hope          # 接受预期车位id
            self.send_request()                                     # 发送注册请求，完成请求后再开始泊车
            response.success = True                                 # 设置成功启动泊车
        else:
            response.success = False
        return response 
    
    # 发送注册请求
    def send_request(self):                                          # 创建一个发送服务请求的函数

        # 1.判断服务是否上线
        while self.client.wait_for_service(timeout_sec=1.0) is False:
            self.get_logger().info(f'等待服务端上线....')
        # 2.构造 Request
        request = RegisterCar.Request()
        """
        1/车辆的名称;2/发送请求的时间辍/3;请求指令时车辆的初始位姿态
        """
        request.car_name = self.car_name
        request.start_time = self.get_clock().now().to_msg()
        request.initial_pose.x = self.current_x
        request.initial_pose.y = self.current_y
        request.initial_pose.theta = self.current_yaw
        request.uwb_id = int(self.uwb_id)
        request.parking_id_hope = self.parking_id_hope

        # # 3.创建发布者
        # # 发布二维位姿信息 100ms 同时接受UWB信息，并更新二维坐标
        # self.publisher_ = self.create_publisher(ParkingPose, self.topic_name, 10)   # 创建发布者对象（消息类型、话题名、队列长度）
        # self.timer = self.create_timer(0.1, self.timer_callback)            # 创建一个定时器（单位为秒的周期，定时执行的回调函数）

        # 4.发送并 spin 等待服务处理完成
        future = self.client.call_async(request)
        future.add_done_callback(self.timer_callback_for_future)                    # 非阻塞地等待响应

    # 注册服务响应后的回调函数
    def timer_callback_for_future(self,future):

        #请求已经完成则处理，否则继续循环等待
        if future.done():
            try:
                #从响应结果中取响应数据
                response = future.result()
            except Exception as e:
                #如果发生异常，记录异常错误信息
                self.get_logger().info('Service call failed %r' % (e,))
            else:
                #正常响应
                self.car_id = response.car_id                           # 该车在停车场中的id
                self.get_logger().info(f'该车在停车场中的id:{self.car_id}')
                self.parking_id = response.parking_id
                self.get_logger().info(f'所选择的车位为：{self.parking_id}')                # 所选择的车位为
                self.command = response.command
                self.get_logger().info(f'收到指令{str(self.command)}')
                # self.logger.save_command(self.command)
                
                # 改变路径产生的方式    
                parh_2d = response.path
                park_path_2d = response.park_path
                self.path = to_path(parh_2d)
                self.park_path= to_path(park_path_2d)
                self.target_x = self.path[0][0]                                      # 初始化寻车路径的第一个目标点坐标
                self.target_y = self.path[0][1]                                      # 初始化寻车路径的第一个目标点坐标
                self.on_parking = True                                  # 开始泊车
                self.path_plannig = np.vstack((self.path, self.park_path))                             # 整合参考路径

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

        if yaw < 0:                                 # 0 ~ 2pi
            yaw += 2*math.pi

        if self.initial_a_ == 0:            # 第一次读到角度信息                # 可能需要线程优化一下，初始角才会准
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

        # self.get_logger().info(
        #     f'Received IMU message:'
        #     f'euler: [{yaw}]'                   # 0~2pi
        # )
    
        # self.current_yaw = msg.theta * 0.0174533        # 更新当前的yaw 弧度制
        # self.yaw = msg.theta

    # 接收UWB定位信息，yaw，并  发布二维位姿信息
    def timer_callback(self):
        # start = time.perf_counter()
        # id, x, y, z = getxy()
        # end = time.perf_counter()
        # self.get_logger().info('uwb用时' + str(1000 * (end - start)) + 'ms')   
        
        # (id, x, y, z) = (0, 15.23, 15.57 , 1.0)
        # self.uwb_id = id
        msg = ParkingPose()
        msg.car_name = self.car_name
        pose = Pose2D()
        pose.x = self.current_x
        pose.y = self.current_y
        pose.theta = self.current_yaw
        msg.current_pose = pose
        self.publisher_.publish(msg)    
        # self.get_logger().info(f"publishing:{msg}")
    
    # 控制
    def vel_change(self):
        # TODO:  ekf
        ekf_start = time.perf_counter()
        # ekf滤波
        control_inputs = np.array([self.linear_k_minus_1,  self.angular_k_minus_1])     # 取上一时刻的控制量
        self.linear_k_minus_1 = self.linear
        self.angular_k_minus_1 = self.angular
        model_estimate_history = self.ekf.predict(control_inputs,  delta_t=0.2)         # 由运动模型计算出的状态
        z = np.asarray([self.current_x,self.current_y,self.current_yaw])    # 输入传感器的观测值
        state_estimate_ekf = self.ekf.update(z)                                         # 由ekf融合后的状态
        self.current_ekf_x = state_estimate_ekf[0]
        self.current_ekf_y = state_estimate_ekf[1]
        self.current_ekf_yaw = state_estimate_ekf[2]

        self.nlos = self.ekf.nlos

        if self.nlos :
            self.controller_.change_Rp()     # 改变
            # self.controller_.recover_Rp()    # 恢复

        else:
            self.controller_.recover_Rp()    # 恢复

        # # 是否使用ekf滤波定位：
        if self.ekf_use == True:
            self.current_x = self.current_ekf_x
            self.current_y = self.current_ekf_y 

        ekf_end = time.perf_counter()
        # self.get_logger().info('ekf用时' + str(1000 * (ekf_end - ekf_start)) + 'ms') 

        if self.on_parking == False:
            self.info_id +=1
            if self.info_id == 10:                       # 每十次打印一次
                self.info_id = 0
                self.get_logger().info(f'不在泊车中')
        else:   # 自动泊车控制                          # 允许泊车的时候发布控制指令
            message = Twist()
            #############################################################################################################
            if self.current_point_id >= len(self.path):         # 当前路径点超过了寻车路径点后，开始进行轨迹跟踪垂直泊车
                # 执行垂直泊车路径跟踪
                self.info_id +=1
                if self.info_id == 10:                       # 每十次打印一次
                    self.info_id = 0
                    self.get_logger().info(f'正在进行垂直泊车')
                    self.get_logger().info(f'当前位姿:{self.current_x,self.current_y,self.current_yaw}')
                    self.get_logger().info(f'当前线速度:{self.linear}')
                    self.get_logger().info(f'当前角速度:{self.angular}')

                if self.current_parkpoint_id >= len(self.park_path)-1:       # 如果路径跟踪点结束了，
                    self.on_parking = False                                                     # 泊车 置0
                    self.logger.save_data(self.path_plannig,self.command)                       # 保存数据
                    self.logger.save_pos()
                    self.logger.save_four_pos()
                    self.linear = 0.0
                    self.angular = 0.0
                    return self.get_logger().info(f'泊车已经结束')
                
                v_last, w_last = self.last_control
                mpc_start = time.perf_counter()
                # linear, angular = self.controller_.optimize(self.my_car_, self.park_path[self.current_parkpoint_id:self.current_parkpoint_id + self.MPC_HORIZON])        # 差速
                
                # TODO: 步长改为2 对应400ms
                linear, angular = self.controller_.optimize(self.my_car_, self.park_path[self.current_parkpoint_id:self.current_parkpoint_id + self.MPC_HORIZON:self.mpc_step], u_prev=np.array([v_last, w_last]))

                mpc_end = time.perf_counter()
                dt = int(1000 * (mpc_end - mpc_start))/100.0
                self.get_logger().info('mpc用时' + str(1000 * (mpc_end - mpc_start)) + 'ms')
                if dt < 0.2:
                    dt = 0.2        # TODO:        
                # if dt < 0.4:
                    # dt = 0.4       
                # self.my_car_.update_state(self.my_car_.move(linear, angular))
                self.my_car_.update_state_pose(self.current_x, self.current_y, linear, self.current_yaw,dt)
                message.linear.x = linear
                message.angular.z = angular
                self.current_parkpoint_id += self.mpc_step

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

                message.linear.x = message.linear.x * self.linaer_scale
                message.angular.z = message.angular.z * self.angular_scale

                self.last_control = [message.linear.x, message.angular.z]

            else:
                self.info_id +=1
                if self.info_id == 10:                       # 每十次打印一次
                    self.info_id = 0
                    self.get_logger().info(f'正在导航至车位')
                    self.get_logger().info(f'当前位姿:{self.current_x,self.current_y,self.current_yaw}')
                    self.get_logger().info(f'当前线速度:{self.linear}')
                    self.get_logger().info(f'当前角速度:{self.angular}')

                v_last, w_last = self.last_control
                mpc_start = time.perf_counter()
                # linear, angular = self.controller_.optimize(self.my_car_, self.path[self.current_point_id:self.current_point_id + self.MPC_HORIZON])        # 差速
                self.get_logger().info(f'目标点{self.path[self.current_point_id:self.current_point_id + self.MPC_HORIZON]}')
                
                # TODO: 步长改为2 对应400ms
                linear, angular = self.controller_.optimize(self.my_car_, self.path[self.current_point_id:self.current_point_id + self.MPC_HORIZON:self.mpc_step], u_prev=np.array([v_last, w_last]))

                # self.my_car_.update_state(self.my_car_.move(linear, angular))
                mpc_end = time.perf_counter()
                dt = int(1000 * (mpc_end - mpc_start))/100.0
                self.get_logger().info('mpc用时' + str(1000 * (mpc_end - mpc_start)) + 'ms') 
                if dt < 0.2:
                    dt = 0.2  # TODO:
                # if dt < 0.4:
                    # dt = 0.4       
                self.my_car_.update_state_pose(self.current_x, self.current_y, linear, self.current_yaw,dt)

                message.linear.x = linear
                message.angular.z = angular
                self.current_point_id += self.mpc_step

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

                message.linear.x = message.linear.x * self.linaer_scale
                message.angular.z = message.angular.z * self.angular_scale

                self.last_control = [message.linear.x, message.angular.z]
            # 记录车辆运动的数据：当前(x，y)，yaw，linear，angular
            ############增加ekf数据############
            self.logger.log((self.current_x, self.current_y), self.current_yaw, message.linear.x, message.angular.z, (self.current_ekf_x,self.current_ekf_y))
            self.logger.log_four_tags([self.current_x_0,self.current_y_0],
                                      [self.current_x_1,self.current_y_1],
                                      [self.current_x_2,self.current_y_2],
                                      [self.current_x_3,self.current_y_3])
            self.logger.log_odom([self.odom_x,self.odom_y])
            self.logger.log_uwb([self.uwb_x,self.uwb_y])
            # 修改当前线速度和角速度
            self.linear=message.linear.x
            self.angular=message.angular.z
            # self.vel_publisher_.publish(message)
            # self.get_logger().info(f'当前位姿:{self.current_x,self.current_y,self.current_yaw}')
            # self.get_logger().info(f'发布线速度:{message.linear.x}')
            # self.get_logger().info(f'发布角速度:{message.angular.z}')

    # 速度控制发布指令
    def vel_publish(self):                      
        message = Twist()
        message.linear.x=self.linear
        message.angular.z=self.angular
        self.vel_publisher_.publish(message)
        # self.get_logger().info(f'发布线速度:{message.linear.x}')
        # self.get_logger().info(f'发布角速度:{message.angular.z}')

def main(args=None):                                 # ROS2节点主入口main函数
    (uwb_id, initial_x, initial_y, z) = (0, 14.23, 45.57 , 0.0)             # 停车场入口
    # (uwb_id, initial_x, initial_y, z) = (0, 24.859, 45.653, 0.0)             # ei
    # (uwb_id, initial_x, initial_y, z) = (0, 15.3, 38.42 , 0.0)             # TODO: mk8000
    # 1.初始点获取
    # uwb_id, initial_x, initial_y, _ = getxy()
    # initial_yaw = 3/2*math.pi                 # TODO: mk8000 小地图调整初始角度
    # initial_yaw = math.pi
    initial_yaw = 0.0                     
    # print(initial_x,initial_y)

    # 2.路径跟踪参数配置
    my_car = Car_Dynamics(initial_x, initial_y, 0, initial_yaw, length=1, dt=0.2) # TODO:
    # my_car = Car_Dynamics(initial_x, initial_y, 0, initial_yaw, length=1, dt=0.4) # TODO:
    controller = MPC_Controller()                           

    # 3.启动节点
    rclpy.init(args=args)                            # ROS2 Python接口初始化
    executor = MultiThreadedExecutor()
    node = CarNode("car1", initial_x , initial_y, initial_yaw, my_car, controller, uwb_id)        # 创建ROS2节点对象并进行初始化
    rclpy.spin(node,executor=executor)                                 # 循环等待ROS2退出
    node.destroy_node()                              # 销毁节点对象
    rclpy.shutdown()                                 # 关闭ROS2 Python接口

if __name__ == '__main__':
    main()


