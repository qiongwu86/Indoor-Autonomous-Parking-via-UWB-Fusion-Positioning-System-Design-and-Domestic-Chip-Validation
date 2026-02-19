#!/usr/bin/env python3
import os
import time
import ctypes
import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from ament_index_python.packages import get_package_share_directory
import ament_index_python
from std_msgs.msg  import Float32MultiArray
import csv
from filter_dis import KalmanFilter
import numpy as np
from rclpy.executors import MultiThreadedExecutor

from moving_average import MovingAverage2D


"""
基于MK8000
"""
class UWBNode(Node):
    def __init__(self):
        super().__init__('uwb_node')

        # 串口初始化
        port = '/dev/ttyUSB0'
        baudrate = 115200
        timeout = 1
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        self.get_logger().info(f"Opened serial port: {port} @ {baudrate} baud")

        # 帧格式定义
        self.header = 0xf0
        self.tail = 0xaa
        self.payload_len = 5

        self.x = 0.
        self.y = 0.
        self.z = 0.

        self.x_minus_1 = 0.
        self.y_minus_1 = 0.
        self.z_minus_1 = 0.
        
        self.pose_list = []
        self.movingaverage = MovingAverage2D(window_size=5)
        self.average_on = True                  # 均值滤波开关 TODO:

        # publishers
        self.publisher  = self.create_publisher(Float32MultiArray,  '/uwb_data', 10)

        # trilateration SO
        so_path = os.path.join(
            get_package_share_directory('mk8000'),
            'mk8000', 'trilateration.so'
        )
        self.lib = ctypes.cdll.LoadLibrary("/home/www/test_ws/src/mk8000/mk8000/trilateration.so")

        # 定义 C 结构体
        class UWBMsg(ctypes.Structure):
            _fields_ = [
                ('x', ctypes.c_double),
                ('y', ctypes.c_double),
                ('z', ctypes.c_double),
            ]
        self.UWBMsg = UWBMsg

        # 初始化 anchor 与 distance 数组
        self.anchor_array = (UWBMsg * 8)()
        # 示例基站坐标（根据实际修改）TODO:
        self.anchor_array[0].x, self.anchor_array[0].y, self.anchor_array[0].z = 0.0, 0.0, 1.84
        self.anchor_array[1].x, self.anchor_array[1].y, self.anchor_array[1].z = 14.25, 0.0, 1.84
        self.anchor_array[2].x, self.anchor_array[2].y, self.anchor_array[2].z = 9.8, 13.73, 1.82
        # self.anchor_array[3].x, self.anchor_array[3].y, self.anchor_array[3].z = 0.0, 0.3, 2.0
        # 其余留空或按需填充

        
        # 无效的测距值一定给 -1，否则会用随机数进行运算
        # 毫米
        self.distance_array = (ctypes.c_int * 8)(*([-1] * 8))
        self.distance_array[0] = 0
        self.distance_array[1] = 0
        self.distance_array[2] = 0

        # 卡尔曼滤波器
        self.kf = KalmanFilter(max_anchors=3, max_tags=1, Q=0.018, R=0.542)

        # 定时器：20 Hz 读距离
        self.timer = self.create_timer(0.01, self.timer_callback)

        # 定时器：5 Hz 取定位
        self.getlocation_timer_ = self.create_timer(0.2, self.getlocation_callback)

        # 定时器：1 HZ 保存定位文件 测试用
        self.save_t = 0
        self.save_timer_ = self.create_timer(1, self.save_callback)

    def save_callback(self):
        t = 90                        # s
        print('self.save_t',self.save_t)
        if self.save_t == t:           
            self.savedata()
            self.save_t +=1
            self.pose_list = self.pose_list.tolist()
        elif self.save_t > t:
            pass
        else:
            self.save_t +=1

    def savedata(self):
        # 查找已知功能包的位置 
        package_name = 'mk8000'  # 替换为实际存在的功能包名称 
        package_path = ament_index_python.get_package_prefix(package_name) 
        # 假设功能包位于src目录下 
        src_path = os.path.dirname(os.path.dirname('/home/www/test_ws/src/mk8000/mk8000')) 
        print("src目录的完整路径:", src_path)
        # 构建目标目录的完整路径 
        self.directory = os.path.join(src_path,  'datalog')
        # 创建目录，如果目录不存在 
        os.makedirs(self.directory,  exist_ok=True)
        # 验证目录是否创建成功 
        if os.path.exists(self.directory): 
            print(f"'{self.directory}' 目录创建成功。")
        else:
            print(f"无法创建 '{self.directory}' 目录。")

        self.mk8000_pos_directory = os.path.join(self.directory,  'mk8000_pos.csv')
        self.pose_list = np.asarray(self.pose_list.copy())
        with open(self.mk8000_pos_directory, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y', 'z'])      # 写入表头
            writer.writerows(self.pose_list)         # 写入坐标行

    def read_frame(self):
        """从串口读一帧，返回 payload 列表或 None"""
        while True:
            hdr = self.ser.read(2)
            print('z')
            if not hdr:
                # 超时或无数据，继续等待
                print("Timeout waiting for header, retrying...")  # 可启用调试
                continue
                # return None
            print(f'hdr[0]:{hdr[0]}')
            
            if hdr[0] != self.header:           # 不是帧头，继续读取
                print('不是帧头')
                print(f'hdr[0]:{hdr[0]}')
                continue
            payload = self.ser.read(self.payload_len)       # 读取有效数据
            tail = self.ser.read(1)                         # 读取帧尾
            if len(payload)==self.payload_len and tail and tail[0]==self.tail:
                print('有效')
                return list(payload)
            # time.sleep(0.01)
            # 否则丢弃，继续

    @staticmethod
    def parse_payload(payload):
        lo, hi, dlo, dhi, sig = payload
        addr = (hi<<8) | lo
        dist = (dhi<<8) | dlo
        return addr, dist, sig
    
    # 读取距离数据 0.05s
    def timer_callback(self):
        payload = self.read_frame()
        if payload is None:
            print('未读到数据')
            return

        addr, dist_raw, sig = self.parse_payload(payload)
        dist_m = dist_raw / 100.0  # cm → m
        # 卡尔曼滤波
        filtered = self.kf.filter(dist_m, anc_idx=addr-1, tag_idx=0)
        # filtered = dist_m
        self.distance_array[addr-1] = int(filtered * 1000)  # m→mm

    # 计算定位数据 0.2s
    def getlocation_callback(self):
        # trilateration
        loc = self.UWBMsg()
        res = self.lib.GetLocation(
            ctypes.byref(loc),
            self.anchor_array,
            self.distance_array
        )
        
        self.get_logger().warn(f"GetLocation 返回值：{res}")

        # 发布
        msg_x = Float64()
        msg_y = Float64()
        msg_z = Float64()
        msg_x.data = loc.x
        msg_y.data = loc.y
        msg_z.data = loc.z

        if res == 3:
            msg = Float32MultiArray()
            # self.get_logger().info(f"Published:  {type(x)}")
            msg.data  = [0., msg_x.data, msg_y.data, msg_z.data]
            self.publisher.publish(msg) 
            
            self.x = msg.data[1]
            self.y = msg.data[2]
            self.z = msg.data[3]

            ####
            # 加入均值滤波
            if self.average_on == True:
                raw_pos = np.array([self.x,self.y])
                smooth_pos = self.movingaverage.update(raw_pos)
                self.x = smooth_pos[0]
                self.y = smooth_pos[1]
            ####
            self.x_minus_1 = self.x
            self.y_minus_1 = self.y
            self.z_minus_1 = self.z

            self.pose_list.append([self.x, self.y, self.z])             # 添加数据
            self.get_logger().info(f"Published:  {msg.data}") 

        else:
            msg = Float32MultiArray()
            # self.get_logger().info(f"Published:  {type(x)}")
            msg.data  = [0., self.x_minus_1, self.y_minus_1, self.z_minus_1]
            self.publisher.publish(msg) 
            
            self.x = msg.data[1]
            self.y = msg.data[2]
            self.z = msg.data[3]

            self.x_minus_1 = self.x
            self.y_minus_1 = self.y
            self.z_minus_1 = self.z
            self.get_logger().warn(f"error 返回上一坐标")
            self.get_logger().info(f"Published:  {msg.data}") 
            self.pose_list.append([0., 0., 0.])             # 添加数据

    def destroy_node(self):
        self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = UWBNode()
    try:
        # rclpy.spin(node,executor=executor)
        rclpy.spin(node)
        # executor.add_node(node) 
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
