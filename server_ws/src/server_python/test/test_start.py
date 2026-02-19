#!/usr/bin/env python3
 
"""
car1 用户节点，发送启动泊车指令的客户端，通过按键发送
"""
import rclpy
from rclpy.node import Node
 
import time
import numpy as np
from pynput import keyboard #引入键盘监听功能库

from autoparking_interface.srv import SetParking
from geometry_msgs.msg import Pose2D
from autoparking_interface.msg import ParkingPose

"""
目前只需要按键发送泊车指令
"""
class UserlNode(Node):
    
    def __init__(self,name):
        super().__init__(name)
        
        self.car_name = "car1"
        self.client = self.create_client(SetParking, f'{self.car_name}/SetParking') # 创建服务客户端对象（服务接口类型，服务名）
        
        #创建键盘事件监听器，并启动
        self.listener = keyboard.Listener(on_press=self.on_press,on_release=self.on_release)
        self.listener.start()                       # 对应listener.stop()
        self.p_press = False
        self.on_parking = False
        
    # 发送服务请求
    def send_request(self):                                          # 创建一个发送服务请求的函数

        # 1.判断服务是否上线
        while self.client.wait_for_service(timeout_sec=1.0) is False:
            self.get_logger().info(f'等待服务端上线....')
        # 2.构造 Request
        request = SetParking.Request()
        """
        1/on_parking是否泊车;2/success是否发送成功
        """
        request.on_parking = True
        # 4.发送并 spin 等待服务处理完成
        future = self.client.call_async(request)
        future.add_done_callback(self.timer_callback_for_future)                    # 非阻塞地等待响应
    
    # 服务响应后的回调函数
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
                self.get_logger().info(f'是否成功发送:{response.success }')
                self.on_parking = True
                self.get_logger().info(f'成功启动泊车')
                # 创建订阅者
                topic_name = f'/{self.car_name}/parking'             # 设置订阅者的话题名字
                self.subscription = self.create_subscription(ParkingPose,topic_name, self.subscriber_callback,1)
    
        # 订阅车俩的位置信息
    def subscriber_callback(self,msg):
        car_name = msg.car_name

        self.get_logger().info(f'车辆{car_name}的位置为{msg.current_pose.x,msg.current_pose.y}') 
      
    #键盘按键按下事件处理
    def on_press(self, key):
        #判断是否是方向键
        if key == keyboard.KeyCode.from_char('p'):
            if not self.p_press:                        # 只有在第一次按下p时才计时
                self.start_time_p = time.time()
            self.p_press = True                         # p键被按下标志
        # elif isinstance(key, keyboard._xorg.KeyCode):
        #     if key.char == 'w':
        #         print(f'按下')
        else:
            print(f'未设置该按键功能.请长按p键启动泊车')

     #键盘按键松开事件处理
    def on_release(self, key):
        if key == keyboard.KeyCode.from_char('p'):
            self.p_press = False
            self.current_time_p = time.time() 
            self.elapsed_time_p = self.current_time_p - self.start_time_p
            self.get_logger().info(f'按下了p键{self.elapsed_time_p}秒')
            if self.elapsed_time_p > 1.0:                   # p键盘按了一秒
                # 发送泊车指令
                self.listener.stop()                        # 并关闭键盘监听
                self.send_request()

def main(args=None):
    rclpy.init(args=args)
    node = UserlNode(name="user_node")
    rclpy.spin(node=node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()