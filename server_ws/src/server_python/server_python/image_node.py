import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray

import cv2 
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import time
import random

# 根据可泊车位id来绘制
def plt_lot(free_lot_id):
    # 车位坐标(像素坐标)
    cars = {1: [389, 55], 2: [431, 55], 3: [473, 55], 4: [532, 55], 5: [574, 55], 6: [616, 55], 7: [676, 55], 8: [718, 55], 9: [759, 55], 
            10: [331, 253], 11: [389, 253], 12: [431, 253], 13: [473, 253], 14: [533, 253], 15: [575, 253], 16: [616, 253], 17: [676, 253], 18: [718, 253], 19: [760, 253], 
            20: [331, 346], 21: [389, 346], 22: [431, 346], 23: [473, 346], 24: [532, 346], 25: [574, 346], 26: [616, 346], 27: [676, 346], 28: [718, 346], 29: [759, 346], 
            30: [331, 540], 31: [389, 541], 32: [431, 541], 33: [473, 541], 34: [533, 541], 35: [575, 541], 36: [616, 541], 37: [676, 541], 38: [718, 541], 39: [760, 541], 
            40: [331, 633], 41: [389, 634], 42: [431, 634], 43: [473, 634], 44: [532, 634], 45: [574, 634], 46: [616, 634], 47: [676, 634], 48: [718, 634], 49: [759, 634], 
            50: [318, 831], 51: [389, 831], 52: [431, 831], 53: [473, 831], 54: [533, 831], 55: [575, 831], 56: [616, 831], 57: [676, 831], 58: [718, 831], 59: [760, 831]}

    # 读取图像 
    image = cv2.imread('/home/www/auto_parking/server_ws/src/server_python/map/MAP.png') 
    
    # 将非方框区域转为灰度  # 取消
    gray_background = cv2.cvtColor(image,  cv2.COLOR_BGR2GRAY)
    gray_background = cv2.cvtColor(gray_background,  cv2.COLOR_GRAY2BGR)
    
    hsv = cv2.cvtColor(image,  cv2.COLOR_BGR2HSV)
    
    # 设定蓝色HSV范围 
    lower_blue = np.array([100,  100, 50])
    upper_blue = np.array([140,  255, 255])
    
    # 生成蓝色掩码 
    mask = cv2.inRange(hsv,  lower_blue, upper_blue)
    
    # 查找轮廓 
    contours, _ = cv2.findContours(mask,  cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # 创建拷贝用于绘制 
    output = image.copy()       # 取消了灰度图，如要其余车位灰度图像 使用output = gray_background.copy()
    
    # 颜色定义 
    PARKING_AVAILABLE_COLOR = (0, 180, 0)  # 深绿色
    PARKING_UNAVAILABLE_COLOR = (0, 0, 200)  # 深红色 
    BORDER_COLOR = (128, 128, 128)  # 灰色边框 
    FILL_COLOR_AVAILABLE = (200, 255, 200)  # 浅绿填充
    FILL_COLOR_UNAVAILABLE = (220, 220, 255)  # 浅红填充

    # 首先对所有车位进行排序（从左到右，从上到下）
    # 使用中心点坐标作为排序依据 
    sorted_contours = sorted(contours, key=lambda c: (cv2.boundingRect(c)[1],  cv2.boundingRect(c)[0])) 

    # 处理方框 
    for cnt in sorted_contours:
        x, y, w, h = cv2.boundingRect(cnt)      # _,_,38,90
        center_x = x + w // 2
        center_y = y + h // 2 

        # 判断是否在指定区域
        in_special_area = center_x < 300 or center_x > 780 
        is_available = not in_special_area  # 指定区域为不可用 
    
        # 方框填充
        fill_color = (255,255,255) if is_available else FILL_COLOR_UNAVAILABLE 
        output[y:y+h, x:x+w] = fill_color
    
        # 绘制虚线边框 
        step = 10 
        for i in range(x, x + w, step):
            cv2.line(output,  (i, y), (i + 5, y), BORDER_COLOR, 1)
            cv2.line(output,  (i, y + h), (i + 5, y + h), BORDER_COLOR, 1)
        for j in range(y, y + h, step):
            cv2.line(output,  (x, j), (x, j + 5), BORDER_COLOR, 1)
            cv2.line(output,  (x + w, j), (x + w, j + 5), BORDER_COLOR, 1)
        if not is_available:
            # 绘制斜线
            cv2.line(output,  (x+5, y+5), (x+w-5, y+h-5), (0, 0, 200), 1)
            cv2.line(output,  (x+w-5, y+5), (x+5, y+h-5), (0, 0, 200), 1)

    for spot_id, (center_x, center_y) in cars.items(): 
        # 检查是否是可泊车位 
        is_available = spot_id in free_lot_id

        # 绘制标记 
        font = cv2.FONT_HERSHEY_DUPLEX
        font_scale = min(38, 90) / 60
        x = center_x - 19
        y = center_y - 45
        w = 38
        h = 90

        # 如果是可泊车位，绘制绿色"P"
        if is_available:
            output[center_y-45:center_y+45, center_x-19:center_x+19] = FILL_COLOR_AVAILABLE

            # 绿色"P"
            text_size = cv2.getTextSize("P",  font, font_scale, 2)[0]
            cv2.putText(output,  "P", 
                        (center_x - text_size[0]//2, center_y + text_size[1]//2),
                        font, font_scale, PARKING_AVAILABLE_COLOR, 2, cv2.LINE_AA)
            if spot_id <= 9:
                number_text = '0'+ str(spot_id)
            else:
                number_text = str(spot_id)
            number_size = cv2.getTextSize(number_text,  font, font_scale*0.8, 1)[0]
            cv2.putText(output,  number_text,
                        (center_x - number_size[0]//2, center_y + text_size[1]//2 - 20),  # 在方框顶部显示编号 
                        font, font_scale*0.8, (0, 0, 0), 1, cv2.LINE_AA)

        else:
            output[center_y-45:center_y+45, center_x-19:center_x+19] = FILL_COLOR_UNAVAILABLE
                    # 绘制斜线
            cv2.line(output,  (center_x-19+5, center_y-45+5), (center_x-19+19*2-5, center_y-45+90-5), PARKING_UNAVAILABLE_COLOR, 1)
            cv2.line(output,  (center_x-19+19*2-5, center_y-45+5), (center_x-19+5, center_y-45+90-5), PARKING_UNAVAILABLE_COLOR, 1) 

        step = 10 
        for i in range(x, x + w, step):
            cv2.line(output,  (i, y), (i + 5, y), BORDER_COLOR, 1)
            cv2.line(output,  (i, y + h), (i + 5, y + h), BORDER_COLOR, 1)
        for j in range(y, y + h, step):
            cv2.line(output,  (x, j), (x, j + 5), BORDER_COLOR, 1)
            cv2.line(output,  (x + w, j), (x + w, j + 5), BORDER_COLOR, 1)

    return output

class LotInfoSubscriber(Node):
    def __init__(self):
        super().__init__('lot_info_subscriber')

        # # 创建订阅者
        # self.subscription = self.create_subscription(
        #     UInt16MultiArray,
        #     '/lot_info',
        #     self.listener_callback,
        #     10)
        # self.subscription  # prevent unused variable warning

        self.image_pub_ = self.create_publisher(Image, "/lot_image", 10)  # ✅ 发布图片的 topic

#####################
        # 截图演示用，发布随机空余车位
        # 1.产生空位
        length = random.randint(0,  50)
        self.free_lot_id = random.sample(range(1,60),  length)  # 保证数字不重复 
        self.free_lot_id = sorted(self.free_lot_id)
        self.info_pub_ = self.create_publisher(UInt16MultiArray, "/lot_info", 10)   # 创建发布者对象（消息类型、话题名、队列长度）
        self.test_timer_ = self.create_timer(1.0, self.test_callback)            # 创建一个定时器（单位为秒的周期，定时执行的回调函数）
#####################

        self.bridge = CvBridge()
        # self.free_lot_id = []
        self.info_pub_timer_ = self.create_timer(2.0, self.info_pub_callback)

#####################
    # 截图演示用，发布随机空余车位
    def test_callback(self):

        #####
        # 随机
        length = random.randint(0,  50)
        self.free_lot_id = random.sample(range(1,60),  length)  # 保证数字不重复 
        self.free_lot_id = sorted(self.free_lot_id)

        #####
        msg = UInt16MultiArray()
        msg.data = self.free_lot_id
        self.get_logger().info(f'当前空余车位id{msg.data}') 
        self.info_pub_.publish(msg)
#####################

    def listener_callback(self, msg):
        free_lot_id = msg.data
        if free_lot_id:
            self.free_lot_id = list(free_lot_id)
            self.get_logger().info(f'接收到空余车位 ID: {list(free_lot_id)}')
        else:
            self.get_logger().info('暂无空余车位')

    def info_pub_callback(self):
        # 使用 plt_lot 绘制停车场图像
        start_time = time.time()
        image = plt_lot(self.free_lot_id)
        end_time = time.time()
        run_time = end_time - start_time
        self.get_logger().info(f"程序的运行时间为：{run_time}")

        # 转换并发布图像消息
        image_msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        self.image_pub_.publish(image_msg)

        self.get_logger().info(f"已发布空车位: {self.free_lot_id}，及对应图像")

def main(args=None):
    rclpy.init(args=args)
    node = LotInfoSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
