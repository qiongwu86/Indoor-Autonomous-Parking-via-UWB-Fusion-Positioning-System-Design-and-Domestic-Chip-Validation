# import cv2
# import numpy as np

# # 读取图像
# # image = cv2.imread('/home/www/auto_parking/auto_parking_ws/better/EI/pdf/ei_map.png')
# image = cv2.imread('/home/www/auto_parking/server_ws/src/server_python/server_python/map2.png')

# # 将非方框区域转为灰度（保留原图用于处理）
# gray_background = cv2.cvtColor(image,  cv2.COLOR_BGR2GRAY)
# gray_background = cv2.cvtColor(gray_background,  cv2.COLOR_GRAY2BGR)  # 转回BGR格式用于后续合成 

# hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# # 设定蓝色 HSV 范围（可根据图像微调）
# lower_blue = np.array([100, 100, 50])
# upper_blue = np.array([140, 255, 255])

# # 生成蓝色掩码
# mask = cv2.inRange(hsv, lower_blue, upper_blue)

# # 查找轮廓
# contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# # 创建拷贝用于绘制
# # output = image.copy()
# output = gray_background.copy() # 灰色

# # 灰色虚线颜色
# line_color = (128, 128, 128)

# is_available = True

# # 处理每个蓝色方框
# for cnt in contours:
#     x, y, w, h = cv2.boundingRect(cnt)

#     center_x = x + w // 2
#     center_y = y + h // 2 

#     if center_x < 300 or center_x > 780:
#         is_available = False
#     else:
#         is_available = True

#     # # 将方框区域填充为白色（或你想要的背景色）
#     # output[y:y+h, x:x+w] = 255  # 变为白色
#     # 方框填充色（可用=浅绿色，不可用=浅红色）
#     fill_color = (200, 255, 200) if is_available else (220, 220, 255)
#     output[y:y+h, x:x+w] = fill_color

#     # 绘制灰色虚线边框
#     step = 10  # 虚线间隔
#     for i in range(x, x + w, step):
#         cv2.line(output, (i, y), (i + 5, y), line_color, 2)         # 上边
#         cv2.line(output, (i, y + h), (i + 5, y + h), line_color, 2) # 下边
#     for j in range(y, y + h, step):
#         cv2.line(output, (x, j), (x, j + 5), line_color, 2)         # 左边
#         cv2.line(output, (x + w, j), (x + w, j + 5), line_color, 2) # 右边

#     # 在方框中心绘制"P"字 
#     font = cv2.FONT_HERSHEY_SIMPLEX
#     font_scale = min(w, h) / 80  # 根据方框大小自动调整字体大小 
#     thickness = 2
#     text_size = cv2.getTextSize("P",  font, font_scale, thickness)[0]

#     # 计算文字位置（使文字居中）
#     text_x = center_x - text_size[0] // 2 
#     text_y = center_y + text_size[1] // 2 
    
#     # cv2.putText(output,  f"{center_y}", (text_x, text_y), font, font_scale, (0, 0, 0), thickness, cv2.LINE_AA)

#     if is_available:
#         cv2.putText(output,  f"P", (text_x, text_y), font, font_scale, (0, 180, 0), thickness, cv2.LINE_AA)
#     else:
#                 # 红色"X"
#         # text_size = cv2.getTextSize("X",  font, font_scale, 2)[0]
#         # cv2.putText(output,  "X", (center_x-text_size[0]//2, center_y+text_size[1]//2), 
#         #            font, font_scale, (0, 0, 255), 2, cv2.LINE_AA)
#         # 可选：绘制斜线加强表示 
#         cv2.line(output,  (x+5, y+5), (x+w-5, y+h-5), (0, 0, 200), 1)
#         cv2.line(output,  (x+w-5, y+5), (x+5, y+h-5), (0, 0, 200), 1)
    
# gray_image = cv2.cvtColor(output,  cv2.COLOR_BGR2GRAY)

# # 显示图像
# cv2.imshow('Gray Boxed Image', output)
# cv2.waitKey(0)
# cv2.destroyAllWindows()

import cv2 
import numpy as np
import random

# # 车位坐标(全图米坐标)
# cars = {1:[22.0, 51.2], 2:[24.3, 51.2], 3:[26.6, 51.2], 4:[30.0, 51.2], 5:[32.3, 51.2], 6:[34.6, 51.2], 7:[38.2, 151.2], 8:[40.5, 51.2], 9:[42.8, 51.2], 
#     10:[18.5, 39.9], 11:[22.0, 39.9], 12:[24.3, 39.9], 13:[26.6, 39.9], 14:[30.0, 39.9], 15:[32.3, 39.9], 16:[34.6, 39.9], 17:[38.2, 39.9], 18:[40.5, 39.9], 19:[42.8, 39.9], 
#     20:[18.5, 35.0], 21:[22.0, 35.0], 22:[24.3, 35.0], 23:[26.6, 35.0], 24:[30.0, 35.0], 25:[32.3, 35.0], 26:[34.6, 35.0], 27:[38.2, 35.0], 28:[40.5, 35.0], 29:[42.8, 35.0],
#     30:[18.5, 23.5], 31:[22.0, 23.5], 32:[24.3, 23.5], 33:[26.6, 23.5], 34:[30.0, 23.5], 35:[32.3, 23.5], 36:[34.6, 23.5], 37:[38.2, 23.5], 38:[40.5, 23.5], 39:[42.8, 23.5],
#     40:[18.5, 18.5], 41:[22.0, 18.5], 42:[24.3, 18.5], 43:[26.6, 18.5], 44:[30.0, 18.5], 45:[32.3, 18.5], 46:[34.6, 18.5], 47:[38.2, 18.5], 48:[40.5, 18.5], 49:[42.8, 18.5],
#     50:[18.0, 7.10], 51:[22.0, 7.10], 52:[24.3, 7.10], 53:[26.6, 7.10], 54:[30.0, 7.10], 55:[32.3, 7.10], 56:[34.6, 7.10], 57:[38.2, 7.10], 58:[40.5, 7.10], 59:[42.8, 7.10],
#     }

#################################
# # 车位坐标(像素坐标)
# cars = {1: [389, 55], 2: [431, 55], 3: [473, 55], 4: [532, 55], 5: [574, 55], 6: [616, 55], 7: [676, 55], 8: [718, 55], 9: [759, 55], 
#         10: [331, 253], 11: [389, 253], 12: [431, 253], 13: [473, 253], 14: [533, 253], 15: [575, 253], 16: [616, 253], 17: [676, 253], 18: [718, 253], 19: [760, 253], 
#         20: [331, 346], 21: [389, 346], 22: [431, 346], 23: [473, 346], 24: [532, 346], 25: [574, 346], 26: [616, 346], 27: [676, 346], 28: [718, 346], 29: [759, 346], 
#         30: [331, 540], 31: [389, 541], 32: [431, 541], 33: [473, 541], 34: [533, 541], 35: [575, 541], 36: [616, 541], 37: [676, 541], 38: [718, 541], 39: [760, 541], 
#         40: [331, 633], 41: [389, 634], 42: [431, 634], 43: [473, 634], 44: [532, 634], 45: [574, 634], 46: [616, 634], 47: [676, 634], 48: [718, 634], 49: [759, 634], 
#         50: [318, 831], 51: [389, 831], 52: [431, 831], 53: [473, 831], 54: [533, 831], 55: [575, 831], 56: [616, 831], 57: [676, 831], 58: [718, 831], 59: [760, 831]}

# length = random.randint(0,  50)
# free_lot_id = random.sample(range(1,60),  length)  # 保证数字不重复 
# print(free_lot_id)
# # 读取图像 
# image = cv2.imread('/home/www/auto_parking/server_ws/src/server_python/server_python/map2.png') 
 
# # 将非方框区域转为灰度 
# gray_background = cv2.cvtColor(image,  cv2.COLOR_BGR2GRAY)
# gray_background = cv2.cvtColor(gray_background,  cv2.COLOR_GRAY2BGR)
 
# hsv = cv2.cvtColor(image,  cv2.COLOR_BGR2HSV)
 
# # 设定蓝色HSV范围 
# lower_blue = np.array([100,  100, 50])
# upper_blue = np.array([140,  255, 255])
 
# # 生成蓝色掩码 
# mask = cv2.inRange(hsv,  lower_blue, upper_blue)
 
# # 查找轮廓 
# contours, _ = cv2.findContours(mask,  cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
 
# # 创建拷贝用于绘制 
# output = gray_background.copy() 
 
# # 颜色定义 
# PARKING_AVAILABLE_COLOR = (0, 180, 0)  # 深绿色
# PARKING_UNAVAILABLE_COLOR = (0, 0, 200)  # 深红色 
# BORDER_COLOR = (128, 128, 128)  # 灰色边框 
# FILL_COLOR_AVAILABLE = (200, 255, 200)  # 浅绿填充
# FILL_COLOR_UNAVAILABLE = (220, 220, 255)  # 浅红填充

# # 首先对所有车位进行排序（从左到右，从上到下）
# # 使用中心点坐标作为排序依据 
# sorted_contours = sorted(contours, key=lambda c: (cv2.boundingRect(c)[1],  cv2.boundingRect(c)[0])) 
 
# # 初始化车位编号 
# parking_number = 1

# # 处理方框 
# for cnt in sorted_contours:
#     x, y, w, h = cv2.boundingRect(cnt)      # _,_,38,90
#     center_x = x + w // 2
#     center_y = y + h // 2 

#     # 判断是否在指定区域
#     in_special_area = center_x < 300 or center_x > 780 
#     is_available = not in_special_area  # 指定区域为不可用 
 
#     # 方框填充
#     fill_color = (255,255,255) if is_available else FILL_COLOR_UNAVAILABLE 
#     output[y:y+h, x:x+w] = fill_color
 
#     # 绘制虚线边框 
#     step = 10 
#     for i in range(x, x + w, step):
#         cv2.line(output,  (i, y), (i + 5, y), BORDER_COLOR, 1)
#         cv2.line(output,  (i, y + h), (i + 5, y + h), BORDER_COLOR, 1)
#     for j in range(y, y + h, step):
#         cv2.line(output,  (x, j), (x, j + 5), BORDER_COLOR, 1)
#         cv2.line(output,  (x + w, j), (x + w, j + 5), BORDER_COLOR, 1)
#     if not is_available:
#         # 绘制斜线
#         cv2.line(output,  (x+5, y+5), (x+w-5, y+h-5), (0, 0, 200), 1)
#         cv2.line(output,  (x+w-5, y+5), (x+5, y+h-5), (0, 0, 200), 1)

# for spot_id, (center_x, center_y) in cars.items(): 
#     # 检查是否是可泊车位 
#     is_available = spot_id in free_lot_id

#     # 绘制标记 
#     font = cv2.FONT_HERSHEY_DUPLEX
#     font_scale = min(38, 90) / 60
#     x = center_x - 19
#     y = center_y - 45
#     w = 38
#     h = 90

#     # 如果是可泊车位，绘制绿色"P"
#     if is_available:
#         output[center_y-45:center_y+45, center_x-19:center_x+19] = FILL_COLOR_AVAILABLE

#         # 绿色"P"
#         text_size = cv2.getTextSize("P",  font, font_scale, 2)[0]
#         cv2.putText(output,  "P", 
#                     (center_x - text_size[0]//2, center_y + text_size[1]//2),
#                     font, font_scale, PARKING_AVAILABLE_COLOR, 2, cv2.LINE_AA)

#         number_text = str(spot_id)
#         number_size = cv2.getTextSize(number_text,  font, font_scale*0.8, 1)[0]
#         cv2.putText(output,  number_text,
#                     (center_x - number_size[0]//2, center_y + text_size[1]//2 - 20),  # 在方框顶部显示编号 
#                     font, font_scale*0.8, (0, 0, 0), 1, cv2.LINE_AA)

#     else:
#         output[center_y-45:center_y+45, center_x-19:center_x+19] = FILL_COLOR_UNAVAILABLE
#                 # 绘制斜线
#         cv2.line(output,  (center_x-19+5, center_y-45+5), (center_x-19+19*2-5, center_y-45+90-5), PARKING_UNAVAILABLE_COLOR, 1)
#         cv2.line(output,  (center_x-19+19*2-5, center_y-45+5), (center_x-19+5, center_y-45+90-5), PARKING_UNAVAILABLE_COLOR, 1) 

#     step = 10 
#     for i in range(x, x + w, step):
#         cv2.line(output,  (i, y), (i + 5, y), BORDER_COLOR, 1)
#         cv2.line(output,  (i, y + h), (i + 5, y + h), BORDER_COLOR, 1)
#     for j in range(y, y + h, step):
#         cv2.line(output,  (x, j), (x, j + 5), BORDER_COLOR, 1)
#         cv2.line(output,  (x + w, j), (x + w, j + 5), BORDER_COLOR, 1)


# # print(cars)
# # 显示图像 
# cv2.imshow('Parking  Status with Numbering', output)
# cv2.waitKey(0) 
# cv2.destroyAllWindows() 

# 绘画初始地图
def create_map():
    # 读取图像 
    image = cv2.imread('/home/www/auto_parking/server_ws/src/server_python/server_python/map2.png') 
    
    # 将非方框区域转为灰度 
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
    output = gray_background.copy() 
    
    # 颜色定义 
    PARKING_AVAILABLE_COLOR = (0, 180, 0)  # 深绿色
    PARKING_UNAVAILABLE_COLOR = (0, 0, 200)  # 深红色 
    BORDER_COLOR = (128, 128, 128)  # 灰色边框 
    FILL_COLOR_AVAILABLE = (200, 255, 200)  # 浅绿填充
    FILL_COLOR_UNAVAILABLE = (220, 220, 255)  # 浅红填充

    # 首先对所有车位进行排序（从左到右，从上到下）
    # 使用中心点坐标作为排序依据 
    sorted_contours = sorted(contours, key=lambda c: (cv2.boundingRect(c)[1],  cv2.boundingRect(c)[0])) 
    
    # 初始化车位编号 
    parking_number = 1

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

    # 显示图像 
    cv2.imshow('Parking  Status with Numbering', output)
    cv2.waitKey(0) 
    cv2.destroyAllWindows() 

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


    # print(cars)
    # 显示图像 
    cv2.imshow('Parking  Status with Numbering', output)
    cv2.waitKey(0) 
    cv2.destroyAllWindows() 

def main():
    # 绘制初始地图并保存
    # create_map() 
    # 产生列表
    length = random.randint(0,  50)
    free_lot_id = random.sample(range(1,60),  length)  # 保证数字不重复 
    print(free_lot_id)
    # 使用
    plt_lot(free_lot_id)

if __name__ == '__main__':
    main()
