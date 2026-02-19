import cv2
import numpy as np

# 读取图像
# image = cv2.imread('/home/www/auto_parking/auto_parking_ws/better/EI/pdf/ei_map.png')
image = cv2.imread('/home/www/auto_parking/server_ws/src/server_python/server_python/map2.png')

hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# 设定蓝色 HSV 范围（可根据图像微调）
lower_blue = np.array([100, 100, 50])
upper_blue = np.array([140, 255, 255])

# 生成蓝色掩码
mask = cv2.inRange(hsv, lower_blue, upper_blue)

# 查找轮廓
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# 创建拷贝用于绘制
output = image.copy()

# 灰色虚线颜色
line_color = (128, 128, 128)

# 处理每个蓝色方框
for cnt in contours:
    x, y, w, h = cv2.boundingRect(cnt)

    # 将方框区域填充为白色（或你想要的背景色）
    output[y:y+h, x:x+w] = 255  # 变为白色

    # 绘制灰色虚线边框
    step = 15  # 虚线间隔
    for i in range(x, x + w, step):
        cv2.line(output, (i, y), (i + 5, y), line_color, 3)         # 上边
        cv2.line(output, (i, y + h), (i + 5, y + h), line_color, 3) # 下边
    for j in range(y, y + h, step):
        cv2.line(output, (x, j), (x, j + 5), line_color, 3)         # 左边
        cv2.line(output, (x + w, j), (x + w, j + 5), line_color, 3) # 右边

gray_image = cv2.cvtColor(output,  cv2.COLOR_BGR2GRAY)

# 显示图像
cv2.imshow('Gray Boxed Image', gray_image)
cv2.waitKey(0)
cv2.destroyAllWindows()

