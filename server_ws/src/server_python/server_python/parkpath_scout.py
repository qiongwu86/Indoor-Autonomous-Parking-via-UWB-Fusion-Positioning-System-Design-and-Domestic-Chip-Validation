import math
import numpy as np
# from .path_theta import *
import matplotlib.pyplot as plt

from .Astar_my import *       # 节点
# from Astar_my import *      # 直接运行


"""
用于产生车辆的泊车路径
"""
# 根据角度返回均匀的圆弧
class CircleArcGenerator:
    def __init__(self, center, point_a, point_b):
        self.center  = center  # 圆心坐标 (O_x, O_y)
        self.point_a  = point_a  # 点 A 的坐标 (A_x, A_y)
        self.point_b  = point_b  # 点 B 的坐标 (B_x, B_y)
        self.radius  = self._calculate_radius()
        self.angle_a  = self._calculate_angle(self.point_a) 
        self.angle_b  = self._calculate_angle(self.point_b) 
        self.total_angle  = self.angle_b  - self.angle_a  
 
    def _calculate_radius(self):
        # 计算半径 
        dx = self.point_a[0]  - self.center[0] 
        dy = self.point_a[1]  - self.center[1] 
        return math.hypot(dx,  dy)
 
    def _calculate_angle(self, point):
        # 计算点相对于圆心的角度 
        dx = point[0] - self.center[0] 
        dy = point[1] - self.center[1] 
        return math.atan2(dy,  dx)
 
    def generate_points(self, num_points=10):
        # 生成均匀分布的点 
        points = []
        if num_points <= 0:
            return points 
 
        step = self.total_angle  / (num_points - 1)
        current_angle = self.angle_a  
 
        for _ in range(num_points):
            x = self.center[0]  + self.radius  * math.cos(current_angle) 
            y = self.center[1]  + self.radius  * math.sin(current_angle) 
            points.append([x,  y])
            current_angle += step 
 
        return points 

def test(x_start, y_start):
    """
    根据起始坐标产生泊车路径
    Args:
        x_start:
        y_start:

    Returns:

    """
    # 以cm为单位

    r = [270, 270, 200]
    # r = [400, 400, 200]

    car_length = 93  # 车长
    car_width = 70  # 车宽
    L = 50

    # 根据起始点计算xy1、2
    # xy1 = [x_start + 3 / 2 * car_width, y_start + car_length + car_length]        # 本来可以跑的
    xy1 = [x_start + 3 / 2 * car_width, y_start + car_length]          # 修改了

    xy2 = [xy1[0],xy1[1] + 125]

    d0 = abs(xy1[0] - x_start)

    # 后轮轴中心坐标
    x0 = x_start
    y0 = y_start
    # print(f'后轮轴初始坐标{(x0, y0)}')

    # 车位入口点坐标
    x_1 = xy1[0]
    y_1 = xy1[1]
    x_2 = xy2[0]
    y_2 = xy2[1]

    # 切换点c的坐标计算
    x_c = (x_1 + x_2) / 2
    y_c = (y_1 + y_2) / 2
    # print(f"c点坐标{(x_c, y_c)}")
    D = 0

    R1 = r[0]
    R2 = r[1]
    # 圆心O2的坐标计算
    x_o2 = x_c
    y_o2 = y_c + R2

    # print('x_o2', x_o2, 'y_o2', y_o2)

    # 切换点b的坐标计算
    alp = math.asin((d0 + R2 + D) / (R1 + R2))  # alp为切换点c到切换点b的转动角度
    x_b = (x_o2 - R2 * (math.sin(alp))) // 1
    y_b = (y_o2 - R2 * (math.cos(alp))) // 1
    x_b = x_b
    y_b = y_b

    # print(f'B点坐标{(x_b, y_b)}')

    # 圆心O1的坐标计算
    x_o1 = x_o2 - (R1 + R2) * (math.sin(alp))
    y_o1 = y_o2 - (R1 + R2) * (math.cos(alp))
    x_o1 = x_o1
    y_o1 = y_o1

    # print('x_o1', x_o1, 'y_o1', y_o1)
    # 切换点A的坐标计算
    x_a = x_o1 + R1
    y_a = y_o1
    # print(f'A点坐标{(x_a, y_a)}')

    # 结束点后轮轴中心坐标
    x_end = (x_1 + x_2) / 2 + car_length + car_length
    y_end = (y_1 + y_2) / 2
    # print(f'后轮轴结束坐标{(x_end, y_end)}')

    # 从初始点到A点
    list0 = []
    for y in range(int(y0), int(y_a), 8):   # 5 1 7 9
        list0.append((x0, y))
#############################################################
    # list1 = []
    # # 从a点到b点
    # for x in range(int(x_b*2), int(x_a*2), 6):  # 3 3
    #     x = x / 2.
    #     y = math.sqrt(R1 ** 2 - (x - x_o1) ** 2) + y_o1
    #     list1.append((x, y))
    # y = math.sqrt(R1 ** 2 - (x_a - 1 - x_o1) ** 2) + y_o1
    # list1.append((x_a - 1, y))
    # list1.reverse()
    # print("list1 len : ",len(list1))        # 29
    # list2 = []

    # # 从b点到c点
    # i = 0
    # for x in range(int(x_b), int(x_c), 7):  # 7 5
    #     if i == 0:
    #         i = i + 1
    #         continue
    #     y = -math.sqrt(R2 ** 2 - (x - x_o2) ** 2) + y_o2

    #     list2.append((x, y))
    # print("list2 len : ",len(list2))        # 26
#############################################################
# 该用角度产生均匀的圆
    # 从A到B
    generator_a2b = CircleArcGenerator([x_o1,y_o1], [x_a,y_a], [x_b,y_b])
    list1 = generator_a2b.generate_points(27) # 8-6 20-》25
    # 从B到C
    generator_b2c = CircleArcGenerator([x_o2,y_o2], [x_b,y_b], [x_c,y_c])
    list2 = generator_b2c.generate_points(27) # 8-6 20-》25
#############################################################
    # 从c点到结束点
    list3 = []
    for x in range(int(x_c) + 5, int(x_end) + 5 + 240, 8):    # 5 3 6 8 9 10
        list3.append((x, y_end))
    list3.append((x_end + 5, y_end))

    ensure_path2 = []
    for x in range(int(x_end), int(x_end) + 5, 3):  # 3 1
        ensure_path2.append((x, y_end))
    ensure_path2.reverse()

    list_t = list0 + list1 + list2 + list3

    park_path = np.array(list_t)
    # park_path = park_path / 10 / 7
    park_path = park_path / 100  # 这个好一点

    return park_path
# 向上泊车
def parkpath(parkstart_x, parkstart_y):
    # 设置起始点
    x_start = 0.
    y_start = 0.
    # 产生路径
    park_path = test(x_start, y_start)

    # 旋转路径
    # park_path = path_theta.rotate_path(park_path, math.radians(270))
    park_path = rotate_path(park_path, math.radians(270))

    # 对称路径
    # park_path = path_theta.flip_path_x(park_path)
    park_path = flip_path_x(park_path)
    park_path = park_path + [parkstart_x, parkstart_y]

    return park_path
# 向下泊车
def down(parkstart_x, parkstart_y):
    # 设置起始点
    x_start = 0.
    y_start = 0.
    # 产生路径
    park_path = test(x_start, y_start)

    # # 旋转路径
    park_path = rotate_path(park_path, math.radians(270))

    # 对称路径
    # park_path = path_theta.flip_path_x(park_path)
    # park_path = flip_path_x(park_path)
    park_path = park_path + [parkstart_x, parkstart_y]

    return park_path

def show_polt(list,ax):
    x = []
    y = []

    for i in range(len(list)):
        x.append(list[i][0])
        y.append(list[i][1])

    plt.figure(figsize=(6, 6))

    if ax:
        # 坐标轴同比例
        ax = plt.gca()
        ax.set_aspect(1)

    plt.scatter(x,y)
    plt.grid()
    plt.show()

if __name__ == '__main__':
    
    park_path = down(0., 0.)
    show_polt(park_path,1)


