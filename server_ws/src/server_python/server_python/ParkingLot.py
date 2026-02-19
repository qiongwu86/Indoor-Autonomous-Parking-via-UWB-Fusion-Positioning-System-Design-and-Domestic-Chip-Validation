from geometry_msgs.msg import Pose2D
import random 
import numpy as np
import math
from sklearn.cluster import KMeans
from geometry_msgs.msg import Pose2D
from nav_2d_msgs.msg import Path2D
import cv2

"""
Vehicle:
    记录停车场中所需的汽车信息:名称;开始时间;初始位姿
ParkingSpot:
    车位类
ParkingLot:
    停车场信息类

计算两点的角度/路径格式转化/地图配置/目标点坐标转换/匹配岔路点/去除重复点，带容许差的/各种坐标转换
"""

# 大图中的
pixel_to_meter = 0.017
# 基站A0的偏置
global offset
offset_x = 13.559322
global offset_y
offset_y = 10.153999

# 小图中的
pixel_to_meter_m=0.0564

# # 岔路点(全图米坐标)
# junction_points = [[15.23,  45.57],[58.5,   45.57],
#                    [15.23,  29.05],[58.5,   29.05],
#                    [15.23,  12.80],[58.5,   12.80]]
# 岔路点(全图米坐标)
junction_points = [[15.40,  45.57],[58.5,   45.57],
                   [15.40,  29.05],[58.5,   29.05],
                   [15.40,  12.80],[58.5,   12.80]]

# 车位坐标(全图米坐标)
cars = {1:[22.0, 51.2], 2:[24.3, 51.2], 3:[26.6, 51.2], 4:[30.0, 51.2], 5:[32.3, 51.2], 6:[34.6, 51.2], 7:[38.2, 151.2], 8:[40.5, 51.2], 9:[42.8, 51.2], 
    10:[18.5, 39.9], 11:[22.0, 39.9], 12:[24.3, 39.9], 13:[26.6, 39.9], 14:[30.0, 39.9], 15:[32.3, 39.9], 16:[34.6, 39.9], 17:[38.2, 39.9], 18:[40.5, 39.9], 19:[42.8, 39.9], 
    20:[18.5, 35.0], 21:[22.0, 35.0], 22:[24.3, 35.0], 23:[26.6, 35.0], 24:[30.0, 35.0], 25:[32.3, 35.0], 26:[34.6, 35.0], 27:[38.2, 35.0], 28:[40.5, 35.0], 29:[42.8, 35.0],
    30:[18.5, 23.5], 31:[22.0, 23.5], 32:[24.3, 23.5], 33:[26.6, 23.5], 34:[30.0, 23.5], 35:[32.3, 23.5], 36:[34.6, 23.5], 37:[38.2, 23.5], 38:[40.5, 23.5], 39:[42.8, 23.5],
    40:[18.5, 18.5], 41:[22.0, 18.5], 42:[24.3, 18.5], 43:[26.6, 18.5], 44:[30.0, 18.5], 45:[32.3, 18.5], 46:[34.6, 18.5], 47:[38.2, 18.5], 48:[40.5, 18.5], 49:[42.8, 18.5],
    50:[18.0, 7.10], 51:[22.0, 7.10], 52:[24.3, 7.10], 53:[26.6, 7.10], 54:[30.0, 7.10], 55:[32.3, 7.10], 56:[34.6, 7.10], 57:[38.2, 7.10], 58:[40.5, 7.10], 59:[42.8, 7.10],
    }


class Vehicle: 
    def __init__(self, car_name, start_time, initial_pose, parking_id,uwb_id): 

        self.car_name  = car_name               # 名称
        self.start_time  = start_time           # 开始时间
        self.initial_pose  = initial_pose       # 初始位姿
        self.current_pose  = initial_pose       # 当前位姿
        self.parking_space = parking_id
        self.info_i = 0
        self.uwb_id = uwb_id
    # def __str__(self): 
        # return f"车牌号: {self.car_name},  入场时间: {self.start_time},  停车位置: {self.parking_space}"  
    
    def update(self, x, y): 
        current_pose = Pose2D()
        current_pose.x = x
        current_pose.y = y
        self.current_pose = current_pose
        # print("this is test")

###################################################################
# 车位信息
class ParkingSpot:
    def __init__(self, spot_id, x, y):
        self.spot_id  = spot_id  # 车位编号 
        self.coordinate  = (x, y)  # 二维坐标 
        self.occupied  = False  # 占用状态，默认为空闲 

# 停车场信息
class ParkingLot:
    def __init__(self, num_spots):
        self.num_spots  = num_spots  # 车位总数 
        self.offset_x = offset_x          # 偏移信息          
        self.offset_y = offset_y
        self.spots_free = {}
        self.spots_occupied = {}
        self.spots  = self._initialize_spots()  # 初始化所有车位 

    def _initialize_spots(self):                # 车位初始化
        spots=[]
        x_list=[]
        y_list=[]
        for j in range(len(cars)):              # 车位坐标为基于A0的米坐标
            x_list.append(cars[j+1][0]-self.offset_x)
            y_list.append(cars[j+1][1]-self.offset_y)
        for i in range(self.num_spots): 
            spot = ParkingSpot(i+1, x_list[i], y_list[i])
            spots.append(spot)
            self.spots_free[i+1] = spot
        return spots 
 
    def select_available_spot(self,spot_id):
        # print(f'所选车位为:{self.spots[spot_id-1].spot_id}')
        # print(f'所选车位坐标为:{self.spots[spot_id-1].coordinate}')
        # print(f'该车位是否被占用:{self.spots[spot_id-1].occupied}')

        # 1.判断当前选择车位是否已经被占用
        if  self.spots[spot_id-1].occupied == True:
            return 0
        # 2.判断当前选择车位是否有效
        if spot_id <= 0 or spot_id > self.num_spots:
            return 1
        # 3.占用该车为，并返回二位坐标
        else:
            self.spots[spot_id-1].occupied = True               # 设置标志位
            value = self.spots_free.pop(spot_id, '键不存在')        # 并移除spots_free
            self.spots_occupied[spot_id] = value                    # 放入spots_occupied
            # print(self.spots_occupied[spot_id].spot_id)       # 测试
            return self.spots[spot_id-1].coordinate             # 返回所选车位的二维坐标
 
    def random_occupy(self,n):
        # 生成5个1到14之间的随机整数 
        random_numbers = random.sample(range(1,  59), n)
        # print("生成的随机整数为：", random_numbers)
        occupied_id_list = []
        for id in random_numbers:
            self.select_available_spot(id)
            occupied_id_list.append(id)

        return occupied_id_list                                 # 返回已经占用的车位id
    
    def check_occupy(self):                                     # 返回所有空余车位的id_list
        # for spot in self.spots:                               # 所有车位的占用情况
            # print(f'{spot.spot_id}:',spot.occupied)
        free_id_list = []
        if self.spots_free == {}:
            print('没有空余车位了！')
        else:
            for id in self.spots_free:
                free_id_list.append(id)
        return free_id_list
    
    def random_choose(self):
        free_id_list = self.check_occupy()
        random_element = random.choice(free_id_list) 
        xy = self.select_available_spot(random_element)
        # print('所选车位为:',random_element)

        return random_element, xy

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

def to_path2d(path):
    
    path2d = Path2D()
    
    for i in range(len(path)):
        pose = Pose2D()                 # 注意每次循环都需要实例化
        pose.x = path[i][0]
        pose.y = path[i][1]
        path2d.poses.append(pose)
        
        # print(path2d.poses)
    return path2d

def to_path(path_2d:Path2D):
    path = []
    for point in path_2d.poses:
        x = point.x
        y = point.y
        path.append((x,y))
    path = np.array(path)
    return path
# 地图配置
def map_config():
    # 读取小图 导航像素坐标(1229*958)
    rah = np.flip(cv2.imread('/home/www/auto_parking/server_ws/src/server_python/server_python/map2.png', 0), axis=0)    # TODO:配置路径

    # 提取障碍物点
    threshold = 128
    obs = np.vstack([np.where(rah < threshold)[1], np.where(rah < threshold)[0]]).T
    ox = [int(item) for item in obs[:, 0]]
    oy = [int(item) for item in obs[:, 1]]
    # Kmeans
    clusters = KMeans(n_clusters=300).fit(np.column_stack((ox, oy)))
    ox, oy = clusters.cluster_centers_[:, 0], clusters.cluster_centers_[:, 1]
    # 基于小图像素坐标的ox,oy
    return ox, oy
# 目标点坐标转换
def gxy_change(id,xy):
    # # 岔路点(全图米坐标)
    # junction_points = [[15.23,  45.57],[58.5,   45.57],
    #                [15.23,  29.05],[58.5,   29.05],
    #                [15.23,  12.80],[58.5,   12.80]]
    # 岔路点(全图米坐标)                                    TODO:
    junction_points = [[15.40,  45.57],[58.5,   45.57],
                    [15.40,  29.05],[58.5,   29.05],
                    [15.40,  12.80],[58.5,   12.80]]
    # A0的米坐标 ---> 导航像素坐标(1229*958)
    if id <= 19:
        gy = junction_points[0][1]/pixel_to_meter_m                # 固定为上层岔路点的y坐标
        gx = (xy[0]+offset_x-1.)/pixel_to_meter_m
    elif id <= 39:
        gy = junction_points[2][1]/pixel_to_meter_m                # 固定为中层岔路点的y坐标
        gx = (xy[0]+offset_x-1.)/pixel_to_meter_m
    else:
        gy = junction_points[4][1]/pixel_to_meter_m                # 固定为下层岔路点的y坐标
        gx = (xy[0]+offset_x-1.)/pixel_to_meter_m
    # print(f'gx,gy:{gx,gy}')
    return gx,gy
# 匹配岔路点
def findJPs(pathx,pathy):
    # # 岔路点(全图米坐标)
    # junction_points = [[15.23,  45.57],[58.5,   45.57],
    #                [15.23,  29.05],[58.5,   29.05],
    #                [15.23,  12.80],[58.5,   12.80]]
    # 岔路点(全图米坐标)                                    TODO:
    junction_points = [[15.40,  45.57],[58.5,   45.57],
                    [15.40,  29.05],[58.5,   29.05],
                    [15.40,  12.80],[58.5,   12.80]]
    path = []
    for i in range(len(pathx)):
        path.append((pathx[i],pathy[i]))
    # 将junction_points转化成像素坐标
    junction_points = np.array(junction_points)
    junction_points = junction_points/pixel_to_meter_m
    # 匹配岔路点
    threshold = 50                                                  # 设置阈值
    JPs = find_near_points(junction_points, path, threshold)     # 匹配
    JPs = remove_with_tolerance(JPs)                          # 去重得到所需经历的岔路点

    return JPs

# 去除重复点，带容许差的
def remove_with_tolerance(points, tol=1e-3):
    """

    :param points:
    :param tol: 容许差
    :return:
    """
    if not points:  # 处理空输入
        print("no points")
        return []

    result = [points[0]]
    for p in points[1:]:
        if not np.allclose(p,  result[-1], atol=tol):       # 比较两个点的所有维度坐标，若每个维度的绝对差值均 <= tol，则视为“足够接近”
            result.append(p)
    return result

def find_near_points(points, path, threshold):
    """
    筛选出到路径距离小于1的所有点
    :param points: 单点列表，格式如[[x1,y1], [x2,y2], ...]
    :param path: 路径点列表，格式如[[x1,y1], [x2,y2], ...]
    :return: 符合条件的点列表
    """
    near_points = []

    # 遍历路径中的每条线段
    for i in range(len(path) - 1):
        min_dist = float('inf')
        min_point=[]
        a = path[i]
        b = path[i + 1]
        for j in range(len(points)):
            p = points[j]
            dist = point_to_segment_dist(p, a, b)
            # print(f'{p}to{a,b}:{dist}')

            # 判断距离是否小于threshold
            if dist < threshold:
                if dist < min_dist:             # 更新最近距离
                    min_dist = dist
                    min_point = p               # 记录最近的点
        if min_dist < threshold:
            near_points.append(min_point)

    return near_points

def point_to_segment_dist(p, a, b):
    """
    计算点p到线段ab的最短距离
    :param p: 待计算点，格式为[x, y]
    :param a: 线段起点，格式为[x, y]
    :param b: 线段终点，格式为[x, y]
    :return: 最短距离（浮点数）
    """
    # 将点转换为向量
    ax, ay = a
    bx, by = b
    px, py = p

    # 计算向量AP和AB
    ap_x = px - ax
    ap_y = py - ay
    ab_x = bx - ax
    ab_y = by - ay

    # 计算投影参数t（归一化后的投影长度）
    dot_product = ap_x * ab_x + ap_y * ab_y
    ab_len_sq = ab_x ** 2 + ab_y ** 2

    if ab_len_sq == 0:  # 线段退化为点
        return math.hypot(px - ax, py - ay)

    t = max(0, min(1, dot_product / ab_len_sq))

    # 计算投影点坐标
    proj_x = ax + t * ab_x
    proj_y = ay + t * ab_y

    # 返回点p到投影点的距离
    return math.hypot(px - proj_x, py - proj_y)

# 测试大图导航时名字后缀
# 像素坐标用于导航
# 基于基站的米坐标转化为像素坐标(1299*958) 输入输出都是列表
def meter2pixel_x(x):
    # 加上偏置
    x += offset_x
    # 变成像素
    x = x / pixel_to_meter_m
    return x
def meter2pixel_y(y):
    y += offset_y
    y = y / pixel_to_meter_m
    return y

# 米坐标用于路径跟踪
# 像素坐标(1299*958)转化为基于基站的米坐标 输入输出都是列表
def pixel2meter_x(x):
    x = x * pixel_to_meter_m
    x = x - offset_x
    return x
def pixel2meter_y(y):
    y = y * pixel_to_meter_m
    y = y - offset_y
    return y

def path2map(path):
    # 需要从导航像素变成基于map的米坐标
    path_list = []
    for i in range(len(path)):
        path_list.append([pixel2meter_x(path[i][0])+offset_x,pixel2meter_y(path[i][1])+offset_y])

    return path_list


# 使用示例 
def main():
    # 1.停车场信息初始化
    parking_lot = ParkingLot(14)
    # print(parking_lot.spots_free[1].coordinate)
    # 2.随机占用车辆
    parking_lot.random_occupy()
    parking_lot.check_occupy()
    # 3.随机选择一个空余车位，并返回车位id与二维坐标
    id, xy = parking_lot.random_choose()
    print(id,xy,xy[0])
    print(parking_lot.check_occupy())
    
if __name__ == '__main__':
    main()