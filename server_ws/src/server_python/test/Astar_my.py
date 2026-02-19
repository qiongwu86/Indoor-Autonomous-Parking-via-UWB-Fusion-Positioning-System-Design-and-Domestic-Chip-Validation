"""

定义了A*类，做了1、优先向目标点寻找；2、优先直行的优化


"""

import math
import numpy as np
import matplotlib.pyplot as plt
import time
import scipy.interpolate as scipy_interpolate
show_animation = True


class AStarPlanner:

    def __init__(self, ox, oy, resolution, rr):

        self.resolution = resolution  # grid resolution [m]
        self.rr = rr  # robot radius [m]
        self.calc_obstacle_map(ox, oy)
        self.motion = self.get_motion_model()

    # 创建一个节点类，节点的信息包括：xy坐标，cost代价,parent_index
    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # index of grid
            self.y = y
            self.cost = cost  # g(n)
            self.parent_index = parent_index  # index of previous Node

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)

    # 经过planning函数处理，传入sx,sy,gx,gy, 返回pathx,pathy(最终的路径)
    def planning(self, sx, sy, gx, gy):
        """
        1、sx = nstart  sy = ngoal
        2、open_set  closed_set
        3、open_set = nstart
        4、将open表中代价最小的子节点=当前节点，并在plot上动态显示，按esc退出
        5、如果当前节点等于ngoal，提示找到目标点
        6、删除open表中的内容，添加到closed表中
        7、基于运动模型定义搜索方式
        8、pathx,pathy = 最终路径(传入ngoal,closed_set)，返回pathx,pathy
        """

        # 1、sx = nstart  sy = ngoal  初始化nstart、ngoal坐标作为一个节点，传入节点全部信息
        nstart = self.Node(self.calc_xyindex(sx, self.minx),  # position min_pos   2 (2.5)
                           self.calc_xyindex(sy, self.miny),  # 2 (2.5)
                           0.0,
                           -1)
        ngoal = self.Node(self.calc_xyindex(gx, self.minx),
                          self.calc_xyindex(gy, self.miny),
                          0.0,
                          -1)
        # 2、open表、closed表设定为字典
        # 3、起点加入open表
        open_set, closed_set = dict(), dict()  # key - value: hash表
        open_set[self.calc_grid_index(nstart)] = nstart

        while 1:
            if len(open_set) == 0:
                print("Open_set is empty...")
                break

            # 4、将open表中代价最小的子节点 = 当前节点，并在plot上动态显示，按esc退出

            # f(n)=g(n)+h(n)  实际代价+预估代价
            c_id = min(open_set, key=lambda o: open_set[o].cost + self.calc_heuristic(ngoal, open_set[o]))
            current = open_set[c_id]

            # 将当前节点显示出来
            if show_animation:
                plt.plot(self.calc_grid_position(current.x, self.minx),
                         self.calc_grid_position(current.y, self.miny),
                         "xc")  # 青色x 搜索点
                # 按esc退出
                plt.gcf().canvas.mpl_connect('key_release_event',
                                             lambda event: [exit(0) if event.key == 'escape' else None]
                                             )
                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)

            if current.x == ngoal.x and current.y == ngoal.y:
                print("Find goal!")
                ngoal.parent_index = current.parent_index
                print("ngoal_parent_index:", ngoal.parent_index)
                ngoal.cost = current.cost
                print("ngoal_cost:", ngoal.cost)
                break

            # 删除open表中的c_id的子节点,并把current添加到closed_set
            del open_set[c_id]
            closed_set[c_id] = current

            if current.parent_index == -1:
                node_parent = current
            else:
                node_parent = closed_set[current.parent_index]             # 当前节点的父节点

            # 基于motion model做栅格扩展，也就是搜索方式，可进行改进，如使用双向搜索、JPS等
            for move_x, move_y, move_cost in self.motion:
                node = self.Node(current.x + move_x,  # 当前x+motion列表中第0个元素dx
                                 current.y + move_y,
                                 current.cost + move_cost + self.calc_extracost(node_parent, current, move_x, move_y, ngoal),
                                 c_id)
                n_id = self.calc_grid_index(node)  # 返回该节点位置index

                # 如果节点不可通过，跳过
                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # 直接加入a new node
                else:
                    if open_set[n_id].cost > node.cost:         # 如果表中的节点成本 大于 当前节点成本 ， 则更新为现在的成本
                        open_set[n_id] = node  # This path is the best until now. record it

        pathx, pathy = self.calc_final_path(ngoal, closed_set)
        pathx.reverse()
        pathy.reverse()
        return pathx, pathy

    def calc_final_path(self, ngoal, closedset):  # 传入目标点和closed表，经过函数处理得到最终所有的xy列表
        pathx, pathy = [self.calc_grid_position(ngoal.x, self.minx)], [
            self.calc_grid_position(ngoal.y, self.miny)]
        parent_index = ngoal.parent_index
        while parent_index != -1:
            n = closedset[parent_index]
            pathx.append(self.calc_grid_position(n.x, self.minx))
            pathy.append(self.calc_grid_position(n.y, self.miny))
            parent_index = n.parent_index

        return pathx, pathy

    @staticmethod  # 静态方法，calc_heuristic函数不用传入self，因为要经常修改启发函数，目的是为了提高阅读性
    def calc_heuristic(n1, n2):  # n1: ngoal，n2: open_set[o]
        h = math.hypot(n1.x - n2.x, n1.y - n2.y)  # 欧几里得距离
        # h =  max(n1.x-n2.x,n1.y-n2.y)#切比雪夫距离

        if h > 18:
            h *= 3.

        return h

    @staticmethod  # 静态方法，calc_heuristic函数不用传入self，因为要经常修改启发函数，目的是为了提高阅读性
    def calc_extracost(node_parent, current, move_x, move_y, ngoal):  # n1: ngoal，n2: open_set[o]
        if ((current.x + move_x == node_parent.x and current.x == node_parent.x)
                or (current.y + move_y == node_parent.y and current.y == node_parent.y)
                or node_parent.parent_index == -1):
            cost = 0
        elif current.x + move_x == ngoal.x or current.y + move_y == ngoal.y:
            cost = 1
            # cost = 0

        else:
            cost = 3.0
            # cost = 0

        return cost


    # 得到全局地图中的具体坐标: 传入地图中最小处障碍物的pos和index
    def calc_grid_position(self, index, minpos):
        pos = index * self.resolution + minpos
        return pos

    # 位置转化为以栅格大小为单位的索引: 传入position,min_pos
    def calc_xyindex(self, position, min_pos):
        return round((position - min_pos) / self.resolution)  # (当前节点-最小处的坐标)/分辨率=pos_index  round四舍五入向下取整

    # 计算栅格地图节点的index： 传入某个节点
    def calc_grid_index(self, node):
        return (node.y - self.miny) * self.xwidth + (node.x - self.minx)

        # 验证是否为可通行节点

    def verify_node(self, node):
        posx = self.calc_grid_position(node.x, self.minx)
        posy = self.calc_grid_position(node.y, self.miny)

        if posx < self.minx:
            return False
        elif posy < self.miny:
            return False
        elif posx >= self.maxx:
            return False
        elif posy >= self.maxy:
            return False

        if self.obmap[int(node.x)][int(node.y)]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy):
        self.minx = round(min(ox))  # 地图中的临界值 -10
        self.miny = round(min(oy))  # -10
        self.maxx = round(max(ox))  # 60
        self.maxy = round(max(oy))  # 60
        print("minx:", self.minx)
        print("miny:", self.miny)
        print("maxx:", self.maxx)
        print("maxy:", self.maxy)

        self.xwidth = round((self.maxx - self.minx) / self.resolution)  # 35
        self.ywidth = round((self.maxy - self.miny) / self.resolution)  # 35
        print("xwidth:", self.xwidth)
        print("ywidth:", self.ywidth)

        self.obmap = [[False for i in range(int(self.ywidth))]
                      for i in range(int(self.xwidth))]
        for ix in range(int(self.xwidth)):
            x = self.calc_grid_position(ix, self.minx)
            for iy in range(int(self.ywidth)):
                y = self.calc_grid_position(iy, self.miny)
                for iox, ioy in zip(ox, oy):  # 将ox,oy打包成元组，返回列表，并遍历
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.rr:  # 代价小于车辆半径，可正常通过，不会穿越障碍物
                        self.obmap[ix][iy] = True
                        break

    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [
            [1, 0, 1],
            [0, 1, 1],
            [-1, 0, 1],
            [0, -1, 1],
            [1, 1, math.sqrt(2)],
            [1, -1, math.sqrt(2)],
            [-1, 1, math.sqrt(2)],
            [-1, -1, math.sqrt(2)]
        ]
        # motion = [
        #     [1, 0, 1],
        #     [0, 1, 1],
        #     [-1, 0, 1],
        #     [0, -1, 1],
        #     # [1, 1, math.sqrt(2)],
        #     # [1, -1, math.sqrt(2)],
        #     # [-1, 1, math.sqrt(2)],
        #     # [-1, -1, math.sqrt(2)]
        # ]

        return motion

###############################
def interpolate_b_spline_path(x, y, n_path_points, degree=3):
    """
    实际上它使用的是 scipy 提供的一元样条插值方法 make_interp_spline
    这种方法在内部通过指定的度数（k 或 degree）构建一个样条曲线，然后可以使用该曲线在任何所需的点上计算插值结果
    :param x:       x 和 y 是路径点的 x 和 y 坐标列表；
    :param y:
    :param n_path_points:       插值后新路径的点数
    :param degree:              插值的度数
    :return:
    """
    ipl_t = np.linspace(0.0, len(x) - 1, len(x))
    # 使用 scipy 的插值函数 make_interp_spline 创建一个一元样条插值器 spl_i_x，spl_i_y，用于 x ， y 坐标
    spl_i_x = scipy_interpolate.make_interp_spline(ipl_t, x, k=degree)
    spl_i_y = scipy_interpolate.make_interp_spline(ipl_t, y, k=degree)

    travel = np.linspace(0.0, len(x) - 1, n_path_points)  # 改变长度为n_path_points
    return spl_i_x(travel), spl_i_y(travel)
# 样条插值
def interpolate_path(path, sample_rate):
    """
    对给定的路径（path）进行采样，并使用B样条插值生成一个新的、更平滑的路径（new_path）。
    通过调整sample_rate和插值点的数量（n_course_point），可以控制采样的密度和插值结果的平滑度。
    :param path:
    :param sample_rate: 每隔多少个点选取一个点进行插值。
    :return:
    """
    choices = np.arange(0, len(path), sample_rate)
    if len(path) - 1 not in choices:
        choices = np.append(choices, len(path) - 1)  # 确保最后一个点被选中
    way_point_x = path[choices, 0]  # 获取这些行的第一列（即x坐标）
    way_point_y = path[choices, 1]  # 获取这些行的第二列（即y坐标）
    n_course_point = len(path) * 3  # 计算新路径的点数，这里简单地将其设置为原路径长度的三倍
    rix, riy = interpolate_b_spline_path(way_point_x, way_point_y, n_course_point)  # 使用选定的x和y坐标以及新路径的点数进行B样条插值
    new_path = np.vstack([rix, riy]).T
    # new_path[new_path<0] = 0
    return new_path
###############################
# 贝塞尔
"""
计算路径的弯曲程度，先采样，再计算三个点之间的角度差，
根据角度差，设置弯曲程度阈值 45
使用贝塞尔曲线对角点处曲线平滑

"""

def bezier(Ps,n,t):
    """递归的方式实现贝塞尔曲线

    Args:
        Ps (_type_): 控制点，格式为numpy数组：array([[x1,y1],[x2,y2],...,[xn,yn]])
        n (_type_): n个控制点，即Ps的第一维度
        t (_type_): 步长t

    Returns:
        _type_: 当前t时刻的贝塞尔点
    """
    """
    # 使用例子
        # path1 = pathplanning.interpolate_path(interval_data[id-front:id+back], 5)   # 插值
        Ps = interval_data[id-front:id+back]    # 贝塞尔控制点 前4+自己+后4 = 9 个点（距离足够的情况）
        path1 = []                              # 路径点存储
        # 贝塞尔曲线生成
        for t in np.arange(0, 1.2, 0.2):
            p_t = bezier(Ps, len(Ps), t)
            path1.append(p_t)
        path1 = np.array(path1)

    """
    if n==1:
        return Ps[0]
    return (1-t)*bezier(Ps[0:n-1],n-1,t)+t*bezier(Ps[1:n],n-1,t)

"""
输入 规划好的路径点，
输出 贝塞尔曲线平滑后的路径点，
对直角点迹平滑处理
"""
def path_by_bezier(arr):
    # interval_data = arr[::3]            # 采样间隔为3
    interval_data = arr[::1]            # 都采样，数据点

    # plath_test.show_polt(interval_data,1)

    theta_arr = np.zeros((len(interval_data)-2,), dtype=float)            # 设初值
    x_arr = interval_data[:,0]          # 取出x
    y_arr = interval_data[:,1]          # 取出y

    # 计算每三个点的弯曲程度
    for i in range(len(interval_data)-2):
        # 计算第一个线段与 x 轴正方向之间的角度
        angle1 = np.arctan2(y_arr[i + 1] - y_arr[i], x_arr[i + 1] - x_arr[i])

        # 计算第二个线段与 x 轴正方向之间的角度
        angle2 = np.arctan2(y_arr[i + 2] - y_arr[i + 1], x_arr[i + 2] - x_arr[i + 1])

        # 计算夹角，确保结果在 -pi 到 pi 之间
        theta_arr[i] = (angle1 - angle2) % (2 * np.pi)
        if theta_arr[i] < -np.pi:
            theta_arr[i] += 2 * np.pi  # 处理负溢出
        elif theta_arr[i] > np.pi:
            theta_arr[i] -= 2 * np.pi  # 处理正溢出
        theta_arr[i] = np.rad2deg(theta_arr[i])

    # print(theta_arr)            # 记录每个点与后两个点之间的弯曲程度
    theta_arr_len = len(theta_arr)

    # # 显示  在数据点上添加弯曲度数
    # labels = [str(val) for val in theta_arr]
    # plt.scatter(x_arr, y_arr)
    # # 在每个数据点上添加数据标签
    # for i in range(theta_arr_len):
    #     plt.text(x_arr[i], y_arr[i], labels[i], ha='right', va='bottom')
    #
    # print("theta_arr_len", theta_arr_len)
    # # 显示图表
    # plt.show()

    indices = np.where(theta_arr.__abs__() > 20)            # 找出弯曲程度大于40的路径点

    for i in range(len(indices[0])):                        # 对所有找出的路径点进行判断
        # print(indices[0][i])
        id = indices[0][i]
        if i == 0:                                          # 第一个大曲率点
            if id > 3:                                      # 前面有足够的距离
                front = 3
            else:
                front = id                                  # 前宽度等于前距离
            if theta_arr_len - id > 5:                      # 末尾有足够的距离
                back = 5
            else:
                back = theta_arr_len - id                   # 后宽度等于后距离
            # path1 = pathplanning.interpolate_path(interval_data[id-front:id+back], 5)   # 插值
            Ps = interval_data[id-front:id+back]    # 贝塞尔控制点 前3+自己+后5 = 9 个点（距离足够的情况）
            # Ps = interval_data[id - front:id + back:2]      # 调拼接岔路口的时候改
            path1 = []                              # 路径点存储
            # 贝塞尔曲线生成
            for t in np.arange(0, 1.2, 0.2):
                p_t = bezier(Ps, len(Ps), t)
                path1.append(p_t)
            path1 = np.array(path1)

            path = np.vstack([interval_data[:id-front], path1])
            path_id_old = id + back
        id_before = indices[0][i-1]
        id = indices[0][i]
        # print(id_before)
        # print(id)
        if id - id_before > 5:                              # 相距超过5，对该路径中改id前5~后5个点进行曲线插值
            if theta_arr_len - id > 5:                      # 末尾有足够的点数
                back = 5
            else:
                back = theta_arr_len - id
                is_last = 1
            # path1 = pathplanning.interpolate_path(interval_data[id - 10:id + back], 5)  # 插值
            Ps = interval_data[id-3:id+back]    # 贝塞尔控制点   id-3     id      id+back
            path1 = []                              # 路径点存储
            # 贝塞尔曲线生成
            for t in np.arange(0, 1.2, 0.2):
                p_t = bezier(Ps, len(Ps), t)
                path1.append(p_t)
            path1 = np.array(path1)                 # 贝塞尔优化

            path = np.vstack([path,                                     # 之前的路径点
                              # pathplanning.interpolate_path(interval_data[path_id_old:id - 10], 2),
                              interval_data[path_id_old:id - 3],        # 无需优化的路径点，从上一个角点的优化截止点到当前角点的优化起始点
                              path1                                     # 贝塞尔优化后的路径点
                              ])
            path_id_old = id + back                                     # 更新一个角点的优化截止点
        else:
            pass
        if i == len(indices[0])-1:              # 末尾角点
            path = np.vstack([path,             # 补全路径
                              interval_data[path_id_old:],
                              ])

    path = interpolate_path(path, 4)    # 进行样条插值
    # plath_test.show_polt(path, 1)                            # 显示更新后的路径点

    return path
################################
# 旋转路径
def rotate_path(path, angle=0.):
    """
    旋转路径angle度(弧度)
    Args:
        pts:        路径点,np数组
        angle:      旋转角度(弧度)

    Returns:
    旋转后的路径
    """
    path = np.array(path)       # 转化为np数组
    ddd = path[0]               # 初始点的偏移量
    path = path - ddd           # 减去初始点的偏移量

    # 旋转矩阵
    R = np.array([[np.cos(angle), -np.sin(angle)],
                [np.sin(angle),  np.cos(angle)]])

    path = (R @ path.T).T       # 旋转path
    path = path + ddd           # 加上偏移量，回到初始点

    return path

def flip_path_x(path):
    """
    路径点关于x轴对称
    Args:
        path:

    Returns:
    """
    path = np.array(path)
    y0 = path[0][1]            # 第一个数据点的y坐标到y轴的距离
    path = path - [0,y0]          # 平移数组
    # print(path)
    # a_change_list = np.array([])
    for i in range(len(path)):
        # path[i][0] = path[i][0]       # x
        path[i][1] =  -path[i][1]       # y
        # print(a[i][1])
    # print(path)
    path = path - [0,y0]
    # plath_test.show_polt(path,1)

    return path

def flip_path_y(path):
    """
    路径点关于y轴对称
    Args:
        path:

    Returns:
    """
    path = np.array(path)
    x0 = path[0][0]            # 第一个数据点的x坐标到x轴的距离
    path = path - [x0,0]          # 平移数组
    # print(path)
    # a_change_list = np.array([])
    for i in range(len(path)):
        path[i][0] = -path[i][0]
        path[i][1] = path[i][1]
        # print(a[i][1])
    # print(path)
    path = path - [x0,0]
    # plath_test.show_polt(path,1)

    return path
################################
# 生成指令
def generate_path_instructions(path_points, pixel_to_meter=0.0564):
    if not path_points or len(path_points) < 2:
        return "路径点不足，无法生成指令"

    instructions = []
    current_direction = None
    current_distance = 0.0

    for i in range(len(path_points) - 1):
        x1, y1 = path_points[i]
        x2, y2 = path_points[i + 1]

        dx = x2 - x1
        dy = y2 - y1

        distance_pixels = math.hypot(dx, dy)
        distance_meters = distance_pixels * pixel_to_meter

        current_angle = math.atan2(dy, dx)

        if current_direction is None:
            current_direction = current_angle
            current_distance += distance_meters
            continue

        delta_angle = current_angle - current_direction

        if abs(delta_angle) > math.radians(45):  # 超过45度视为转弯
            # 将弧度转换为角度
            delta_angle_deg = math.degrees(delta_angle)

            # 记录当前直行距离
            instructions.append(f" 直行{current_distance:.2f}米")

            # 判断转弯方向
            if delta_angle_deg > 0:
                instructions.append(" 右转")
            else:
                instructions.append(" 左转")

            # 更新方向和距离
            current_direction = current_angle
            current_distance = distance_meters
        else:
            current_distance += distance_meters

            # 处理最后一段直行
    instructions.append(f" 直行{current_distance:.2f}米")

    return '，'.join(instructions)

def main():
    """
    测试用,画图
    :return:
    """
    print(__file__ + '  start!')
    plt.title(" Original Astar")

    # start and goal position  [m]
    sx = -5.0
    sy = -5.0
    gx = 50
    gy = 50
    grid_size = 2.0
    robot_radius = 1.0

    # obstacle positions
    ox, oy = [], []
    for i in range(-10, 60):
        ox.append(i)
        oy.append(-10)  # y坐标-10的一行-10~60的坐标添加到列表并显示为黑色障碍物
    for i in range(-10, 60):
        ox.append(i)
        oy.append(60)  # y坐标60的一行-10~60的坐标添加到列表并显示为黑色障碍物
    for i in range(-10, 61):
        ox.append(-10)
        oy.append(i)  # x坐标-10的一列-10~61的坐标添加到列表并显示为黑色障碍物
    for i in range(-10, 61):
        ox.append(60)
        oy.append(i)  # x坐标60的一列-10~61的坐标添加到列表并显示为黑色障碍物
    for i in range(-10, 40):
        ox.append(20)
        oy.append(i)  # x坐标20的一列-10~40的坐标添加到列表并显示为黑色障碍物
    for i in range(0, 40):
        ox.append(40)
        oy.append(60 - i)  # x坐标40的一列20~60的坐标添加到列表并显示为黑色障碍物

    if show_animation:
        plt.plot(ox, oy, ".k")  # 黑色.       障碍物
        plt.plot(sx, sy, "og")  # 绿色圆圈    开始坐标
        plt.plot(gx, gy, "xb")  # 蓝色x       目标点
        plt.grid(True)
        plt.axis('equal')  # 保持栅格的横纵坐标刻度一致

    # 路径计算，并计时

    a_star = AStarPlanner(ox, oy, grid_size, robot_radius)  # grid_size=resolution 初始化中传入的参数

    # 图形化显示
    start = time.perf_counter()
    pathx, pathy = a_star.planning(sx, sy, gx, gy)  # 开始与结束的坐标传入函数进行处理后，得到pathx,pathy：最终规划出的路径坐标
    end = time.perf_counter()
    print('用时' + str(1000 * (end - start)) + 'ms')

    if show_animation:
        plt.plot(pathx, pathy, "-r")  # 红色直线 最终路径
        plt.show()
        plt.pause(0.001)  # 动态显示


if __name__ == '__main__':
    main()
