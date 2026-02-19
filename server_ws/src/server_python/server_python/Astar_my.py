"""

定义了A*类，做了1、优先向目标点寻找；2、优先直行的优化
包含样条插值/贝塞尔平滑/旋转路径/翻转路径/根据路径生成指令/根据指令生成路径 函数

"""

import math
import numpy as np
import matplotlib.pyplot as plt
import time
import scipy.interpolate as scipy_interpolate
import heapq
import cv2

from fpdf import FPDF
from PIL import Image
import os

show_animation = False


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
        1、sx = nstart  gy = ngoal
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
        distance = math.sqrt((ngoal.x  - nstart.x) ** 2 + (ngoal.y - nstart.y) ** 2)      # 计算起终点中的距离 传入代价函数
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
            c_id = min(open_set, key=lambda o: open_set[o].cost + self.calc_heuristic(ngoal, open_set[o],distance))
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
                # print("Find goal!")
                ngoal.parent_index = current.parent_index
                # print("ngoal_parent_index:", ngoal.parent_index)
                ngoal.cost = current.cost
                # print("ngoal_cost:", ngoal.cost)
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
    def calc_heuristic(n1, n2, distance):  # n1: ngoal，n2: open_set[o]
        h = math.hypot(n1.x - n2.x, n1.y - n2.y)  # 欧几里得距离
        # h =  max(n1.x-n2.x,n1.y-n2.y)#切比雪夫距离

        if h > distance / 3.0:              # 距离超过1/3时，优先前往终点
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
        # print("minx:", self.minx)
        # print("miny:", self.miny)
        # print("maxx:", self.maxx)
        # print("maxy:", self.maxy)

        self.xwidth = round((self.maxx - self.minx) / self.resolution)  # 35
        self.ywidth = round((self.maxy - self.miny) / self.resolution)  # 35
        # print("xwidth:", self.xwidth)
        # print("ywidth:", self.ywidth)

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

        return motion

class AStarPlanner2:

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
        1、sx = nstart  gy = ngoal
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

        if not self.verify_node(nstart):
            print("Error: Start position is in obstacle or out of bounds!")
            return [], []

        distance = math.sqrt((ngoal.x  - nstart.x) ** 2 + (ngoal.y - nstart.y) ** 2)      # 计算起终点中的距离 传入代价函数
        # 2、open表、closed表设定为字典
        # 3、起点加入open表
        open_set, closed_set = dict(), dict()  # key - value: hash表
        open_set[self.calc_grid_index(nstart)] = nstart

        open_heap = []  # 优先队列 (f, node_id)     # new
        # 起点入队
        start_id = self.calc_grid_index(nstart)
        open_set[start_id] = nstart
        heapq.heappush(
            open_heap,
            (nstart.cost + self.calc_heuristic(ngoal, nstart, distance), start_id)
        )

        explored_x = []  # 记录搜索过的x坐标
        explored_y = []  # 记录搜索过的y坐标

        while 1:
            if len(open_set) == 0:
                print("Open_set is empty...")
                break

            # 4、将open表中代价最小的子节点 = 当前节点，并在plot上动态显示，按esc退出

            ######          # f(n)=g(n)+h(n)  实际代价+预估代价    # 原
            # c_id = min(open_set, key=lambda o: open_set[o].cost + self.calc_heuristic(ngoal, open_set[o],distance))
            # current = open_set[c_id]
            ######

            _, c_id = heapq.heappop(open_heap)      # 取出代价最小节点      # new
            if c_id not in open_set:
                continue  # 已被更优路径或目标切换淘汰
            current = open_set[c_id]

            # 将当前节点坐标加入explored列表
            explored_x.append(self.calc_grid_position(current.x, self.minx))
            explored_y.append(self.calc_grid_position(current.y, self.miny))

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
                # print("Find goal!")
                ngoal.parent_index = current.parent_index
                # print("ngoal_parent_index:", ngoal.parent_index)
                ngoal.cost = current.cost
                # print("ngoal_cost:", ngoal.cost)
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

                ######          # 原
                # if n_id not in open_set:
                #     open_set[n_id] = node  # 直接加入a new node
                # else:
                #     if open_set[n_id].cost > node.cost:         # 如果表中的节点成本 大于 当前节点成本 ， 则更新为现在的成本
                #         open_set[n_id] = node  # This path is the best until now. record it

                # new               # 优先队列，速度提升 0.019-》0.0068s
                if n_id not in open_set or open_set[n_id].cost > node.cost:
                    open_set[n_id] = node
                    f = node.cost + self.calc_heuristic(ngoal, node, distance)
                    heapq.heappush(open_heap, (f, n_id))

        pathx, pathy = self.calc_final_path(ngoal, closed_set)
        pathx.reverse()
        pathy.reverse()
        return pathx, pathy, explored_x, explored_y

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
    def calc_heuristic(n1, n2, distance):  # n1: ngoal，n2: open_set[o]
        h = math.hypot(n1.x - n2.x, n1.y - n2.y)  # 欧几里得距离
        # h =  max(n1.x-n2.x,n1.y-n2.y)#切比雪夫距离

        if h > distance / 3.0:              # 距离超过1/3时，优先前往终点
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
        # print("minx:", self.minx)
        # print("miny:", self.miny)
        # print("maxx:", self.maxx)
        # print("maxy:", self.maxy)

        self.xwidth = round((self.maxx - self.minx) / self.resolution)  # 35
        self.ywidth = round((self.maxy - self.miny) / self.resolution)  # 35
        # print("xwidth:", self.xwidth)
        # print("ywidth:", self.ywidth)

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
    n_course_point = int(len(path) * 3.5)  # 计算新路径的点数，这里简单地将其设置为原路径长度的4倍 # TODO:8-6 *4-》*3
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

# 初始
def path_by_bezier_0(arr):
    """
    输入 规划好的路径点，
    输出 贝塞尔曲线平滑后的路径点，
    对直角点迹平滑处理
    """
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

    indices = np.where(theta_arr.__abs__() > 20)            # 找出弯曲程度大于20的路径点
    if len(indices[0]) <= 0:
        path = np.array(interval_data)
    else:
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
                print(f'原始长度:{len(interval_data[id-front:id+back])}')
                
                # Ps = interval_data[id - front:id + back:2]      # 调拼接岔路口的时候改
                path1 = []                              # 路径点存储
                # 贝塞尔曲线生成
                for t in np.arange(0, 1.2, 0.2):                        # 6个点
                    p_t = bezier(Ps, len(Ps), t)
                    path1.append(p_t)
                print(f'更新后长度:{len(path1)}')
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
                print(f'原始长度:{len(interval_data[id-3:id+back])}')

                path1 = []                              # 路径点存储
                # 贝塞尔曲线生成
                for t in np.arange(0, 1.2, 0.2):                    # 6个点
                    p_t = bezier(Ps, len(Ps), t)
                    path1.append(p_t)
                print(f'更新后长度:{len(path1)}')

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

    path = interpolate_path(path, 1)    # 进行样条插值 2
    # plath_test.show_polt(path, 1)                            # 显示更新后的路径点

    return path

# 修改的优化 测试记得该名字 所使用名称 path_by_bezier
def path_by_bezier(arr):
    """
    修改的优化
    """
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

    indices = np.where(theta_arr.__abs__() > 30)            # 找出弯曲程度大于60的路径点 TODO:EI测试时改成了40
    if len(indices[0]) <= 0:
        path = np.array(interval_data)
    else:
        for i in range(len(indices[0])):                        # 对所有找出的路径点进行判断
            # print(indices[0][i])
            id = indices[0][i]
            if i == 0:                                          # 第一个大曲率点
                if id > 10:                                      # 前面有足够的距离
                    front = 10
                else:
                    front = id                                  # 前宽度等于前距离
                if theta_arr_len - id > 10:                      # 末尾有足够的距离
                    back = 10
                else:
                    back = theta_arr_len - id                   # 后宽度等于后距离
                # path1 = pathplanning.interpolate_path(interval_data[id-front:id+back], 5)   # 插值

                Ps = interval_data[id-front:id+back:4]    # 贝塞尔控制点 前3+自己+后5 = 9 个点（距离足够的情况）
                ps_1 = interval_data[id-front:id+back][-1]                          # 记录最后一个控制点
                # print(f'原始长度:{len(interval_data[id-front:id+back])}')

                if (Ps[-1] == ps_1).all():              # 两个相等
                    pass
                else:
                   Ps = np.vstack([Ps,ps_1])            # 添加最后一个控制点

                # Ps = interval_data[id - front:id + back:2]      # 调拼接岔路口的时候改
                path1 = []                              # 路径点存储
                # 贝塞尔曲线生成
                for t in np.arange(0, 1.0, 1.0/(front+back-1)):
                    # print('1111')               
                    # print(t)               
                    p_t = bezier(Ps, len(Ps), t)
                    path1.append(p_t)

                p_t = bezier(Ps, len(Ps), 1.0)
                path1.append(p_t)

                path1 = np.array(path1)
                # print(f'更新后长度:{len(path1)}')
                path = np.vstack([interval_data[:id-front], path1])
                path_id_old = id + back
            id_before = indices[0][i-1]
            id = indices[0][i]
            # print(id_before)
            # print(id)
            if id - id_before > 10:                              # 相距超过5，对该路径中改id前5~后5个点进行曲线插值
                if theta_arr_len - id > 10:                      # 末尾有足够的点数
                    back = 10
                else:
                    back = theta_arr_len - id
                    is_last = 1
                # path1 = pathplanning.interpolate_path(interval_data[id - 10:id + back], 5)  # 插值
                Ps = interval_data[id-10:id+back:4]    # 贝塞尔控制点   id-3     id      id+back
                # print('Ps：',Ps)
                # print(f'原始长度:{len(interval_data[id-10:id+back])}')

                ps_1 = interval_data[id-10:id+back][-1]                          # 记录最后一个控制点

                if (Ps[-1] == ps_1).all():              # 两个相等
                    pass
                else:
                   Ps = np.vstack([Ps,ps_1])            # 添加最后一个执行点

                path1 = []                              # 路径点存储
                # 贝塞尔曲线生成
                for t in np.arange(0, 1.0, 1.0/(10+back-1)):                    # 6个点
                    # print('2222')               
                    # print(t)          
                    p_t = bezier(Ps, len(Ps), t)
                    path1.append(p_t)

                p_t = bezier(Ps, len(Ps), 1.0)
                path1.append(p_t)
                # print(f'更新后长度:{len(path1)}')

                path1 = np.array(path1)                 # 贝塞尔优化

                path = np.vstack([path,                                     # 之前的路径点
                                # pathplanning.interpolate_path(interval_data[path_id_old:id - 10], 2),
                                interval_data[path_id_old:id - 10],        # 无需优化的路径点，从上一个角点的优化截止点到当前角点的优化起始点
                                path1                                     # 贝塞尔优化后的路径点
                                ])
                path_id_old = id + back                                     # 更新一个角点的优化截止点
            else:
                pass
            if i == len(indices[0])-1:              # 末尾角点
                path = np.vstack([path,             # 补全路径
                                interval_data[path_id_old:],
                                ])

    # path = interpolate_path(path, 1)    # 进行样条插值 2
    # plath_test.show_polt(path, 1)                            # 显示更新后的路径点

    return path, indices                                        # TODO:测试时多传了indices测试

def path_by_bezier_2(arr):
    """
    二次优化，第一次不插值，第二次插值  EI
    """
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

    indices = np.where(theta_arr.__abs__() > 30)            # 找出弯曲程度大于60的路径点 TODO:EI测试时改成了40
    if len(indices[0]) <= 0:
        path = np.array(interval_data)
    else:
        for i in range(len(indices[0])):                        # 对所有找出的路径点进行判断
            # print(indices[0][i])
            id = indices[0][i]
            if i == 0:                                          # 第一个大曲率点
                if id > 10:                                      # 前面有足够的距离
                    front = 10
                else:
                    front = id                                  # 前宽度等于前距离
                if theta_arr_len - id > 10:                      # 末尾有足够的距离
                    back = 10
                else:
                    back = theta_arr_len - id                   # 后宽度等于后距离
                # path1 = pathplanning.interpolate_path(interval_data[id-front:id+back], 5)   # 插值

                Ps = interval_data[id-front:id+back:4]    # 贝塞尔控制点 前3+自己+后5 = 9 个点（距离足够的情况）
                ps_1 = interval_data[id-front:id+back][-1]                          # 记录最后一个控制点
                # print(f'原始长度:{len(interval_data[id-front:id+back])}')

                if (Ps[-1] == ps_1).all():              # 两个相等
                    pass
                else:
                   Ps = np.vstack([Ps,ps_1])            # 添加最后一个控制点

                # Ps = interval_data[id - front:id + back:2]      # 调拼接岔路口的时候改
                path1 = []                              # 路径点存储
                # 贝塞尔曲线生成
                for t in np.arange(0, 1.0, 1.0/(front+back-1)):         # 第二次的话布长再大点
                # for t in np.arange(0, 1.0, 1.0/(front+back-1-back)):         # 第二次的话布长再大点 TODO:
                    # print('1111')               
                    # print(t)               
                    p_t = bezier(Ps, len(Ps), t)
                    path1.append(p_t)

                p_t = bezier(Ps, len(Ps), 1.0)
                path1.append(p_t)

                path1 = np.array(path1)
                # print(f'更新后长度:{len(path1)}')
                path = np.vstack([interval_data[:id-front], path1])
                path_id_old = id + back
            id_before = indices[0][i-1]
            id = indices[0][i]
            # print(id_before)
            # print(id)
            if id - id_before > 10:                              # 相距超过5，对该路径中改id前5~后5个点进行曲线插值
                if theta_arr_len - id > 10:                      # 末尾有足够的点数
                    back = 10
                else:
                    back = theta_arr_len - id
                    is_last = 1
                # path1 = pathplanning.interpolate_path(interval_data[id - 10:id + back], 5)  # 插值
                Ps = interval_data[id-10:id+back:4]    # 贝塞尔控制点   id-3     id      id+back
                # print('Ps：',Ps)
                # print(f'原始长度:{len(interval_data[id-10:id+back])}')

                ps_1 = interval_data[id-10:id+back][-1]                          # 记录最后一个控制点

                if (Ps[-1] == ps_1).all():              # 两个相等
                    pass
                else:
                   Ps = np.vstack([Ps,ps_1])            # 添加最后一个执行点

                path1 = []                              # 路径点存储
                # 贝塞尔曲线生成
                for t in np.arange(0, 1.0, 1.0/(10+back-1)):                    # 6个点
                # for t in np.arange(0, 1.0, 1.0/(10+back-1- back)):                    # 6个点  第二次的话布长再大点 TODO:
                    # print('2222')               
                    # print(t)          
                    p_t = bezier(Ps, len(Ps), t)
                    path1.append(p_t)

                p_t = bezier(Ps, len(Ps), 1.0)
                path1.append(p_t)
                # print(f'更新后长度:{len(path1)}')

                path1 = np.array(path1)                 # 贝塞尔优化

                path = np.vstack([path,                                     # 之前的路径点
                                # pathplanning.interpolate_path(interval_data[path_id_old:id - 10], 2),
                                interval_data[path_id_old:id - 10],        # 无需优化的路径点，从上一个角点的优化截止点到当前角点的优化起始点
                                path1                                     # 贝塞尔优化后的路径点
                                ])
                path_id_old = id + back                                     # 更新一个角点的优化截止点
            else:
                pass
            if i == len(indices[0])-1:              # 末尾角点
                path = np.vstack([path,             # 补全路径
                                interval_data[path_id_old:],
                                ])

    path = interpolate_path(path, 1)    # 进行样条插值 2
    # plath_test.show_polt(path, 1)                            # 显示更新后的路径点

    return path, indices                                        # TODO:测试时多传了indices测试

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
# 根据路线生成指令
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
            instructions.append(f"直行{current_distance:.2f}米")

            # 判断转弯方向
            if delta_angle_deg > 0:
                instructions.append("左转")
            else:
                instructions.append("右转")

            # 更新方向和距离
            current_direction = current_angle
            current_distance = distance_meters
        else:
            current_distance += distance_meters

            # 处理最后一段直行
    instructions.append(f"直行{current_distance:.2f}米")

    return '，'.join(instructions)
##############################################################################################
# 根据指令生成路径
def commands2path(commands_str,x,y,yaw):
    """
    解析命令字符串，返回动作列表 
    例如，输入“直行5米，右转，直行3米”，
    输出 [('forward', 5), ('turn_right', ), ('forward', 3)]
    """
    actions = []
    parts = commands_str.split('，') 
    for part in parts:
        part = part.strip() 
        if '直行' in part:
            # 提取数字部分 
            distance_str = part.replace('直行', '').replace('米', '').strip()
            try:
                distance = float(distance_str)
                actions.append(('forward',  distance))
            except ValueError:
                pass  # 忽略无效的数字 
        elif '右转' in part:
            actions.append(('turn_right',  ))
        elif '左转' in part:
            actions.append(('turn_left',  ))
    
    scale = 0.0564  # 米/像素 
    direction = 0  # 朝向 0: 右 (x+), 1: 上 (y+), 2: 左 (x-), 3: 下 (y-)
    path = [[x, y]]  # 起始点 
 
    for action in actions:
        if action[0] == 'forward':
            distance = action[1]
            pixels = distance / scale 
            if direction == 0:
                x += pixels 
            elif direction == 1:
                y += pixels 
            elif direction == 2:
                x -= pixels 
            elif direction == 3:
                y -= pixels 
            path.append([x,  y])
        elif action[0] == 'turn_left':
            direction = (direction + 1) % 4 
        elif action[0] == 'turn_right':
            direction = (direction - 1) % 4 
    path = rotate_path(path,yaw)
    path = path.tolist()
    return path 

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

def instructions_main():
    pass
def bezier_main():
    # path1 = pathplanning.interpolate_path(interval_data[id-front:id+back], 5)   # 插值
    Ps = np.array([[0.,  0.],   # 修正后的二维数组定义 
                [1.,  0.],
                [2., 0.],
                [2., 1.],
                [3., 1.],
                [4., 1.],
                [5., 1.]])  # 形状为(4,2)
    path1 = []                              # 路径点存储
    # 贝塞尔曲线生成
    for t in np.arange(0, 1.0, 0.1):
        p_t = bezier(Ps, len(Ps), t)
        path1.append(p_t)
    path1 = np.array(path1)

    path2 = []                              # 路径点存储
    # 贝塞尔曲线生成
    for t in np.arange(0, 1.2, 0.1):
        p_t = bezier(Ps, len(Ps), t)
        path2.append(p_t)
    path2 = np.array(path2)

    # print(path1[:,0],path1[:,1])
    # print("******************")
    # print(path2[:,0],path2[:,1])
    
    # plt.scatter(Ps[:,0],Ps[:,1])
    # plt.plot(path1[:,0],path1[:,1],'g')
    # plt.plot(path2[:,0],path2[:,1],'r')
    # plt.show()
    
def plt_lot(free_lot_id, start_pos=None, target_lot_id=None, key_points=None,
            search_path=None, parking_path=None):
    # 车位坐标(像素坐标) y轴向下版，
    cars = {1: [389, 55], 2: [431, 55], 3: [473, 55], 4: [532, 55], 5: [574, 55], 6: [616, 55], 7: [676, 55],
            8: [718, 55], 9: [759, 55],
            10: [331, 253], 11: [389, 253], 12: [431, 253], 13: [473, 253], 14: [533, 253], 15: [575, 253],
            16: [616, 253], 17: [676, 253], 18: [718, 253], 19: [760, 253],
            20: [331, 346], 21: [389, 346], 22: [431, 346], 23: [473, 346], 24: [532, 346], 25: [574, 346],
            26: [616, 346], 27: [676, 346], 28: [718, 346], 29: [759, 346],
            30: [331, 540], 31: [389, 541], 32: [431, 541], 33: [473, 541], 34: [533, 541], 35: [575, 541],
            36: [616, 541], 37: [676, 541], 38: [718, 541], 39: [760, 541],
            40: [331, 633], 41: [389, 634], 42: [431, 634], 43: [473, 634], 44: [532, 634], 45: [574, 634],
            46: [616, 634], 47: [676, 634], 48: [718, 634], 49: [759, 634],
            50: [318, 831], 51: [389, 831], 52: [431, 831], 53: [473, 831], 54: [533, 831], 55: [575, 831],
            56: [616, 831], 57: [676, 831], 58: [718, 831], 59: [760, 831]}

    # 读取图像
    image = cv2.imread(r'/home/www/auto_parking/server_ws/src/server_python/server_python/parking_status_result.png')

    pix = 0.0564
    # 坐标变换
    img_height = image.shape[0]
    if start_pos is not None:
        # [x, y] -> [x, height - y]
        start_pos = [int(start_pos[0]/pix), int(img_height - start_pos[1]/pix)]

    if key_points is not None:
        # [[x1, y1], [x2, y2], ...] -> [[x1, height - y1], [x2, height - y2], ...]
        key_points = [[int(p[0]), int(img_height - p[1])] for p in key_points]

    if search_path is not None:
        # [[x1, y1], [x2, y2], ...] -> [[x1, height - y1], [x2, height - y2], ...]
        search_path = [[int(p[0]/pix), int(img_height - p[1]/pix)] for p in search_path]

    if parking_path is not None:
        # [[x1, y1], [x2, y2], ...] -> [[x1, height - y1], [x2, height - y2], ...]
        parking_path = [[p[0]/pix, img_height - p[1]/pix] for p in parking_path]

    # 将非方框区域转为灰度  # 取消
    gray_background = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray_background = cv2.cvtColor(gray_background, cv2.COLOR_GRAY2BGR)

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # 设定蓝色HSV范围
    lower_blue = np.array([100, 100, 50])
    upper_blue = np.array([140, 255, 255])

    # 生成蓝色掩码
    mask = cv2.inRange(hsv, lower_blue, upper_blue)

    # 查找轮廓
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 创建拷贝用于绘制
    output = image.copy()  # 取消了灰度图，如要其余车位灰度图像 使用output = gray_background.copy()

    # 颜色定义
    PARKING_AVAILABLE_COLOR = (0, 180, 0)  # 深绿色
    PARKING_UNAVAILABLE_COLOR = (0, 0, 200)  # 深红色
    BORDER_COLOR = (128, 128, 128)  # 灰色边框
    FILL_COLOR_AVAILABLE = (200, 255, 200)  # 浅绿填充
    FILL_COLOR_UNAVAILABLE = (220, 220, 255)  # 浅红填充

    START_COLOR = (0, 180, 0)  # 深绿色
    TARGET_COLOR = (0, 165, 255)  # 橙色目标车位
    KEY_POINT_COLOR = (255, 255, 0)  # 青色关键点
    SEARCH_PATH_COLOR = (255, 100, 100)  # 粉色寻车路径
    PARKING_PATH_COLOR = (100, 100, 255)  # 紫色泊车路径

    # 首先对所有车位进行排序（从左到右，从上到下）
    # 使用中心点坐标作为排序依据
    sorted_contours = sorted(contours, key=lambda c: (cv2.boundingRect(c)[1], cv2.boundingRect(c)[0]))

    # 处理方框
    for cnt in sorted_contours:
        x, y, w, h = cv2.boundingRect(cnt)  # _,_,38,90
        center_x = x + w // 2
        center_y = y + h // 2

        # 判断是否在指定区域
        in_special_area = center_x < 300 or center_x > 780
        is_available = not in_special_area  # 指定区域为不可用

        # 方框填充
        fill_color = (255, 255, 255) if is_available else FILL_COLOR_UNAVAILABLE
        output[y:y + h, x:x + w] = fill_color

        # 绘制虚线边框
        step = 10
        for i in range(x, x + w, step):
            cv2.line(output, (i, y), (i + 5, y), BORDER_COLOR, 1)
            cv2.line(output, (i, y + h), (i + 5, y + h), BORDER_COLOR, 1)
        for j in range(y, y + h, step):
            cv2.line(output, (x, j), (x, j + 5), BORDER_COLOR, 1)
            cv2.line(output, (x + w, j), (x + w, j + 5), BORDER_COLOR, 1)
        if not is_available:
            # 绘制斜线
            cv2.line(output, (x + 5, y + 5), (x + w - 5, y + h - 5), (0, 0, 200), 1)
            cv2.line(output, (x + w - 5, y + 5), (x + 5, y + h - 5), (0, 0, 200), 1)

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
            output[center_y - 45:center_y + 45, center_x - 19:center_x + 19] = FILL_COLOR_AVAILABLE

            # 绿色"P"
            text_size = cv2.getTextSize("P", font, font_scale, 2)[0]
            cv2.putText(output, "P",
                        (center_x - text_size[0] // 2, center_y + text_size[1] // 2),
                        font, font_scale, PARKING_AVAILABLE_COLOR, 2, cv2.LINE_AA)

            number_text = str(spot_id)
            number_size = cv2.getTextSize(number_text, font, font_scale * 0.8, 1)[0]
            cv2.putText(output, number_text,
                        (center_x - number_size[0] // 2, center_y + text_size[1] // 2 - 20),  # 在方框顶部显示编号
                        font, font_scale * 0.8, (0, 0, 0), 1, cv2.LINE_AA)

        else:
            output[center_y - 45:center_y + 45, center_x - 19:center_x + 19] = FILL_COLOR_UNAVAILABLE
            # 绘制斜线
            cv2.line(output, (center_x - 19 + 5, center_y - 45 + 5),
                     (center_x - 19 + 19 * 2 - 5, center_y - 45 + 90 - 5), PARKING_UNAVAILABLE_COLOR, 1)
            cv2.line(output, (center_x - 19 + 19 * 2 - 5, center_y - 45 + 5),
                     (center_x - 19 + 5, center_y - 45 + 90 - 5), PARKING_UNAVAILABLE_COLOR, 1)

        step = 10
        for i in range(x, x + w, step):
            cv2.line(output, (i, y), (i + 5, y), BORDER_COLOR, 1)
            cv2.line(output, (i, y + h), (i + 5, y + h), BORDER_COLOR, 1)
        for j in range(y, y + h, step):
            cv2.line(output, (x, j), (x, j + 5), BORDER_COLOR, 1)
            cv2.line(output, (x + w, j), (x + w, j + 5), BORDER_COLOR, 1)

    # ==================== 新增：绘制路径和关键点 ====================
    # # 绘制起始点（黄色圆点）
    # if start_pos is not None:
    #     cv2.circle(output, (int(start_pos[0]), int(start_pos[1])), 10, START_COLOR, -1)
    #     cv2.putText(output, "Start", (int(start_pos[0]) - 15, int(start_pos[1])),
    #                 cv2.FONT_HERSHEY_SIMPLEX, 0.6, START_COLOR, 2)
    # 绘制起始点（绿色向右箭头）
    if start_pos is not None:
        # 箭头中心坐标
        center_x, center_y = int(start_pos[0]), int(start_pos[1])

        # 画向右的箭头 (从中心左侧画到中心右侧)
        # (center_x - 20, center_y) 是起点，(center_x + 20, center_y) 是终点
        cv2.arrowedLine(output, (center_x - 20, center_y), (center_x + 20, center_y), START_COLOR, 6, tipLength=0.2)

        # 文字也对应改为绿色
        cv2.putText(output, "Start", (center_x - 45, center_y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, START_COLOR, 2)

    # 绘制目标车位（橙色标记）
    if target_lot_id is not None and target_lot_id in cars:
        target_x, target_y = cars[target_lot_id]
        cv2.circle(output, (target_x, target_y), 15, PARKING_PATH_COLOR, 3)
        cv2.putText(output, f"Target {target_lot_id}", (target_x - 40, target_y - 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, PARKING_PATH_COLOR, 2)

    # # 绘制关键路径点（青色小圆点）
    # if key_points is not None:
    #     for i, point in enumerate(key_points):
    #         cv2.circle(output, (int(point[0]), int(point[1])), 8, KEY_POINT_COLOR, -1)
    #         cv2.putText(output, f"K{i}", (int(point[0]) + 12, int(point[1])),
    #                     cv2.FONT_HERSHEY_SIMPLEX, 0.4, KEY_POINT_COLOR, 1)
    # 绘制关键路径点（青色五角星）
    if key_points is not None:
        for i, point in enumerate(key_points):
            cx, cy = int(point[0]), int(point[1])
            radius = 20  # 五角星大小
            # 计算五角星的10个顶点坐标
            star_points = []
            for j in range(10):
                angle = j * 36 * np.pi / 180 - np.pi / 2  # 从顶部开始，逆时针
                # 外圆半径（5个点）和内圆半径（5个点）交替
                r = radius if j % 2 == 0 else radius * 0.4
                px = cx + r * np.cos(angle)
                py = cy + r * np.sin(angle)
                star_points.append([px, py])
            star_points = np.array(star_points, np.int32)
            cv2.fillPoly(output, [star_points], KEY_POINT_COLOR)
            cv2.putText(output, f"KP{i}", (cx + radius + 5, cy + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, KEY_POINT_COLOR, 2)

    # 绘制寻车路径（粉色实线）
    if search_path is not None and len(search_path) > 1:
        pts = np.array(search_path, dtype=np.int32)
        pts = pts[::3].astype(np.int32)

        pts = pts.reshape((-1, 1, 2))
        # pts = pts[::3]
        cv2.polylines(output, [pts], False, SEARCH_PATH_COLOR, 3, cv2.LINE_AA)
        cv2.putText(output, "Search Path", (int(search_path[0][0]) + 20, int(search_path[0][1])-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, SEARCH_PATH_COLOR, 2)

    # 绘制泊车路径（紫色虚线）
    if parking_path is not None and len(parking_path) > 1:
        for i in range(len(parking_path) - 1):
            # 绘制虚线效果
            pt1 = (int(parking_path[i][0]), int(parking_path[i][1]))
            pt2 = (int(parking_path[i + 1][0]), int(parking_path[i + 1][1]))
            # 虚线参数
            dash_length = 10
            gap_length = 5
            # 计算线段长度和方向
            dx = pt2[0] - pt1[0]
            dy = pt2[1] - pt1[1]
            length = np.sqrt(dx * dx + dy * dy)
            if length > 0:
                num_dashes = int(length / (dash_length + gap_length))
                for d in range(num_dashes + 1):
                    t1 = d * (dash_length + gap_length) / length
                    t2 = min((d * (dash_length + gap_length) + dash_length) / length, 1.0)
                    dash_pt1 = (int(pt1[0] + dx * t1), int(pt1[1] + dy * t1))
                    dash_pt2 = (int(pt1[0] + dx * t2), int(pt1[1] + dy * t2))
                    cv2.line(output, dash_pt1, dash_pt2, TARGET_COLOR, 3)
        cv2.putText(output, "Parking Path", (int(parking_path[0][0]) + 35, int(parking_path[0][1]) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, TARGET_COLOR, 2)

    # 保存图像
    # cv2.imwrite('/home/www/auto_parking/server_ws/src/server_python/server_python/LLMA/parking_result_with_paths.png', output)

    # # 也可以选择显示
    # cv2.imshow('Parking Status with Paths', output)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    # return output
    # 步骤1：将OpenCV的BGR图像转为PIL的RGB图像
    output_rgb = cv2.cvtColor(output, cv2.COLOR_BGR2RGB)
    pil_img = Image.fromarray(output_rgb)

    # 步骤2：保存为临时PNG（也可直接用内存流，避免临时文件）
    temp_png_path = '/home/www/auto_parking/server_ws/src/server_python/server_python/LLMA/temp_parking.png'
    pil_img.save(temp_png_path)

    # 步骤3：创建PDF并插入图像
    pdf = FPDF()
    pdf.add_page()  # 添加一页
    # 获取图像尺寸，按PDF页面适配（单位：mm，可自定义尺寸）
    img_width = 180  # PDF页面宽度约210mm，留边距设180mm
    pdf.image(temp_png_path, x=10, y=10, w=img_width)  # x/y为左上角坐标

    # 步骤4：保存PDF文件
    pdf_path = '/home/www/auto_parking/server_ws/src/server_python/server_python/LLMA/parking_result_with_paths.pdf'
    pdf.output(pdf_path)

    # 可选：删除临时PNG文件
    os.remove(temp_png_path)

def plt_lot(free_lot_id, start_pos=None, target_lot_id=None, key_points=None,
            search_path=None, parking_path=None, explored_path=None):
    # 车位坐标(像素坐标) y轴向下版，
    cars = {1: [389, 55], 2: [431, 55], 3: [473, 55], 4: [532, 55], 5: [574, 55], 6: [616, 55], 7: [676, 55],
            8: [718, 55], 9: [759, 55],
            10: [331, 253], 11: [389, 253], 12: [431, 253], 13: [473, 253], 14: [533, 253], 15: [575, 253],
            16: [616, 253], 17: [676, 253], 18: [718, 253], 19: [760, 253],
            20: [331, 346], 21: [389, 346], 22: [431, 346], 23: [473, 346], 24: [532, 346], 25: [574, 346],
            26: [616, 346], 27: [676, 346], 28: [718, 346], 29: [759, 346],
            30: [331, 540], 31: [389, 541], 32: [431, 541], 33: [473, 541], 34: [533, 541], 35: [575, 541],
            36: [616, 541], 37: [676, 541], 38: [718, 541], 39: [760, 541],
            40: [331, 633], 41: [389, 634], 42: [431, 634], 43: [473, 634], 44: [532, 634], 45: [574, 634],
            46: [616, 634], 47: [676, 634], 48: [718, 634], 49: [759, 634],
            50: [318, 831], 51: [389, 831], 52: [431, 831], 53: [473, 831], 54: [533, 831], 55: [575, 831],
            56: [616, 831], 57: [676, 831], 58: [718, 831], 59: [760, 831]}

    # 读取图像
    image = cv2.imread(r'/home/www/auto_parking/server_ws/src/server_python/server_python/parking_status_result.png')

    pix = 0.0564
    # 坐标变换
    img_height = image.shape[0]
    if start_pos is not None:
        # [x, y] -> [x, height - y]
        start_pos = [int(start_pos[0]/pix), int(img_height - start_pos[1]/pix)]

    if key_points is not None:
        # [[x1, y1], [x2, y2], ...] -> [[x1, height - y1], [x2, height - y2], ...]
        key_points = [[int(p[0]), int(img_height - p[1])] for p in key_points]

    if search_path is not None:
        # [[x1, y1], [x2, y2], ...] -> [[x1, height - y1], [x2, height - y2], ...]
        search_path = [[int(p[0]/pix), int(img_height - p[1]/pix)] for p in search_path]
    
    if explored_path is not None:
        # [[x1, y1], [x2, y2], ...] -> [[x1, height - y1], [x2, height - y2], ...]
        explored_path = [[int(p[0]/pix), int(img_height - p[1]/pix)] for p in explored_path]

    if parking_path is not None:
        # [[x1, y1], [x2, y2], ...] -> [[x1, height - y1], [x2, height - y2], ...]
        parking_path = [[p[0]/pix, img_height - p[1]/pix] for p in parking_path]

    # 将非方框区域转为灰度  # 取消
    gray_background = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray_background = cv2.cvtColor(gray_background, cv2.COLOR_GRAY2BGR)

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # 设定蓝色HSV范围
    lower_blue = np.array([100, 100, 50])
    upper_blue = np.array([140, 255, 255])

    # 生成蓝色掩码
    mask = cv2.inRange(hsv, lower_blue, upper_blue)

    # 查找轮廓
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 创建拷贝用于绘制
    output = image.copy()  # 取消了灰度图，如要其余车位灰度图像 使用output = gray_background.copy()

    # 颜色定义
    PARKING_AVAILABLE_COLOR = (0, 180, 0)  # 深绿色
    PARKING_UNAVAILABLE_COLOR = (0, 0, 200)  # 深红色
    BORDER_COLOR = (128, 128, 128)  # 灰色边框
    FILL_COLOR_AVAILABLE = (200, 255, 200)  # 浅绿填充
    FILL_COLOR_UNAVAILABLE = (220, 220, 255)  # 浅红填充

    START_COLOR = (0, 180, 0)  # 深绿色
    TARGET_COLOR = (0, 165, 255)  # 橙色目标车位
    KEY_POINT_COLOR = (255, 255, 0)  # 青色关键点
    SEARCH_PATH_COLOR = (255, 100, 100)  # 粉色寻车路径
    PARKING_PATH_COLOR = (100, 100, 255)  # 紫色泊车路径

    # 首先对所有车位进行排序（从左到右，从上到下）
    # 使用中心点坐标作为排序依据
    sorted_contours = sorted(contours, key=lambda c: (cv2.boundingRect(c)[1], cv2.boundingRect(c)[0]))

    # 处理方框
    for cnt in sorted_contours:
        x, y, w, h = cv2.boundingRect(cnt)  # _,_,38,90
        center_x = x + w // 2
        center_y = y + h // 2

        # 判断是否在指定区域
        in_special_area = center_x < 300 or center_x > 780
        is_available = not in_special_area  # 指定区域为不可用

        # 方框填充
        fill_color = (255, 255, 255) if is_available else FILL_COLOR_UNAVAILABLE
        output[y:y + h, x:x + w] = fill_color

        # 绘制虚线边框
        step = 10
        for i in range(x, x + w, step):
            cv2.line(output, (i, y), (i + 5, y), BORDER_COLOR, 1)
            cv2.line(output, (i, y + h), (i + 5, y + h), BORDER_COLOR, 1)
        for j in range(y, y + h, step):
            cv2.line(output, (x, j), (x, j + 5), BORDER_COLOR, 1)
            cv2.line(output, (x + w, j), (x + w, j + 5), BORDER_COLOR, 1)
        if not is_available:
            # 绘制斜线
            cv2.line(output, (x + 5, y + 5), (x + w - 5, y + h - 5), (0, 0, 200), 1)
            cv2.line(output, (x + w - 5, y + 5), (x + 5, y + h - 5), (0, 0, 200), 1)

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
            output[center_y - 45:center_y + 45, center_x - 19:center_x + 19] = FILL_COLOR_AVAILABLE

            # 绿色"P"
            text_size = cv2.getTextSize("P", font, font_scale, 2)[0]
            cv2.putText(output, "P",
                        (center_x - text_size[0] // 2, center_y + text_size[1] // 2),
                        font, font_scale, PARKING_AVAILABLE_COLOR, 2, cv2.LINE_AA)

            number_text = str(spot_id)
            number_size = cv2.getTextSize(number_text, font, font_scale * 0.8, 1)[0]
            cv2.putText(output, number_text,
                        (center_x - number_size[0] // 2, center_y + text_size[1] // 2 - 20),  # 在方框顶部显示编号
                        font, font_scale * 0.8, (0, 0, 0), 1, cv2.LINE_AA)

        else:
            output[center_y - 45:center_y + 45, center_x - 19:center_x + 19] = FILL_COLOR_UNAVAILABLE
            # 绘制斜线
            cv2.line(output, (center_x - 19 + 5, center_y - 45 + 5),
                     (center_x - 19 + 19 * 2 - 5, center_y - 45 + 90 - 5), PARKING_UNAVAILABLE_COLOR, 1)
            cv2.line(output, (center_x - 19 + 19 * 2 - 5, center_y - 45 + 5),
                     (center_x - 19 + 5, center_y - 45 + 90 - 5), PARKING_UNAVAILABLE_COLOR, 1)

        step = 10
        for i in range(x, x + w, step):
            cv2.line(output, (i, y), (i + 5, y), BORDER_COLOR, 1)
            cv2.line(output, (i, y + h), (i + 5, y + h), BORDER_COLOR, 1)
        for j in range(y, y + h, step):
            cv2.line(output, (x, j), (x, j + 5), BORDER_COLOR, 1)
            cv2.line(output, (x + w, j), (x + w, j + 5), BORDER_COLOR, 1)

    # ==================== 新增：绘制路径和关键点 ====================
    # # 绘制起始点（黄色圆点）
    # if start_pos is not None:
    #     cv2.circle(output, (int(start_pos[0]), int(start_pos[1])), 10, START_COLOR, -1)
    #     cv2.putText(output, "Start", (int(start_pos[0]) - 15, int(start_pos[1])),
    #                 cv2.FONT_HERSHEY_SIMPLEX, 0.6, START_COLOR, 2)
    # 绘制起始点（绿色向右箭头）
    if start_pos is not None:
        # 箭头中心坐标
        center_x, center_y = int(start_pos[0]), int(start_pos[1])

        # 画向右的箭头 (从中心左侧画到中心右侧)
        # (center_x - 20, center_y) 是起点，(center_x + 20, center_y) 是终点
        cv2.arrowedLine(output, (center_x - 20, center_y), (center_x + 20, center_y), START_COLOR, 6, tipLength=0.2)

        # 文字也对应改为绿色
        cv2.putText(output, "Start", (center_x - 45, center_y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, START_COLOR, 2)

    # 绘制目标车位（橙色标记）
    if target_lot_id is not None and target_lot_id in cars:
        target_x, target_y = cars[target_lot_id]
        cv2.circle(output, (target_x, target_y), 15, PARKING_PATH_COLOR, 3)
        cv2.putText(output, f"Target {target_lot_id}", (target_x - 40, target_y - 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, PARKING_PATH_COLOR, 2)

    # # 绘制关键路径点（青色小圆点）
    # if key_points is not None:
    #     for i, point in enumerate(key_points):
    #         cv2.circle(output, (int(point[0]), int(point[1])), 8, KEY_POINT_COLOR, -1)
    #         cv2.putText(output, f"K{i}", (int(point[0]) + 12, int(point[1])),
    #                     cv2.FONT_HERSHEY_SIMPLEX, 0.4, KEY_POINT_COLOR, 1)
    # 绘制关键路径点（青色五角星）
    if key_points is not None:
        for i, point in enumerate(key_points):
            cx, cy = int(point[0]), int(point[1])
            radius = 20  # 五角星大小
            # 计算五角星的10个顶点坐标
            star_points = []
            for j in range(10):
                angle = j * 36 * np.pi / 180 - np.pi / 2  # 从顶部开始，逆时针
                # 外圆半径（5个点）和内圆半径（5个点）交替
                r = radius if j % 2 == 0 else radius * 0.4
                px = cx + r * np.cos(angle)
                py = cy + r * np.sin(angle)
                star_points.append([px, py])
            star_points = np.array(star_points, np.int32)
            cv2.fillPoly(output, [star_points], KEY_POINT_COLOR)
            cv2.putText(output, f"KP{i}", (cx + radius + 5, cy + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, KEY_POINT_COLOR, 2)

    if explored_path is not None:
        # 坐标转换: [[x, y]] -> [[x, height - y]]
        for pt in explored_path:
            x, y = pt
            size = 5  # '×' 大小
            color = (255, 182, 193)  # 可以设置为粉色或你喜欢的颜色
            thickness = 2
            # 绘制 '×'：两条对角线
            cv2.line(output, (x - size, y - size), (x + size, y + size), color, thickness, cv2.LINE_AA)
            cv2.line(output, (x - size, y + size), (x + size, y - size), color, thickness, cv2.LINE_AA)
        # 可选文字说明
        if len(explored_path) > 0:
            cv2.putText(output, "Explored Points", 
                        (explored_path[0][0] + 10, explored_path[0][1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
    # 绘制寻车路径（粉色实线）
    if search_path is not None and len(search_path) > 1:
        pts = np.array(search_path, dtype=np.int32)
        pts = pts[::3].astype(np.int32)

        pts = pts.reshape((-1, 1, 2))
        # pts = pts[::3]
        cv2.polylines(output, [pts], False, SEARCH_PATH_COLOR, 3, cv2.LINE_AA)
        cv2.putText(output, "Search Path", (int(search_path[0][0]) + 20, int(search_path[0][1])-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, SEARCH_PATH_COLOR, 2)

    # 绘制泊车路径（紫色虚线）
    if parking_path is not None and len(parking_path) > 1:
        for i in range(len(parking_path) - 1):
            # 绘制虚线效果
            pt1 = (int(parking_path[i][0]), int(parking_path[i][1]))
            pt2 = (int(parking_path[i + 1][0]), int(parking_path[i + 1][1]))
            # 虚线参数
            dash_length = 10
            gap_length = 5
            # 计算线段长度和方向
            dx = pt2[0] - pt1[0]
            dy = pt2[1] - pt1[1]
            length = np.sqrt(dx * dx + dy * dy)
            if length > 0:
                num_dashes = int(length / (dash_length + gap_length))
                for d in range(num_dashes + 1):
                    t1 = d * (dash_length + gap_length) / length
                    t2 = min((d * (dash_length + gap_length) + dash_length) / length, 1.0)
                    dash_pt1 = (int(pt1[0] + dx * t1), int(pt1[1] + dy * t1))
                    dash_pt2 = (int(pt1[0] + dx * t2), int(pt1[1] + dy * t2))
                    cv2.line(output, dash_pt1, dash_pt2, TARGET_COLOR, 3)
        cv2.putText(output, "Parking Path", (int(parking_path[0][0]) + 35, int(parking_path[0][1]) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, TARGET_COLOR, 2)

    # 保存图像
    # cv2.imwrite('/home/www/auto_parking/server_ws/src/server_python/server_python/LLMA/parking_result_with_paths.png', output)

    # # 也可以选择显示
    # cv2.imshow('Parking Status with Paths', output)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    # return output
    # 步骤1：将OpenCV的BGR图像转为PIL的RGB图像
    output_rgb = cv2.cvtColor(output, cv2.COLOR_BGR2RGB)
    pil_img = Image.fromarray(output_rgb)

    # 步骤2：保存为临时PNG（也可直接用内存流，避免临时文件）
    temp_png_path = '/home/www/auto_parking/server_ws/src/server_python/server_python/LLMA/temp_parking.png'
    pil_img.save(temp_png_path)

    # 步骤3：创建PDF并插入图像
    pdf = FPDF()
    pdf.add_page()  # 添加一页
    # 获取图像尺寸，按PDF页面适配（单位：mm，可自定义尺寸）
    img_width = 180  # PDF页面宽度约210mm，留边距设180mm
    pdf.image(temp_png_path, x=10, y=10, w=img_width)  # x/y为左上角坐标

    # 步骤4：保存PDF文件
    pdf_path = '/home/www/auto_parking/server_ws/src/server_python/server_python/LLMA/parking_result_with_paths.pdf'
    pdf.output(pdf_path)

    # 可选：删除临时PNG文件
    os.remove(temp_png_path)

if __name__ == '__main__':
    # main()
    # bezier_main()
    
    Ps = np.array([[14.1564,  45.5712],   # 修正后的二维数组定义 
            [15.1716,  45.5712],
            [15.1716, 44.556],
            [15.1716, 43.5408],
            ])  # 形状为(4,2)

    path1 = []                              # 路径点存储
    # 贝塞尔曲线生成
    for t in np.arange(0, 1.2, 0.08):                        # 6个点
        p_t = bezier(Ps, len(Ps), t)
        path1.append(p_t)
    path1 = np.array(path1)
    
    plt.plot(path1[:,0],path1[:,1],'.r')
    plt.show()
