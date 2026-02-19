import math
import numpy as np
import os
import matplotlib.pyplot as plt
import matplotlib.font_manager as font_manager
from datetime import datetime
import ament_index_python
import matplotlib
from matplotlib.colors  import LogNorm 

import csv
import pandas as pd
# matplotlib.use('agg')

class DataLogger:
    def __init__(self):
        self.path = []
        self.yaw = []
        self.linear = []
        self.angular = []
        self.ekf_pos = []
        self.odom_path = []
        self.uwb_path = []

        self.T0_pos = []
        self.T1_pos = []
        self.T2_pos = []
        self.T3_pos = []


        # 查找已知功能包的位置 
        package_name = 'autoparking_python'  # 替换为实际存在的功能包名称 
        package_path = ament_index_python.get_package_prefix(package_name) 
        # 假设功能包位于src目录下 
        src_path = os.path.dirname(os.path.dirname(package_path)) 
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
        
        self.ekf_pos_directory = os.path.join(self.directory,  'ekf_pos.csv')
        self.uwb_pos_directory = os.path.join(self.directory,  'uwb_pos.csv')
        self.uwbs_pos_directory = os.path.join(self.directory,  'uwbs_pos.csv')
        self.reference_directory = os.path.join(self.directory,  'reference.csv')
        self.odom_directory = os.path.join(self.directory,  'odom.csv')                     # 里程计
        self.uwb_directory = os.path.join(self.directory,  'uwb.csv')                       # 纯 UWB ， x,y

    # def log(self, point, my_car, acc, delta):# 原始
    #     self.path.append(point*10)
    #     self.car_state.append([my_car.x*10, my_car.y*10, my_car.v*10, my_car.psi])
    #     self.u.append([acc, delta])
    
    # 记录车辆运动的数据：当前x，y，yaw，linear，angular
    ############增加ekf数据############
    def log(self, point, yaw, linear, angular,ekf_pos):
        self.path.append(point)                 # 用了滤波 就是 滤波后的坐标
        self.yaw.append(yaw)
        self.linear.append(linear)
        self.angular.append(angular)
        self.ekf_pos.append(ekf_pos)            # aekf

    def log_odom(self,odom):
        self.odom_path.append(odom)
    
    def log_uwb(self,uwb):                      # uwb
        self.uwb_path.append(uwb)

    def save_command(self, command):

        system_time = datetime.now()
        full_datetime = datetime.combine(
            system_time.date(),
            system_time.time(),
        )
        formatted_full_datetime = full_datetime.strftime("%Y-%m-%d %H:%M:%S")
        directory_command = os.path.join(self.directory,  'command.txt')
        try:
            with open(directory_command,  'a', encoding='utf-8') as file:
                file.write(command)
                file.write('    --')
                file.write(formatted_full_datetime) 
                file.write('\n')
            print("文件保存成功！")
        except Exception as e:
            print(f"错误: {e}")

    def save_data(self,path_p,command):

        # 调整文件路径
        self.save_command(command)
        t = np.arange(0,len(self.path),1)
        t = t * 0.2
        # self.path_p = path_p[:len(path_p)-1-6]                                                    # planning参考轨迹
        self.path_p = path_p                                                 # planning参考轨迹
        self.path = np.array(self.path)
        print(f'payh_p:{len(self.path_p)}\npath:{len(self.path)}')              # 校准路径长度
        self.yaw = np.array(self.yaw)
        self.linear = np.array(self.linear)
        self.angular = np.array(self.angular)
        font = font_manager.FontProperties(family='Times New Roman', weight='bold',style='normal', size=20)

        # plot x
        plt.figure(figsize=(12,8))
        plt.plot(t, self.path[:,0], color='r', linewidth=5)
        plt.title('car\'s x in time',fontsize=20)
        plt.xlabel('time (s)',fontsize=20)
        plt.ylabel('x (m)',fontsize=20)
        plt.grid()
        plt.axis('equal')
        plt.legend(['reference', 'car\'s x'], prop=font) # using a named size
        # 智能边距压缩（自适应策略）
        plt.tight_layout(pad=0.5)   # pad控制整体边距
        plt.savefig(os.path.join(self.directory,  'x.png'),  
                bbox_inches='tight',  # 边界框紧贴内容 
                pad_inches=0.05)      # 保留5%的安全边距 
        # plt.savefig(os.path.join(self.directory,  'x.png'))

        # plot y
        plt.figure(figsize=(12,8))
        plt.plot(t, self.path[:,1], color='r', linewidth=5)
        plt.title('car\'s y in time',fontsize=20)
        plt.xlabel('time (s)',fontsize=20)
        plt.ylabel('y (m)',fontsize=20)
        plt.axis('equal')
        plt.grid()
        plt.legend(['reference', 'car\'s y'], prop=font) # using a named size
        # 智能边距压缩（自适应策略）
        plt.tight_layout(pad=0.5)   # pad控制整体边距
        plt.savefig(os.path.join(self.directory,  'y.png'),  
                bbox_inches='tight',  # 边界框紧贴内容 
                pad_inches=0.05)      # 保留5%的安全边距 
        # plt.savefig(os.path.join(self.directory,  'y.png'))
        # plt.savefig('log results/y.png')

        # plot linear        
        plt.figure(figsize=(12,8))
        plt.plot(t, self.linear, color='r', linewidth=4)
        plt.title('car\'s speed in time',fontsize=20)
        plt.xlabel('time (s)',fontsize=20)
        plt.ylabel('v (m/s)',fontsize=20)
        plt.axis('equal')
        plt.grid()
        plt.legend(['car speed (m/s)'], prop=font) # using a named size
        # 智能边距压缩（自适应策略）
        plt.tight_layout(pad=0.5)   # pad控制整体边距
        plt.savefig(os.path.join(self.directory,  'v.png'),  
                bbox_inches='tight',  # 边界框紧贴内容 
                pad_inches=0.05)      # 保留5%的安全边距 
        # plt.savefig(os.path.join(self.directory,  'v.png'))
        # plt.savefig('log results/v.png')

        # plot yaw
        plt.figure(figsize=(12,8))
        plt.plot(t, np.rad2deg(self.yaw), color='r', linewidth=4)
        plt.title('car\'s yaw in time',fontsize=20)
        plt.xlabel('time (s)',fontsize=20)
        plt.ylabel('psi (degree)',fontsize=20)
        plt.axis('equal')
        plt.grid()
        plt.legend(['car yaw (degree)'], prop=font) # using a named size
        # 智能边距压缩（自适应策略）
        plt.tight_layout(pad=0.5)   # pad控制整体边距
        plt.savefig(os.path.join(self.directory,  'psi.png'),  
                bbox_inches='tight',  # 边界框紧贴内容 
                pad_inches=0.05)      # 保留5%的安全边距 
        # plt.savefig(os.path.join(self.directory,  'psi.png'))
        # plt.savefig('log results/psi.png')

        # plot position
        plt.figure(figsize=(12,12))
        plt.scatter(self.path[:,0], self.path[:,1], color='r', linewidth=5)
        plt.plot(self.path_p[:,0], self.path_p[:,1], color='b', linewidth=4)      # TODO:蓝色的参考路径
        plt.title('car\'s position in time',fontsize=20)
        plt.xlabel('x (m)',fontsize=20)
        plt.ylabel('y (m)',fontsize=20)
        plt.axis('equal')
        plt.grid()
        plt.legend(['car\'s position','reference'], prop=font) # using a named size
        # 智能边距压缩（自适应策略）
        plt.tight_layout(pad=0.5)   # pad控制整体边距
        plt.savefig(os.path.join(self.directory,  'position.png'),  
                bbox_inches='tight',  # 边界框紧贴内容 
                pad_inches=0.05)      # 保留5%的安全边距 
        # plt.savefig(os.path.join(self.directory,  'position.png'))
        # plt.savefig('log results/position.png')

        # plot angular
        plt.figure(figsize=(12,8))
        plt.plot(t, np.rad2deg(self.angular), color='r', linewidth=4)
        plt.title('car\'s angular in time',fontsize=20)
        plt.xlabel('time (s)',fontsize=20)
        plt.ylabel('angular (degree/s)',fontsize=20)
        plt.axis('equal')
        plt.grid()
        plt.legend(['car angular (degree/s)'], prop=font) # using a named size
        # 智能边距压缩（自适应策略）
        plt.tight_layout(pad=0.5)   # pad控制整体边距
        plt.savefig(os.path.join(self.directory,  'angular.png'),  
                bbox_inches='tight',  # 边界框紧贴内容 
                pad_inches=0.05)      # 保留5%的安全边距 
        # plt.savefig(os.path.join(self.directory,  'angular.png'))
        # plt.savefig('log results/angular.png')

        print('all data saved on log results ...')
        
    ############增加保存定位数据############
    def save_pos(self):

        with open(self.ekf_pos_directory, 'w', newline='') as f:                            # aekf
            writer = csv.writer(f)
            writer.writerow(['ekf_x', 'ekf_y'])      # 写入表头
            writer.writerows(self.ekf_pos)         # 写入坐标行

        self.uwb_pos = np.column_stack([self.path, self.yaw, self.linear, self.angular])    # 用了滤波 就是 滤波后的坐标，车辆MPC用的坐标
        with open(self.uwb_pos_directory, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['uwb_x', 'uwb_y', 'uwb_yaw', 'linear', 'angular'])      # 写入表头
            writer.writerows(self.uwb_pos)             # 写入坐标行

        with open(self.reference_directory, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['reference_x', 'reference_y'])      # 写入表头
            writer.writerows(self.path_p)             # 写入坐标行

        with open(self.odom_directory, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['odom_x', 'odom_y'])      # 写入表头
            writer.writerows(self.odom_path)             # 写入坐标行

        with open(self.uwb_directory, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y'])      # 写入表头
            writer.writerows(self.uwb_path)             # 写入坐标行
        
    def save_pos_test(self):
        with open(self.uwb_directory, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y'])      # 写入表头
            writer.writerows(self.uwb_path)             # 写入坐标行

    ############增加绘制定位数据############
    def plt_pos(self):
        ekf_df = pd.read_csv(self.ekf_pos_directory)
        ekf_x = ekf_df['ekf_x'].tolist()           # 转成 Python 列表
        ekf_y = ekf_df['ekf_y'].tolist()
        print(len(ekf_y))

        uwb_df = pd.read_csv(self.uwb_pos_directory)
        uwb_x = uwb_df['uwb_x'].tolist()           # 转成 Python 列表
        uwb_y = uwb_df['uwb_y'].tolist()
        
        # 创建画布与双轴对象（1行2列布局）
        fig, (ax1, ax2) = plt.subplots(1,  2, figsize=(24, 8), dpi=100, facecolor='#f5f5f5')
        
        # 配置全局参数 
        plt.rcParams['font.size']  = 18  # 统一字体大小 
        plt.rc('axes',  titlesize=22)    # 标题字号 
        
        # ------------------ 全局共性配置 ------------------
        for ax in [ax1, ax2]:
            ax.set_xlabel('X  (m)', labelpad=15)    # 增加标签间距 
            ax.set_ylabel('Y  (m)', labelpad=15)
            # ax.legend(loc='upper  right', framealpha=0.9)  # 半透明图例 
            
            # 添加坐标边框美学优化 
            for spine in ax.spines.values(): 
                spine.set_color('#444444') 
                spine.set_linewidth(1.5) 
        ###############################
        # 在单图中叠加显示两种轨迹 
        ax3 = plt.subplot(111) 
        ax3.scatter(ekf_x,  ekf_y, c='r', label='EKF', alpha=0.5)
        ax3.scatter(uwb_x,  uwb_y, c='b', label='UWB', alpha=0.5)
        ax3.plot(ekf_x,  ekf_y, 'r--', lw=1)  # 增加轨迹连线 
        ax3.plot(uwb_x,  uwb_y, 'b:', lw=1)
        plt.axis('equal')
        # ###############################
        # dx = np.array(ekf_x)  - np.array(uwb_x) 
        # dy = np.array(ekf_y)  - np.array(uwb_y) 
        # error = np.sqrt(dx**2  + dy**2)
        # # 在第二个子图显示误差分布 
        # im = ax2.scatter(ekf_x,  ekf_y, c=error, cmap='viridis', 
        #                 norm=LogNorm(), alpha=0.7)
        # fig.colorbar(im,  ax=ax2, label='Position Error (m)')

        # 动态调整布局 
        plt.tight_layout(pad=4.0) 
        plt.show() 

    def log_four_tags(self, T0, T1, T2, T3):
        """
        [current_x_0, current_y_0],
        [current_x_1, current_y_1],
        [current_x_2, current_y_2],
        [current_x_3, current_y_3]
        """
        self.T0_pos.append(T0)
        self.T1_pos.append(T1)
        self.T2_pos.append(T2)
        self.T3_pos.append(T3)

    def save_four_pos(self):

        self.uwbs_pos = np.column_stack([self.T0_pos, self.T1_pos, self.T2_pos, self.T3_pos])
        with open(self.uwbs_pos_directory, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['T0_x', 'T0_y', 'T1_x', 'T1_y', 'T2_x', 'T2_y', 'T3_x', 'T3_y'])      # 写入表头
            writer.writerows(self.uwbs_pos)             # 写入坐标行
    
    def plt_four_pos(self):
        uwbs_df = pd.read_csv(self.uwbs_pos_directory)
        T0_x = uwbs_df['T0_x'].tolist()           # 转成 Python 列表
        T0_y = uwbs_df['T0_y'].tolist()
        T1_x = uwbs_df['T1_x'].tolist()           # 转成 Python 列表
        T1_y = uwbs_df['T1_y'].tolist()
        T2_x = uwbs_df['T2_x'].tolist()           # 转成 Python 列表
        T2_y = uwbs_df['T2_y'].tolist()
        T3_x = uwbs_df['T3_x'].tolist()           # 转成 Python 列表
        T3_y = uwbs_df['T3_y'].tolist()

        plt.title('UWBs\' position in time',fontsize=20)

        colors = ["#1F77B4", "#FF7F0E", "#2CA02C", "#D62728"]   # 经典四色组合（高对比度）
        # colors = ["#2E5BFF", "#FF8C00", "#A3A3A3", "#00CC66"]  # 科技蓝+活力橙+极简灰+清新绿 
        # plt.scatter(ekf_x, ekf_y, color='r', label = 'ekf', alpha=0.3)
        plt.scatter(T0_x, T0_y, color=colors[0], label = 'T0', alpha=0.3)
        plt.scatter(T1_x, T1_y, color=colors[1], label = 'T1', alpha=0.3)
        plt.scatter(T2_x, T2_y, color=colors[2], label = 'T2', alpha=0.3)
        plt.scatter(T3_x, T3_y, color=colors[3], label = 'T3', alpha=0.3)
        plt.xlabel('x (m)',fontsize=20)
        plt.ylabel('y (m)',fontsize=20)
        plt.grid()
        plt.axis('equal')
        plt.legend()
        plt.show()

    def save_test(self):

        arr_custom = np.random.uniform(low=5,  high=10, size=(4,2))

        font = font_manager.FontProperties(family='Times New Roman', weight='bold',style='normal', size=20)

        # plot x
        plt.figure(figsize=(12,8))
        plt.plot(arr_custom[:,0], arr_custom[:,1], color='r', linewidth=5)
        plt.title('car\'s x in time',fontsize=20)
        plt.xlabel('time (s)',fontsize=20)
        plt.ylabel('x (m)',fontsize=20)
        plt.axis('equal')
        plt.grid()
        plt.legend(['reference', 'car\'s x'], prop=font) # using a named size
        # 智能边距压缩（自适应策略）
        plt.tight_layout(pad=0.5)   # pad控制整体边距
        plt.savefig(os.path.join(self.directory,  'test.png'),  
                bbox_inches='tight',  # 边界框紧贴内容 
                pad_inches=0.05)      # 保留5%的安全边距 
        # plt.savefig(os.path.join(self.directory,  'test.png'))

def main():
    mylog = DataLogger()

    # mylog.log((0., 1.), 2, 3, 4, (5.,6.))
    # mylog.log((0., 5.), 2, 3, 4, (7.,8.))
    # mylog.log((0., 9.), 2, 3, 4, (9.,10.))
    # mylog.log((5.,6.), 2, 3, 4, (91,11.))

    mylog.log_four_tags([0.,0.],[0.5,0.5],[0.,0.],[0.,0.])
    mylog.log_four_tags([1.,1.],[1.5,1.5],[0.,0.],[0.,0.])
    mylog.log_four_tags([2.,2.],[2.5,2.5],[0.,0.],[0.,0.])
    mylog.log_four_tags([3.,3.],[3.5,3.5],[0.,0.],[0.,0.])


    mylog.log_uwb([0.,0.])
    mylog.log_uwb([1.,2.])
    mylog.save_pos_test()

    # print(mylog.ekf_pos)
    # mylog.save_four_pos()
    # mylog.plt_pos()
    # mylog.plt_four_pos()

    # mylog.save_test()

if __name__ == '__main__':
    main()
