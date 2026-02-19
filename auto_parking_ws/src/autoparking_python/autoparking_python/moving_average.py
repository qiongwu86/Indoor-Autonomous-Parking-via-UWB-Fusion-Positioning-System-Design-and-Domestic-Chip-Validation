from collections import deque
import numpy as np
import matplotlib.pyplot  as plt 
import csv
import pandas as pd

# def moving_average_filter(positions, win_size=5):
#     """
#     positions: N×2 的 ndarray
#     win_size: 窗口大小
#     返回平滑后的位置序列
#     """
#     smoothed = []
#     buf = deque(maxlen=win_size)
#     for p in positions:
#         buf.append(p)
#         smoothed.append(np.mean(buf, axis=0))
#     return np.array(smoothed)

# # # 示例
# # poses = np.array([[0,0], [0.1,0.05], [0.2,0.1], [5,5], [0.3,0.15], [0.4,0.2]])
# # sm = moving_average_filter(poses, win_size=3)
# # print(sm)

# ekf_df = pd.read_csv('/home/www/test_ws/src/mk8000/mk8000/ekf_pos.csv')
# ekf_x = ekf_df['ekf_x'].tolist()           # 转成 Python 列表
# ekf_y = ekf_df['ekf_y'].tolist()

# uwb_df = pd.read_csv('/home/www/test_ws/src/mk8000/mk8000/uwb_pos.csv')
# uwb_x = uwb_df['uwb_x'].tolist()           # 转成 Python 列表
# uwb_y = uwb_df['uwb_y'].tolist()

# poses = np.column_stack([uwb_x,uwb_y])
# sm = moving_average_filter(poses, win_size=3)

# # 可视化
# plt.plot(poses[:,0],poses[:,1], '.b', label='Measurements')
# plt.plot(sm[:,0],sm[:,1], '.y', linewidth=2, label='moving_average_filter')
# plt.legend()
# plt.show()

######################################################################################################
import numpy as np
from collections import deque

class MovingAverage2D:
    def __init__(self, window_size: int):
        self.window_size = window_size
        self.buf = deque(maxlen=window_size)  # 自动丢弃最旧元素

    def update(self, new_point: np.ndarray) -> np.ndarray:
        """
        接收一个形状为 (2,) 的新位置，返回平滑后的位置 (2,)。
        """
        self.buf.append(new_point)
        # 直接对缓冲区中所有点取均值
        return np.mean(self.buf, axis=0)

def main():
    # 示例用法
    ma = MovingAverage2D(window_size=5)

    ekf_df = pd.read_csv('/home/www/auto_parking/auto_parking_ws/datalog/ekf_pos.csv')
    ekf_x = ekf_df['ekf_x'].tolist()           # 转成 Python 列表
    ekf_y = ekf_df['ekf_y'].tolist()

    uwb_df = pd.read_csv('/home/www/auto_parking/auto_parking_ws/datalog/uwb_pos.csv')
    uwb_x = uwb_df['uwb_x'].tolist()           # 转成 Python 列表
    uwb_y = uwb_df['uwb_y'].tolist()

    # poses = np.column_stack([uwb_x,uwb_y])
    poses = np.column_stack([ekf_x,ekf_y])

    smooth_pos_list = []
    # 模拟实时接收位置
    for raw_pos in poses:
        smooth_pos = ma.update(raw_pos)
        smooth_pos_list.append(smooth_pos)
        print(f"原始：{raw_pos}，平滑后：{smooth_pos}")

    smooth_pos_list = np.asarray(smooth_pos_list)
    # print(smooth_pos_list)

    # smooth_pos_list_2 = []
    # for raw_pos in smooth_pos_list:
    #     smooth_pos = ma.update(raw_pos)
    #     smooth_pos_list_2.append(smooth_pos)
    #     print(f"原始：{raw_pos}，平滑后：{smooth_pos}")
    # smooth_pos_list_2 = np.asarray(smooth_pos_list_2)

    # 可视化
    # plt.plot(poses[:,0],poses[:,1], '.b', label='Measurements')
    # plt.plot(smooth_pos_list[:,0],smooth_pos_list[:,1], '.r', linewidth=2, label='moving_average_filter')
    # plt.plot(ekf_x,ekf_y, '.r', linewidth=2, label='ekf')

    # plt.plot(smooth_pos_list_2[:,0],smooth_pos_list_2[:,1], '.y', linewidth=2, label='moving_average_filter')
    plt.legend()
    plt.show()

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

    # 在单图中叠加显示两种轨迹 
    ax3 = plt.subplot(111) 
    # ax3.scatter(smooth_pos_list[:,0],  smooth_pos_list[:,1], 'r', label='EKF', alpha=0.5)
    ax3.plot(smooth_pos_list[:,0],  smooth_pos_list[:,1], 'r', label='EKF', alpha=0.5)
    # ax3.scatter(poses[:,0],  poses[:,1], c='b', label='UWB', alpha=0.5)
    # ax3.plot(ekf_x,  ekf_y, 'r--', lw=1)  # 增加轨迹连线 
    # ax3.plot(uwb_x,  uwb_y, 'b:', lw=1)
    plt.axis('equal')

    # 动态调整布局 
    plt.tight_layout(pad=4.0) 
    plt.show() 

if __name__ == "__main__":
    main()