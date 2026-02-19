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

    ekf_df = pd.read_csv('/home/www/test_ws/src/mk8000/mk8000/ekf_pos.csv')
    ekf_x = ekf_df['ekf_x'].tolist()           # 转成 Python 列表
    ekf_y = ekf_df['ekf_y'].tolist()

    uwb_df = pd.read_csv('/home/www/test_ws/src/mk8000/mk8000/uwb_pos.csv')
    uwb_x = uwb_df['uwb_x'].tolist()           # 转成 Python 列表
    uwb_y = uwb_df['uwb_y'].tolist()

    poses = np.column_stack([uwb_x,uwb_y])

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
    plt.plot(poses[:,0],poses[:,1], '.b', label='Measurements')
    plt.plot(smooth_pos_list[:,0],smooth_pos_list[:,1], '.r', linewidth=2, label='moving_average_filter')
    # plt.plot(smooth_pos_list_2[:,0],smooth_pos_list_2[:,1], '.y', linewidth=2, label='moving_average_filter')
    plt.legend()
    plt.show()

if __name__ == '__main__':
    main()


