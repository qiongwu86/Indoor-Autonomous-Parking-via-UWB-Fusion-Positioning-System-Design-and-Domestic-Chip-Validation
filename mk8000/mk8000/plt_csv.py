import pandas as pd
import matplotlib.pyplot  as plt 
import ctypes
from moving_average import MovingAverage2D
import numpy as np

# 在泊车系统中的定位偏置
offset_x = 13.559322
offset_y = 26.271186

class UWBMsg(ctypes.Structure):
    _fields_ = [
        ('x', ctypes.c_double),
        ('y', ctypes.c_double),
        ('z', ctypes.c_double),
    ]
UWBMsg = UWBMsg

# 初始化 anchor 与 distance 数组
anchor_array = (UWBMsg * 8)()

# # 基站坐标        # TODO:
# anchor_array[0].x, anchor_array[0].y, anchor_array[0].z = 0.0, 0.0, 1.804
# anchor_array[1].x, anchor_array[1].y, anchor_array[1].z = 9.9, 0.0, 1.8
# anchor_array[2].x, anchor_array[2].y, anchor_array[2].z = 4.76, 10.63, 1.852

# 基站坐标      泊车版
anchor_array[0].x, anchor_array[0].y, anchor_array[0].z = 0.0, 0.0, 1.84
anchor_array[1].x, anchor_array[1].y, anchor_array[1].z = 14.25, 0.0, 1.84
anchor_array[2].x, anchor_array[2].y, anchor_array[2].z = 9.8, 13.73, 1.82

# for i in range(3):
#     # print(i)
#     anchor_array[i].x += offset_x
#     anchor_array[i].y += offset_y

plt.figure(figsize=(8, 8))
# # 绘制基站
# plt.scatter([anchor_array[0].x,anchor_array[1].x,anchor_array[2].x],  [anchor_array[0].y,anchor_array[1].y,anchor_array[2].y], 
#             marker='^',       # 设置三角形标记 
#             s=80,            # 控制标记大小 
#             c='#FF6B6B',     # 设置填充颜色 
#             edgecolors='r',  # 设置边框颜色 
#             linewidths=1,
#             label = 'anchor')    # 边框粗细 

# 显示基站坐标
# plt.text(anchor_array[0].x, anchor_array[0].y, f'{anchor_array[0].x, anchor_array[0].y}', 
#         #  fontdict=None, 
#          ha='center', 
#          va='bottom', 
#          rotation=0, 
#         #  transform=None, 
#          fontsize=14)
# plt.text(anchor_array[1].x, anchor_array[1].y, f'{anchor_array[1].x, anchor_array[1].y}', 
#         #  fontdict=None, 
#          ha='center', 
#          va='bottom', 
#          rotation=0, 
#         #  transform=None, 
#          fontsize=14)
# plt.text(anchor_array[2].x, anchor_array[2].y, f'{anchor_array[2].x, anchor_array[2].y}', 
#         #  fontdict=None, 
#          ha='center', 
#          va='bottom', 
#          rotation=0, 
#         #  transform=None, 
#          fontsize=14)

# uwb_df = pd.read_csv('/home/www/test_ws/src/datalog/mk8000_pos_9.csv')                          # TODO:
uwb_df = pd.read_csv('/home/www/auto_parking/auto_parking_ws/better/5-9-8/uwb_pos.csv')        # 泊车版

# uwb_x = uwb_df['x'].tolist()           # 转成 Python 列表
# uwb_y = uwb_df['y'].tolist()
uwb_x = uwb_df['uwb_x'].tolist()           # 转成 Python 列表
uwb_y = uwb_df['uwb_y'].tolist()

# 平滑
ma = MovingAverage2D(window_size=5)
poses = np.column_stack([uwb_x,uwb_y])
smooth_pos_list = []
# 模拟实时接收位置
for raw_pos in poses:
    smooth_pos = ma.update(raw_pos)
    smooth_pos_list.append(smooth_pos)
    # print(f"原始：{raw_pos}，平滑后：{smooth_pos}")

smooth_pos_list = np.asarray(smooth_pos_list)

# 在单图中叠加显示两种轨迹 
ax3 = plt.subplot(111) 
# # uwb 初始数据
ax3.scatter(uwb_x, uwb_y, c='b', label='uwb', alpha=0.5)
ax3.plot(uwb_x,  uwb_y, 'b--', lw=1)  # 增加轨迹连线 
# 平滑滤波
# ax3.scatter(smooth_pos_list[:,0],  smooth_pos_list[:,1], c='r', label='aver', alpha=0.5)
# ax3.plot(smooth_pos_list[:,0],  smooth_pos_list[:,1], 'r--', lw=1)  # 增加轨迹连线 

plt.axis('equal')
plt.legend()

# 动态调整布局 
plt.tight_layout(pad=4.0) 
plt.show() 