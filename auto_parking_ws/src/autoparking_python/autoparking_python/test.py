

"""
测试ekf
"""
import numpy as np
from my_ekf import *

import csv
import pandas as pd

offset_x = 13.559322
offset_y = 10.153

current_ekf_x = 0.
current_ekf_y = 0.
current_ekf_yaw = 0.    
ekf = ExtendedKalmanFilter(
    process_noise=np.array([0.00,0.00,0.000]),  # 运动噪声
    sensor_noise=np.array([0.00,0.00,0.00]),    # 传感器噪声
    Q_scale=1.0,                                # 模型协方差
    R_scale=1.0,                                 # 传感器协方差
    x=0.,y=0.,yaw=0.
)


ekf_df = pd.read_csv("/home/www/auto_parking/auto_parking_ws/datalog/ekf_pos.csv")
ekf_x = ekf_df['ekf_x'].tolist()           # 转成 Python 列表
ekf_y = ekf_df['ekf_y'].tolist()

uwb_df = pd.read_csv("/home/www/auto_parking/auto_parking_ws/datalog/uwb_pos.csv")
uwb_x = uwb_df['uwb_x'].tolist()           # 转成 Python 列表
uwb_y = uwb_df['uwb_y'].tolist()
# print(type(uwb_x))
uwb_x = np.asarray(uwb_x) - offset_x
uwb_y = np.asarray(uwb_y) - offset_y
# print(type(uwb_x))
# measurements = np.column_stack([uwb_x,uwb_y])
measurements = [(x, 1) for x in range(0, 101)]
measurements = np.asarray(measurements)

x_list = []
y_list = []
for z in measurements:
    # print(z)
    # ekf滤波
    control_inputs = np.array([1.0,  0.0])     # 取上一时刻的控制量
    linear_k_minus_1 = 1.0
    angular_k_minus_1 = 0.0
    model_estimate_history = ekf.predict(control_inputs,  delta_t=0.1)         # 由运动模型计算出的状态    # TODO:修改delta_t为uwb定位间隔
    z = np.asarray([z[0],z[1],current_ekf_yaw])    # 输入传感器的观测值
    state_estimate_ekf = ekf.update(z)                                         # 由ekf融合后的状态
    current_ekf_x = state_estimate_ekf[0]
    current_ekf_y = state_estimate_ekf[1]+offset_y
    current_ekf_yaw = state_estimate_ekf[2]
    x_list.append(current_ekf_x)
    y_list.append(current_ekf_y)

plt.title('car\'s position in time',fontsize=20)
# plt.scatter(ekf_x, ekf_y, color='r', label = 'ekf', alpha=0.3)
# plt.scatter(uwb_x, uwb_y, color='b', label = 'uwb', alpha=0.3)
plt.scatter(measurements[:,0], measurements[:,1], color='b', label = 'uwb', alpha=0.3)
plt.scatter(x_list, y_list, color='r', label = 'ekf', alpha=0.3)
plt.xlabel('x (m)',fontsize=20)
plt.ylabel('y (m)',fontsize=20)
plt.grid()
plt.axis('equal')
plt.legend()
plt.show()