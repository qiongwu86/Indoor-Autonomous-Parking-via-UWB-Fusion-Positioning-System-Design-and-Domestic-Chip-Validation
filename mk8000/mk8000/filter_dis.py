import numpy as np
import matplotlib.pyplot as plt


class KalmanFilter:
    def __init__(self, max_anchors=10, max_tags=10, Q=0.018, R=0.542):
        """
        初始化卡尔曼滤波器
        :param max_anchors: 最大锚点数量（类似基站）
        :param max_tags:    最大标签数量（类似移动设备）
        :param Q:          过程噪声协方差（系统模型不确定性）
        :param R:          测量噪声协方差（传感器误差）
        """
        self.Q = Q
        self.R = R
        # 状态数组：shape=[锚点][标签][x_last, p_last]
        # 最优值，最优协方差
        self.state = np.zeros((max_anchors, max_tags, 2), dtype=np.float32)

    def filter(self, measurement, anc_idx, tag_idx):
        """
        执行卡尔曼滤波
        :param measurement: 当前测量值（如距离、速度等）
        :param anc_idx:     锚点索引
        :param tag_idx:     标签索引
        :return:            滤波后的最优估计值
        """
        # 首次测量初始化
        if self.state[anc_idx, tag_idx, 0] == 0:
            self.state[anc_idx, tag_idx, 0] = measurement  # x_last 
            self.state[anc_idx, tag_idx, 1] = measurement  # p_last

        # 预测步骤
        x_mid = self.state[anc_idx, tag_idx, 0]  # x_last -> x_pred
        p_mid = self.state[anc_idx, tag_idx, 1] + self.Q  # p_last + Q

        # 更新步骤
        kg = p_mid / (p_mid + self.R)  # 卡尔曼增益
        x_now = x_mid + kg * (measurement - x_mid)  # 最优估计
        p_now = (1 - kg) * p_mid  # 更新协方差

        # 保存状态
        self.state[anc_idx, tag_idx, 0] = x_now
        self.state[anc_idx, tag_idx, 1] = p_now

        return x_now

if __name__ == '__main__':
    # 1. 创建滤波器实例（假设系统含5个锚点、20个标签）
    kf = KalmanFilter(max_anchors=3, max_tags=1, Q=0.018, R=0.542)

    measurements = np.random.normal(72, 5, 100)  # 均值为72，标准差为5的高斯噪声

    # 2. 模拟连续测量（锚点0与标签3的测距数据）
    # measurements = [150, 152, 149, 155, 148]
    # measurements+=measurements
    # measurements = [180, 150, 150, 150, 150, 150, 150,]
    filtered_list=[]
    for idx, val in enumerate(measurements):
        filtered_val = kf.filter(val, anc_idx=3, tag_idx=0)
        filtered_list.append(filtered_val)
        print(f"Step {idx + 1}: Raw={val}, Filtered={filtered_val:.2f}")

    # 可视化
    plt.plot(measurements, 'b', label='Measurements')
    plt.plot(filtered_list, 'y', linewidth=2, label='Kalman Filter')
    plt.legend()
    plt.show()
