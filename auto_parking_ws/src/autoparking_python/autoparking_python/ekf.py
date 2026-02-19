import numpy as np 
import matplotlib.pyplot  as plt 
from scipy.stats import chi2

# 抑制打印NumPy数组时的科学计数法 
np.set_printoptions(precision=3,suppress=True)

def normalize_angle_to_0_360(angle_deg):
    angle_deg = np.asarray(angle_deg)
    normalized_angle_deg = angle_deg % 360
    if normalized_angle_deg < 0:
        normalized_angle_deg = normalized_angle_deg + 360

    return normalized_angle_deg

class EKF:
    """
    
    特性：
    - 支持动态噪声参数配置 
    - 内置状态协方差矩阵自动校正 
    - 提供可视化接口 
    - 支持噪声相关性配置 
    
    初始化参数：
        state_dim (int): 状态空间维度，默认为3(x,y,yaw)
        control_dim (int): 控制输入维度，默认为2(速度, 转向速率)
        process_noise (np.ndarray):  过程噪声标准差向量 
        sensor_noise (np.ndarray):  传感器噪声标准差向量 
        Q_scale (float): 过程噪声协方差缩放因子，默认1.0 
        R_scale (float): 观测噪声协方差缩放因子，默认1.0 
    """
    
    def __init__(self, state_dim=3, control_dim=2,
                 process_noise=np.array([0.01,0.01,0.003]), 
                 sensor_noise=np.array([0.07,0.07,0.04]), 
                 Q_scale=1.0, R_scale=1.0,
                 x=0.0,y=0.0,yaw=0.0):
        # 维度配置 
        self.state_dim  = state_dim 
        self.control_dim  = control_dim 
        
        # 核心矩阵初始化 
        self.A = np.eye(state_dim)   # 状态转移矩阵 
        self.H = np.eye(state_dim)   # 观测矩阵 
        
        # 噪声参数配置 
        self.process_noise_std  = process_noise 
        self.sensor_noise_std  = sensor_noise 
        self.Q = Q_scale * np.eye(state_dim)   # 过程噪声协方差 
        self.R = R_scale * np.eye(state_dim)   # 观测噪声协方差 
        self.R_init = self.R.copy()
        
        # 状态初始化 
        self.state_estimate  = np.array([x,y,yaw]) 
        self.P = np.diag([1000]*state_dim)   # 协方差矩阵 
        
        # 历史轨迹记录 
        self.estimate_history  = []
        self.model_estimate_history  = []
        self.measurement_history  = []
        
    def _compute_B_matrix(self, yaw, delta_t):
        """
        计算控制输入矩阵(私有方法)
        
        参数:
            yaw (float): 当前偏航角(弧度)
            delta_t (float): 采样时间间隔(秒)
            
        返回:
            B (np.ndarray):  控制输入矩阵 
        """
        B = np.array([ 
            [np.cos(yaw)*delta_t, 0],
            [np.sin(yaw)*delta_t, 0],
            [0, delta_t]
        ])
        return B 
        
    def predict(self, control_input, delta_t):
        """
        执行预测步骤 
        
        参数:
            control_input (np.ndarray):  控制输入向量 
            delta_t (float): 时间间隔(秒)
        """
        # 生成过程噪声 
        process_noise = np.random.normal(0,  self.process_noise_std) 
        
        # 计算控制矩阵 
        B = self._compute_B_matrix(self.state_estimate[2],  delta_t)
        
        # 状态预测 
        state_estimate  = self.A @ self.state_estimate  + B @ control_input + process_noise 
        state_estimate[2] = normalize_angle_to_0_360(state_estimate[2]/3.1415926*180)/180*3.1415926
        self.state_estimate = state_estimate
        # self.state_estimate[2] = normalize_angle_to_0_360(state_estimate[2])
        # 协方差预测 
        self.P = self.A @ self.P @ self.A.T + self.Q 
        
        return state_estimate

    # GPT R
    def update(self, measurement):
        """
        执行观测更新步骤 
        
        参数:
            measurement (np.ndarray):  传感器测量值 
        """
        # 生成传感器噪声 
        sensor_noise = np.random.normal(0,  self.sensor_noise_std) 
        
        # 计算残差 
        y = measurement - (self.H @ self.state_estimate  + sensor_noise)
        innovation = y
        # if abs((y[0]**2+y[1]**2)**0.5) > 0.5:
        #     print("[IAKF] Observation suspicious. Inflating R.")
        #     self.R *= 200  # 或者 self.R += X * I

        # 卡尔曼增益 & 状态更新
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.pinv(S)
        # self.state_estimate += (K @ y)
        state_estimate  = self.state_estimate + K @ y 
        state_estimate[2] = normalize_angle_to_0_360(state_estimate[2]/3.1415926*180)/180*3.1415926
        self.state_estimate = state_estimate
        self.P = (np.eye(self.state_dim) - K @ self.H) @ self.P

        # # 自适应恢复原始R（平滑）
        # # self.R = 0.1 * self.R + 0.9 * self.R_init
        # # 自适应权重
        # residual_norm = np.linalg.norm(innovation)
        # # 将残差转换为 [0, 1] 的置信系数 α，越大越可信
        # alpha = np.clip(1.0 - residual_norm, 0.01, 0.99)
        # print(alpha)
        # # 用 α 调整恢复速度（α 越大，恢复越快）
        # self.R = alpha * self.R + (1 - alpha) * self.R_init

        return state_estimate

    def visualize(self):
        """生成轨迹对比可视化图表"""
        estimates = np.array(self.estimate_history) 
        measurements = np.array(self.measurement_history) 
        model_estimate_history = np.array(self.model_estimate_history)
        # print("6666666666,",estimates)
        # print("7777777777,",model_estimate_history)
        plt.figure(figsize=(10,  6))
        plt.plot(measurements[:,0],  measurements[:,1], '-bo', label='measure')
        plt.plot(estimates[:,0],  estimates[:,1], '-rx', label='EKF')
        plt.plot(model_estimate_history[:,0],  model_estimate_history[:,1], '-g', label='model')
        # plt.title(' 移动机器人轨迹估计对比')
        plt.xlabel('X(m)')
        plt.ylabel('Y(m)')
        plt.grid(True) 
        plt.legend() 
        # plt.axis('equal') 
        plt.show()    

def generate_measurements(base_speed=0.5, total_sec=25):
    """
    生成符合运动学模型的增强测量数据 
    
    参数:
        base_speed (float): 基准速度 (m/s)
        total_sec (int): 总观测时长 (秒)
        
    返回:
        np.ndarray:  Nx3维测量矩阵 
    """
    measurements = []
    for t in range(1, total_sec+1):
        # 基础运动模型 
        ideal_x = base_speed * t 
        
        # 复合噪声注入 
        x_noise = 0.1 * np.random.randn()  + 0.02 * np.sin(t/3)   # 周期干扰 
        y_noise = 0.07 * (1 + 0.3 * (-1)**(t//5)) * np.random.randn()   # 脉冲混合噪声 
        yaw_noise = 0.003 * t**0.5 * np.random.randn()   # 累积型角度噪声 
        
        measurements.append([ 
            ideal_x + x_noise,
            y_noise,
            yaw_noise 
        ])
    return np.array(measurements).round(3) 
def generate_drift_points(base_trajectory):
    """
    生成复合型漂移点的高级算法 
    参数:
        base_trajectory (np.ndarray):  基准轨迹Nx3矩阵 
    返回:
        np.ndarray:  注入漂移点后的轨迹 
    """
    # 漂移类型配置 
    drift_config = {
        'impulse': {'axis': 1, 'magnitude': 0.5, 'positions': [5,15]},  # Y轴瞬时脉冲 
        'cumulative': {'axis': 0, 'slope': 0.3, 'start': 10},          # X轴累积偏移 
        'oscillation': {'axis': 2, 'amp': 0.2, 'freq': 0.5}           # 航向角振荡 
    }
    
    modified = base_trajectory.copy() 
    # 注入瞬时脉冲漂移 
    for pos in drift_config['impulse']['positions']:
        modified[pos,1] += drift_config['impulse']['magnitude'] * (-1)**pos 
        
    # 添加累积性漂移 
    for t in range(drift_config['cumulative']['start'], len(modified)):
        modified[t,0] += drift_config['cumulative']['slope']*(t-drift_config['cumulative']['start'])
        
    # 叠加振荡漂移 
    modified[:,2] += drift_config['oscillation']['amp'] * np.sin( 
        drift_config['oscillation']['freq']*np.arange(len(modified))) 
        
    return np.round(modified,  3)


# 示例用法 
if __name__ == "__main__":
    # 初始化滤波器 
    ekf = EKF(
        process_noise=np.array([0.00,0.00,0.000]), 
        sensor_noise=np.array([0.00,0.00,0.00]), 
        Q_scale=1.0,                                # 模型协方差
        R_scale=3.0,                                # 传感器协方差
        # x=99.0,y=99.0,yaw=0.0
    )
    
    # 模拟控制输入和观测数据 
    control_inputs = np.array([0.3,  0.0])
    # measurements = np.array([ 
    #     [4.721,0.143,0.006], [9.353,0.284,0.007],
    #     [14.773,0.122,0.009], [18.246,0.255,0.011],
    #     [22.609,0.115,0.012], [27.109,0.215,0.008],
    #     [31.609,0.115,0.009], [36.109,0.215,0.008],
    #     [40.609,0.115,0.005],
    # ])
    measurements = generate_measurements(base_speed=0.3, total_sec=100)
    measurements = generate_drift_points(measurements)
    # 执行滤波流程 
    for z in measurements:
        # print(type(z))
        model_estimate_history = ekf.predict(control_inputs,  delta_t=1.0)  # 由运动模型计算出的状态
        # print("6666666666,",model_estimate_history)
        # ekf.model_estimate_history.append(model_estimate_history.copy())         
        # 内存优化版：使用np.copy 代替原生copy()
        
        state_estimate_ekf = ekf.update(z)                                  # 由ekf融合后的状态
        # print("7777777777,",state_estimate_ekf[0])
        # print(state_estimate)

        # 记录历史数据 
        ekf.model_estimate_history.append(np.copy(model_estimate_history))
        ekf.estimate_history.append(ekf.state_estimate.copy()) 
        ekf.measurement_history.append(z.copy()) 
        
    # print(len(ekf.estimate_history))
    # 可视化结果 
    ekf.visualize() 


    ############################
    # a = generate_measurements(base_speed=4.5, total_sec=60)
    # print(type(a))