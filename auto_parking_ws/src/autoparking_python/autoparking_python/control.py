import numpy as np
from scipy.optimize import minimize
import copy
# import draw_arrow
# import math

mp_pi = 3.1415926
def normalize_angle_to_0_360(angle_deg):
    angle_deg = np.asarray(angle_deg)
    normalized_angle_deg = angle_deg % 360
    if normalized_angle_deg < 0:
        normalized_angle_deg = normalized_angle_deg + 360

    return normalized_angle_deg

def map_angle(theta):
    """
    将[0, 2π]区间的角度映射到[-π, π]区间
    
    参数:
        theta: 输入角度，单位为弧度，应在[0, 2π]范围内
        
    返回:
        映射后的角度，单位为弧度，在[-π, π]范围内
    """
    if not (0 <= theta <= 2 * mp_pi):
        raise ValueError("输入角度必须在[0, 2π]范围内")
    
    if theta <= mp_pi:
        return theta
    else:
        return theta - 2 * mp_pi
    
class Car_Dynamics:
    def __init__(self, x_0, y_0, v_0, psi_0, length, dt):
        self.dt = dt             # sampling time
        self.L = length          # vehicle length
        self.x = x_0
        self.y = y_0
        self.v = v_0
        self.psi = psi_0        # 航行角
        self.state = np.array([[self.x, self.y, self.v, self.psi]]).T
                                # delta 偏转角
    # def move(self, accelerate, delta):
    #     x_dot = self.v*np.cos(self.psi)
    #     y_dot = self.v*np.sin(self.psi)
    #     v_dot = accelerate
    #     psi_dot = self.v*np.tan(delta)/self.L
    #     return np.array([[x_dot, y_dot, v_dot, psi_dot]]).T
    def move(self, linear , angular):       # 差速
        """
        Args:
            linear:     线速度
            angular:    角速度

        Returns:

        """
        self.v = linear
        x_dot = self.v*np.cos(self.psi)
        y_dot = self.v*np.sin(self.psi)
        v_dot = 0
        psi_dot = angular
        return np.array([[x_dot, y_dot, v_dot, psi_dot]]).T

    def update_state(self, state_dot):
        # self.u_k = command
        # self.z_k = state
        self.state = self.state + self.dt*state_dot
        self.x = self.state[0,0]
        self.y = self.state[1,0]
        self.v = self.state[2,0]
        self.psi = np.deg2rad(normalize_angle_to_0_360(np.rad2deg(self.state[3,0])))

    def update_state_pose(self, x, y, v, psi, dt):
        # psi 弧度
        # self.u_k = command
        # self.z_k = state
        # self.state = self.state + self.dt*state_dot
        self.x = x
        self.y = y
        self.v = v
        self.psi = np.deg2rad(normalize_angle_to_0_360(np.rad2deg(psi)))
        self.dt=dt
        self.state[0,0] = self.x
        self.state[1,0] = self.y
        self.state[2,0] = self.v
        self.state[3,0] = self.psi

###################################################################
# 所使用的 MPC_Controller
class MPC_Controller1:
    def __init__(self):
        # self.horiz = None
        # self.R = np.diag([0.01, 0.01])                 # input cost matrix  输入成本矩阵，用于控制输入的代价
        # self.Rd = np.diag([0.1, 0.2])                 # input difference cost matrix   输入差异成本矩阵，用于控制输入变化的代价
        # self.Q = np.diag([1.0, 1.0])                   # state cost matrix  状态成本矩阵，用于控制状态的代价
        # self.Qf = self.Q                               # state final matrix 终端状态成本矩阵，这里假设与状态成本矩阵相同
        ###################################################################
        # 测试用
        self.R = np.diag([0.01, 0.01])                 # input cost matrix  输入成本矩阵，用于控制输入的代价
        self.Rd = np.diag([0.1, 10.0])                 # input difference cost matrix   输入差异成本矩阵，用于控制输入变化的代价
        self.Q = np.diag([0.7, 0.7])                   # state cost matrix  状态成本矩阵，用于控制状态的代价
        self.Qf = self.Q                               # state final matrix 终端状态成本矩阵，这里假设与状态成本矩阵相同
        ###################################################################

    def change_cost(self):
        self.R = np.diag([0.01, 0.01])                 # input cost matrix  输入成本矩阵，用于控制输入的代价
        self.Rd = np.diag([0.1, 10.0])                 # input difference cost matrix   输入差异成本矩阵，用于控制输入变化的代价
        self.Q = np.diag([0.7, 0.7])                   # state cost matrix  状态成本矩阵，用于控制状态的代价
        self.Qf = self.Q                               # state final matrix 终端状态成本矩阵，这里假设与状态成本矩阵相同

    def mpc_cost(self, u_k, my_car, points):
        # 计算给定控制输入序列u_k下的MPC成本
        mpc_car = copy.copy(my_car)                     # 复制当前车辆状态作为MPC模拟的起点
        # print(f'mpc_car{mpc_car.y}')
        u_k = u_k.reshape(self.horiz, 2).T              # 将控制输入序列重塑为(2, horiz)的矩阵，并转置为(horiz, 2)
        z_k = np.zeros((2, self.horiz+1))               # 初始化状态轨迹矩阵，多一列用于存储初始状态（虽然这里未使用）
    
        desired_state = points.T                        # 转换目标点矩阵为列向量形式
        cost = 0.0                                      # 初始化成本

        for i in range(self.horiz):
            state_dot = mpc_car.move(u_k[0,i], u_k[1,i])
            mpc_car.update_state(state_dot)
        
            z_k[:,i] = [mpc_car.x, mpc_car.y]
            cost += np.sum(self.R@(u_k[:,i]**2))
            cost += np.sum(self.Q@((desired_state[:,i]-z_k[:,i])**2))
            if i < (self.horiz-1):     
                cost += np.sum(self.Rd@((u_k[:,i+1] - u_k[:,i])**2))
        return cost

    def optimize(self, my_car, points):
        self.horiz = points.shape[0]
        bnd = [(-5, 5),(np.deg2rad(-30), np.deg2rad(30))]*self.horiz        # 约束
        result = minimize(self.mpc_cost, args=(my_car, points), x0 = np.zeros((2*self.horiz)), method='SLSQP', bounds = bnd)
        return result.x[0],  result.x[1]            # 差速下 0：linear；1：angular
    
###################################################################
# 优化后
class MPC_Controller:
    def __init__(self):
        self.horiz = None
        # self.R = np.diag([0.01, 0.01])                 # 控制输入惩罚
        # self.Rd = np.diag([0.1, 10.0])                  # 控制输入变化惩罚（更平滑）
        # self.Q = np.diag([0.7, 0.7])                 # 跟踪误差惩罚（更重视位置）
        # self.Qf = self.Q                               # 最终状态惩罚
        # self.Rp = np.diag([0.0, 0.0])                  # 第一个输入与前一帧输入差异惩罚

        self.R = np.diag([0.00, 0.00])                 # 控制输入惩罚
        self.Rd = np.diag([0.1, 10.0])                  # 控制输入变化惩罚（更平滑）10->20
        # self.Q = np.diag([0.7, 0.7])                 # 跟踪误差惩罚（更重视位置）       0.7->0.9 ->0.8->1.0
        self.Q = np.diag([0.8, 0.8])                 # 跟踪误差惩罚（更重视位置）       0.7->0.9 ->0.8->1.0
        self.Qf = self.Q                               # 最终状态惩罚
        self.Rp = np.diag([0.01, 0.01])                  # 第一个输入与前一帧输入差异惩罚

    def change_Rp(self):
        # NOLS下，改变矩阵参数
        # self.Q = np.diag([0.5, 0.5])                    # 降低 0.05->0.01
        self.Q = np.diag([0.6, 0.6])                    # 降低 0.05->0.01       mk8000

        # self.Rp = np.diag([10.0, 10.0])                   # 增加 4->15->20
        self.Rp = np.diag([0.5, 1.0])                   # 增加 4->15->20
        self.Rd = np.diag([0.01, 0.01])                  # 控制输入变化惩罚（更平滑）
        # self.R = np.diag([0.00, 10.00])                 # 控制输入惩罚 惩罚角速度
        self.R = np.diag([0.00, 5.00])                 # 控制输入惩罚 惩罚角速度    mk8000

    def recover_Rp(self):
        # LOS下，恢复矩阵参数
        self.Q = np.diag([0.8, 0.8])                    # 降低
        self.Rd = np.diag([0.1, 10.0])                  # 控制输入变化惩罚（更平滑）
        self.Rp = np.diag([0.01, 0.01])                   # 增加
        self.R = np.diag([0.00, 0.00])                 # 控制输入惩罚

    def mpc_cost(self, u_k, my_car, points, u_prev):
        mpc_car = copy.deepcopy(my_car)               # 完整拷贝车辆状态
        u_k = u_k.reshape(self.horiz, 2).T            # 转置为 (2, horiz)
        z_k = np.zeros((2, self.horiz + 1))
        desired_state = points.T
        cost = 0.0

        # 第一个输入与前一帧输入差异惩罚
        du0 = u_k[:, 0] - u_prev
        cost += du0.T @ self.Rp @ du0

        for i in range(self.horiz):
            state_dot = mpc_car.move(u_k[0, i], u_k[1, i])
            mpc_car.update_state(state_dot)
            z_k[:, i] = [mpc_car.x, mpc_car.y]

            # 跟踪误差
            e = desired_state[:, i] - z_k[:, i]
            cost += e.T @ self.Q @ e

            # 控制输入成本
            cost += u_k[:, i].T @ self.R @ u_k[:, i]

            # 输入变化成本
            if i < self.horiz - 1:
                du = u_k[:, i+1] - u_k[:, i]
                cost += du.T @ self.Rd @ du

        return cost

    def generate_initial_input(self, points, current_pose):
        """
        points: 目标轨迹点 [N, 2]
        current_pose: [x, y, psi] 当前车辆姿态
        返回：初始控制输入向量 [2 * horiz]
        """
        x0, y0, psi0 = current_pose
        psi0 = psi0 % (2 * mp_pi)
        u_init = np.zeros((self.horiz, 2))

        for i in range(self.horiz):
            dx = points[i, 0] - x0
            dy = points[i, 1] - y0
            angle_to_target = np.arctan2(dy, dx)
            angle_to_target = angle_to_target % (2 * mp_pi)  # 将[-π, π]转换为[0, 2π)
            angle_diff = (angle_to_target - psi0 ) % (2 * mp_pi)
            angle_diff = map_angle(angle_diff)
            dist = np.hypot(dx, dy)

            # 允许前进/后退，根据角度选择符号
            direction = 1.0 if abs(angle_diff) < mp_pi / 2 else -1.0
            low = 0.0 if abs(angle_diff) > mp_pi / 6  and abs(angle_diff) < mp_pi * 5 / 6 else 1.0  # 角度差大时减速
            v = low * direction * min(dist, 0.5)   # 限制最大初速度
            # 3. 角速度（关键改进：差速转向需根据线速度方向调整符号）
            if direction > 0:
                raw_w = np.clip(angle_diff, -np.deg2rad(30), np.deg2rad(30))  # 原始角速度
            else:
                # raw_w = np.clip(mp_pi - abs(angle_diff), 0, np.deg2rad(30))  # 原始角速度
                # if angle_diff > 0 :
                #         raw_w = -1.0*raw_w
                # else:
                #         pass
                raw_w = 0.
            w = raw_w
            
            u_init[i] = [v, w]

            # 模拟移动一小步，改进下一个点的方向判断
            dt_sim = 0.2
            x0 += v * np.cos(psi0) * dt_sim
            y0 += v * np.sin(psi0) * dt_sim
            psi0 += w * dt_sim
            psi0 = (psi0 + w * dt_sim) % (2 * mp_pi)  # 保持航向角范围

        return u_init.flatten()

    def optimize(self, my_car, points, u_prev=np.zeros(2)):
        self.horiz = points.shape[0]

        # 输入边界设置：线速度 [-1, 1]，角速度 [-40°, 40°]
        max_v = 1.0
        max_w = np.deg2rad(30)
        bnd = [(-max_v, max_v), (-max_w, max_w)] * self.horiz

        # 当前姿态
        current_pose = [my_car.x, my_car.y, my_car.psi]

        # 启发式初始值
        u_init = self.generate_initial_input(points, current_pose)
        # print(f'u_init:{u_init}')

        # 优化器求解
        result = minimize(self.mpc_cost,
                          args=(my_car, points, u_prev),
                          x0=u_init,
                          method='SLSQP',
                          bounds=bnd,
                        #   options={'maxiter': 100, 'ftol': 1e-2, 'disp': False},
                          )

        # 返回第一个控制指令（线速度、角速度）
        if result.success:
            # print(f'control:{result.x[0].tolist(),result.x[1].tolist()}')
            return result.x[0], result.x[1]
        else:
            # print("[Warning] MPC optimization failed:", result.message)
            return u_prev

###########
    # def optimize(self, my_car, points):
    #     self.horiz = points.shape[0]
    #     bnd = [(-5, 5),(np.deg2rad(-56), np.deg2rad(49))]*self.horiz        # 约束
    #     result = minimize(self.mpc_cost, args=(my_car, points), x0 = np.zeros((2*self.horiz)), method='SLSQP', bounds = bnd)
    #     return result.x[0],  result.x[1]            # 差速下 0：linear；1：angular

######################################################################################################################################################################

class Linear_MPC_Controller:
    def __init__(self):
        self.horiz = None
        self.R = np.diag([0.01, 0.01])                 # input cost matrix
        self.Rd = np.diag([1, 0.01])                 # input difference cost matrix
        self.Q = np.diag([10.0, 10.0])                   # state cost matrix
        self.Qf = self.Q                               # state final matrix
        self.dt=0.35
        self.L=2                                      ###### 轴距需要更改

    def make_model(self, v, psi, delta):        
        # matrices
        # 4*4
        A = np.array([[1, 0, self.dt*np.cos(psi)         , -self.dt*v*np.sin(psi)],
                    [0, 1, self.dt*np.sin(psi)         , self.dt*v*np.cos(psi) ],
                    [0, 0, 1                           , 0                     ],
                    [0, 0, self.dt*np.tan(delta)/self.L, 1                     ]])
        # 4*2 
        B = np.array([[0      , 0                                  ],
                    [0      , 0                                  ],
                    [self.dt, 0                                  ],
                    [0      , self.dt*v/(self.L*np.cos(delta)**2)]])

        # 4*1
        C = np.array([[self.dt*v* np.sin(psi)*psi                ],
                    [-self.dt*v*np.cos(psi)*psi                ],
                    [0                                         ],
                    [-self.dt*v*delta/(self.L*np.cos(delta)**2)]])
        
        return A, B, C

    def mpc_cost(self, u_k, my_car, points):
        
        u_k = u_k.reshape(self.horiz, 2).T
        z_k = np.zeros((2, self.horiz+1))
        desired_state = points.T
        cost = 0.0
        old_state = np.array([my_car.x, my_car.y, my_car.v, my_car.psi]).reshape(4,1)

        for i in range(self.horiz):
            delta = u_k[1,i]
            A,B,C = self.make_model(my_car.v, my_car.psi, delta)
            new_state = A@old_state + B@u_k + C
        
            z_k[:,i] = [new_state[0,0], new_state[1,0]]
            cost += np.sum(self.R@(u_k[:,i]**2))
            cost += np.sum(self.Q@((desired_state[:,i]-z_k[:,i])**2))
            if i < (self.horiz-1):     
                cost += np.sum(self.Rd@((u_k[:,i+1] - u_k[:,i])**2))
            
            old_state = new_state
        return cost

    def optimize(self, my_car, points):
        self.horiz = points.shape[0]            # 预测点数
        # bnd = [(-5, 5),(np.deg2rad(-60), np.deg2rad(60))]*self.horiz

        # bnd = [(-5, 5), (np.deg2rad(-60), np.deg2rad(60))] * self.horiz
        bnd = [(-5, 5), (np.deg2rad(-56), np.deg2rad(49))] * self.horiz
        # cons = ({'type': 'eq', 'fun': lambda x: x[0]})
        result = minimize(self.mpc_cost, args=(my_car, points), x0 = np.zeros((2*self.horiz)),
                          method='SLSQP', bounds = bnd,
                          # constraints=cons
                          )
        # print('rrrr',result.x[0],  result.x[1])
        return result.x[0],  result.x[1]
