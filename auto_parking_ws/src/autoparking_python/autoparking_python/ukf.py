import numpy as np
import matplotlib.pyplot as plt

class UKF_Localization:
    def __init__(self, alpha=1e-3, beta=2, kappa=0, dt=0.2, x=0.0,y=0.0,yaw=0.0):
        self.n = 3  # 状态维度: [x, y, theta]
        self.lmbda = alpha**2 * (self.n + kappa) - self.n
        self.dt = dt

        # UKF 权重
        self.Wm = np.full(2 * self.n + 1, 0.5 / (self.n + self.lmbda))
        self.Wc = np.copy(self.Wm)
        self.Wm[0] = self.lmbda / (self.n + self.lmbda)
        self.Wc[0] = self.lmbda / (self.n + self.lmbda) + (1 - alpha**2 + beta)

        # 初始状态
        # self.x = np.zeros(self.n)  # [x, y, theta]
        self.x = np.array([x,y,yaw])   # [x, y, theta]
        # self.state_estimate  = np.array([x,y,yaw]) 

        self.P = np.eye(self.n) * 1.0

        # 过程噪声协方差
        self.Q = np.diag([0.1, 0.1, np.deg2rad(1.0)])**2

        # 观测噪声协方差
        self.R = np.diag([0.5, 0.5, np.deg2rad(5.0)])**2

        # 历史轨迹记录
        self.trajectory = []

    def motion_model(self, x, u):
        v, omega = u
        theta = x[2]
        dx = v * np.cos(theta) * self.dt
        dy = v * np.sin(theta) * self.dt
        dtheta = omega * self.dt
        x_new = np.copy(x)
        x_new[0] += dx
        x_new[1] += dy
        x_new[2] = self.normalize_angle(x[2] + dtheta)
        return x_new

    def observation_model(self, x):
        return x  # 观测直接是状态 [x, y, theta]

    def normalize_angle(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def generate_sigma_points(self, x, P):
        sigma_points = np.zeros((2 * self.n + 1, self.n))
        sqrt_P = np.linalg.cholesky((self.n + self.lmbda) * P)
        sigma_points[0] = x
        for i in range(self.n):
            sigma_points[i + 1] = x + sqrt_P[i]
            sigma_points[self.n + i + 1] = x - sqrt_P[i]
        return sigma_points

    def predict(self, u):
        sigma_points = self.generate_sigma_points(self.x, self.P)
        sigma_points_pred = np.array([self.motion_model(sp, u) for sp in sigma_points])

        # 预测均值
        x_pred = np.dot(self.Wm, sigma_points_pred)
        x_pred[2] = self.normalize_angle(x_pred[2])

        # 预测协方差
        P_pred = np.zeros((self.n, self.n))
        for i in range(2 * self.n + 1):
            dx = sigma_points_pred[i] - x_pred
            dx[2] = self.normalize_angle(dx[2])
            P_pred += self.Wc[i] * np.outer(dx, dx)
        P_pred += self.Q

        self.x = x_pred
        self.P = P_pred
        self.trajectory.append(self.x.copy())

    def update(self, z):
        sigma_points = self.generate_sigma_points(self.x, self.P)
        sigma_points_obs = np.array([self.observation_model(sp) for sp in sigma_points])

        # 观测均值
        z_pred = np.dot(self.Wm, sigma_points_obs)
        z_pred[2] = self.normalize_angle(z_pred[2])

        # 观测协方差和交叉协方差
        Pz = np.zeros((self.n, self.n))
        Pxz = np.zeros((self.n, self.n))
        for i in range(2 * self.n + 1):
            dz = sigma_points_obs[i] - z_pred
            dz[2] = self.normalize_angle(dz[2])
            dx = sigma_points[i] - self.x
            dx[2] = self.normalize_angle(dx[2])
            Pz += self.Wc[i] * np.outer(dz, dz)
            Pxz += self.Wc[i] * np.outer(dx, dz)
        Pz += self.R

        # 卡尔曼增益
        K = Pxz @ np.linalg.inv(Pz)
        dz = z - z_pred
        dz[2] = self.normalize_angle(dz[2])

        # 状态更新
        self.x += K @ dz
        self.x[2] = self.normalize_angle(self.x[2])
        self.P -= K @ Pz @ K.T

    def get_state(self):
        return self.x.copy()

    def plot_trajectory(self):
        trajectory = np.array(self.trajectory)
        plt.figure(figsize=(8, 6))
        plt.plot(trajectory[:, 0], trajectory[:, 1], label='UKF Estimated Trajectory')
        plt.xlabel('x [m]')
        plt.ylabel('y [m]')
        plt.title('UKF Localization Trajectory')
        plt.axis('equal')
        plt.grid(True)
        plt.legend()
        plt.show()
