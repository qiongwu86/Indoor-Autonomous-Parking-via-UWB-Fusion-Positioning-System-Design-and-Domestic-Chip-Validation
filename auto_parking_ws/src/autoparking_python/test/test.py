import numpy as np
import math

def map_angle(theta):
    """
    将[0, 2π]区间的角度映射到[-π, π]区间
    
    参数:
        theta: 输入角度，单位为弧度，应在[0, 2π]范围内
        
    返回:
        映射后的角度，单位为弧度，在[-π, π]范围内
    """
    if not (0 <= theta <= 2 * math.pi):
        raise ValueError("输入角度必须在[0, 2π]范围内")
    
    if theta <= math.pi:
        return theta
    else:
        return theta - 2 * math.pi
    
data = {1:[False,8,'8-5'],
        2:[False,10,'8-6'],
        3:[True,8,'8-7'],}

dx, dy = -1,-1
dist = np.hypot(dx, dy)

angle_to_target = np.arctan2(dy, dx)
angle_to_target = angle_to_target % (2 * np.pi)  # 将[-π, π]转换为[0, 2π)
print(angle_to_target * 180 / np.pi)
psi0 = np.pi / 3 * 2 
# psi0 = np.pi / 6 * 5 
angle_diff = (angle_to_target - psi0 ) % (2 * np.pi)

print("angle_diff",angle_diff* 180 / np.pi)

angle_diff = map_angle(angle_diff)
print("angle_diff(after)",angle_diff* 180 / np.pi)


# 1. 线速度方向判断（前进/后退）
direction = 1.0 if abs(angle_diff) < np.pi / 2 else -1.0  # 与原逻辑一致

# 2. 线速度大小（考虑转弯减速）
low = 0.0 if abs(angle_diff) > np.pi / 6  and abs(angle_diff) < np.pi * 5 / 6 else 1.0  # 角度差大时减速
v = low * direction * min(dist, 0.5)  # 线速度 = 方向 × 大小

# 3. 角速度（关键改进：差速转向需根据线速度方向调整符号）
if direction > 0:
        raw_w = np.clip(angle_diff, -np.deg2rad(30), np.deg2rad(30))  # 原始角速度
else:
        raw_w = np.clip(np.pi - abs(angle_diff), 0, np.deg2rad(30))  # 原始角速度
        if angle_diff > 0 :
             raw_w = -1.0*raw_w
        else:
             pass
w = raw_w

print([v,w*180/np.pi])

a = [1,2,3,4,5]

print(a[-1])
