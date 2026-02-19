import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.distance import euclidean
from fastdtw import fastdtw

def draw_dtw_alignment(ref_path, real_path, path_alignment):
    plt.figure(figsize=(10, 6))
    # 画轨迹
    plt.plot(ref_path[:,0], ref_path[:,1], 'o-', label='ref', color='blue')
    plt.plot(real_path[:,0], real_path[:,1], 'x--', label='true', color='red')
    
    # 画对齐线
    for i, j in path_alignment:
        plt.plot([ref_path[i,0], real_path[j,0]], 
                 [ref_path[i,1], real_path[j,1]], 
                 color='gray', alpha=0.4, linewidth=0.7)
    
    plt.title("DTW")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.legend()
    plt.axis('equal')
    plt.grid(True)
    # plt.tight_layout()
    plt.show()

# 示例轨迹（可以替换为你自己的）
ref_path = np.array([
    [0, 0],
    [1, 1],
    [2, 2],
    [3, 3],
    [4, 4]
])

real_path = np.array([
    [0, 0],
    [0.8, 1.2],
    [2.1, 2.0],
    [3.2, 3.1]
])

# 计算 DTW 和匹配路径
distance, path_alignment = fastdtw(ref_path, real_path, dist=euclidean)
print("DTW 距离:", distance)

# 可视化
draw_dtw_alignment(ref_path, real_path, path_alignment)
