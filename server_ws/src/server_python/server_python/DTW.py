import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.distance import euclidean
from fastdtw import fastdtw
import pandas as pd
import os

def draw_dtw_alignment(ref_path, real_path, path_alignment,uwb_path):
    plt.figure(figsize=(10, 6))
    # 画轨迹
    plt.plot(ref_path[:,0], ref_path[:,1], 'o-', label='ref', color='blue')
    plt.plot(real_path[:,0], real_path[:,1], 'x--', label='true', color='red')
    plt.plot(uwb_path[:,0], uwb_path[:,1], 's--', label='uwb', color='green')
    
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

# # 横向误差（lateral error）和沿路径误差（along-track error，也叫路径进展误差）
# def compute_lateral_along_error(path_points, actual_point):
#     # path_points: Nx2 array
#     # actual_point: 1x2 array
    
#     min_dist = float('inf')
#     closest_idx = 0
#     for i in range(len(path_points)-1):
#         p1 = path_points[i]
#         p2 = path_points[i+1]
        
#         # 当前线段方向和长度
#         seg_vec = p2 - p1
#         seg_len = np.linalg.norm(seg_vec)
#         if seg_len == 0:
#             continue
#         seg_unit = seg_vec / seg_len
        
#         # 投影到线段
#         proj = np.dot(actual_point - p1, seg_unit)
#         proj_point = p1 + np.clip(proj, 0, seg_len) * seg_unit
#         dist = np.linalg.norm(actual_point - proj_point)
        
#         if dist < min_dist:
#             min_dist = dist
#             closest_idx = i
#             closest_proj = proj
#             closest_proj_point = proj_point
#             closest_tangent = seg_unit
    
#     # 误差向量
#     error_vec = actual_point - closest_proj_point
    
#     # 横向方向是切向的法向（顺时针旋转90度）
#     normal_vec = np.array([-closest_tangent[1], closest_tangent[0]])
    
#     lateral_error = np.dot(error_vec, normal_vec)
#     along_track_error = np.dot(error_vec, closest_tangent)
    
#     return lateral_error, along_track_error, closest_proj_point

# def plot_lateral_error_heatmap(path_points, actual_points, distance, normalized_dtw, date, cmap='viridis'):
#     """
#     绘制横向误差热力图

#     参数:
#         path_points: Nx2 理想路径点（参考轨迹）
#         actual_points: Mx2 实际轨迹点（有偏移的测量轨迹）
#         cmap: matplotlib colormap（默认 'viridis'）

#     返回:
#         None
#     """
#     lateral_errors = []
#     along_track_errors = []
#     x_vals = []
#     y_vals = []

#     for actual in actual_points:
#         lat_err, al_err, _ = compute_lateral_along_error(path_points, actual)
#         lateral_errors.append((lat_err))
#         along_track_errors.append(abs(al_err))
#         x_vals.append(actual[0])
#         y_vals.append(actual[1])

#     lateral_errors = np.array(lateral_errors)
#     along_track_errors = np.array(along_track_errors)

#     mean_lateral_error = np.mean(lateral_errors) 
#     mean_along_error = np.mean(along_track_errors) 
#     print(f"横向误差的平均值为: {mean_lateral_error}")
#     print(f"纵向误差的平均值为: {mean_along_error}")

#     plt.figure(figsize=(8, 6))
#     sc = plt.scatter(x_vals, y_vals, c=lateral_errors, cmap=cmap, s=60, edgecolors='k', alpha=0.5, linewidths= 0.0)
#     plt.plot(path_points[:, 0], path_points[:, 1], 'k--', linewidth=1.5, label='Reference Path')

#     plt.text(20.0, 43,              # 坐标位置（居中）
#          f"dtw:{distance:.3f}m\n n_dtw:{normalized_dtw:.3f}m",         # 标签文本 
#          ha='center', va='center', # 水平/垂直对齐 
#          fontsize=10,            # 字体大小 
#          bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.4)) # 背景框样式 
    
#     cbar = plt.colorbar(sc)
#     cbar.set_label('Lateral Error (m)', fontsize=12)
#     plt.title('Lateral Error Heatmap', fontsize=14)
#     plt.xlabel('X (m)')
#     plt.ylabel('Y (m)')
#     plt.axis('equal')
#     plt.grid(True, linestyle='--', alpha=0.5)
#     plt.legend()
#     plt.tight_layout()
#     # plt.show()

#     directory = "/home/www/auto_parking/server_ws/datalog"
#     directory_path = os.path.join(directory,  f'dtw-{date}.pdf')
#     plt.savefig(directory_path,  
#         bbox_inches='tight',  # 边界框紧贴内容 
#         pad_inches=0.05)      # 保留5%的安全边距 

# 修改热力图：
def compute_lateral_along_error(path_points, actual_point):
    min_dist = float('inf')
    closest_idx = 0
    for i in range(len(path_points) - 1):
        p1 = path_points[i]
        p2 = path_points[i + 1]
        seg_vec = p2 - p1
        seg_len = np.linalg.norm(seg_vec)
        if seg_len == 0:
            continue
        seg_unit = seg_vec / seg_len

        proj = np.dot(actual_point - p1, seg_unit)
        proj_point = p1 + np.clip(proj, 0, seg_len) * seg_unit
        dist = np.linalg.norm(actual_point - proj_point)

        if dist < min_dist:
            min_dist = dist
            closest_proj_point = proj_point
            closest_tangent = seg_unit
            closest_proj = proj

    error_vec = actual_point - closest_proj_point
    normal_vec = np.array([-closest_tangent[1], closest_tangent[0]])

    lateral_error = np.dot(error_vec, normal_vec)
    along_track_error = np.dot(error_vec, closest_tangent)
    euclidean_error = np.linalg.norm(error_vec)

    return lateral_error, along_track_error, euclidean_error, closest_proj_point

def plot_euclidean_error_heatmap(path_points, actual_points, distance, normalized_dtw, date, cmap='viridis'):
    euclidean_errors = []
    x_vals = []
    y_vals = []

    for actual in actual_points:
        _, _, eu_err, _ = compute_lateral_along_error(path_points, actual)
        euclidean_errors.append(eu_err)
        x_vals.append(actual[0])
        y_vals.append(actual[1])

    euclidean_errors = np.array(euclidean_errors)
    mean_eu_error = np.mean(euclidean_errors)
    max_eu_error = np.max(euclidean_errors)
    print(f"欧几里得误差平均值: {mean_eu_error:.3f} m, 最大值: {max_eu_error:.3f} m")

    plt.figure(figsize=(8, 6))
    vmin = 0    # 最小误差颜色对应
    vmax = 1.0  # 最大误差颜色对应（你可以根据实际调整）
    sc = plt.scatter(x_vals, y_vals, c=euclidean_errors, vmin=vmin, vmax=vmax, cmap=cmap, s=60, edgecolors='k', alpha=0.6, linewidths=0.0)
    plt.plot(path_points[:, 0], path_points[:, 1], 'k--', linewidth=1.5, label='Reference Path')

    # plt.text(20.0, 43,
    #      f"dtw:{distance:.3f}m\nn_dtw:{normalized_dtw:.3f}m",
    #      ha='center', va='center',
    #      fontsize=17,
    #      bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.4))
    
    cbar = plt.colorbar(sc)
    cbar.set_label('Euclidean Error (m)', fontsize=17)
    plt.title('Euclidean Error Heatmap', fontsize=17)
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.axis('equal')
    plt.grid(True, linestyle='--', alpha=0.5)
    plt.legend()
    plt.tight_layout()

    directory = "/home/www/auto_parking/server_ws/datalog"
    directory_path = os.path.join(directory, f'dtw-euclidean-{date}.pdf')
    plt.savefig(directory_path, bbox_inches='tight', pad_inches=0.05)

    plt.show()

def main():
    # directory_ref_path = "/home/www/auto_parking/server_ws/datalog/reference_path.csv"            #1

    # directory_real_path = "/home/www/auto_parking/auto_parking_ws/src/autoparking_python/autoparking_python/ekf_path.csv"
    # directory_real_path = "/home/www/auto_parking/auto_parking_ws/src/autoparking_python/autoparking_python/ukf_path.csv"
    # directory_real_path = "/home/www/auto_parking/auto_parking_ws/better/5-13-3/uwb_pos.csv"

    # directory_real_path = "/home/www/auto_parking/auto_parking_ws/better/5-9-1/uwb_pos.csv"      #2

    ############
    # ei
    date = "6-18-4"
    directory_ref_path = f"/home/www/auto_parking/auto_parking_ws/better/{date}/reference.csv"            #1
    directory_real_path = f"/home/www/auto_parking/auto_parking_ws/better/{date}/uwb_pos.csv"      #2

    # directory_real_path = f"/home/www/auto_parking/auto_parking_ws/better/{date}/odom.csv"      #2
    # uwb_dir = f"/home/www/auto_parking/auto_parking_ws/better/{date}/uwb.csv" 

    # new
    date = "7-9-3"
    uwb = 1                 # 0:UWB 1:AEKF 2:UKF 3:EKF
    directory_ref_path = f"/home/www/auto_parking/server_ws/src/server_python/server_python/reference_path.csv"       
    if uwb == 0:    
        directory_real_path = f"/home/www/auto_parking/auto_parking_ws/better/{date}/uwb_pos.csv"      
        real_path_df = pd.read_csv(directory_real_path) 
        uwb_x = real_path_df['uwb_x'].tolist()           # 转成 Python 列表
        uwb_y = real_path_df['uwb_y'].tolist() 
    elif uwb == 1:
        directory_real_path = f"/home/www/auto_parking/auto_parking_ws/src/autoparking_python/autoparking_python/ekf_path.csv"      
        real_path_df = pd.read_csv(directory_real_path) 
        uwb_x = real_path_df['ekf_x'].tolist()           # 转成 Python 列表
        uwb_y = real_path_df['ekf_y'].tolist() 
    elif uwb == 2:
        directory_real_path = f"/home/www/auto_parking/auto_parking_ws/src/autoparking_python/autoparking_python/ukf_path.csv"     
        real_path_df = pd.read_csv(directory_real_path)  
        uwb_x = real_path_df['ukf_x'].tolist()           # 转成 Python 列表
        uwb_y = real_path_df['ukf_y'].tolist() 
    else:
        directory_real_path = f"/home/www/auto_parking/auto_parking_ws/src/autoparking_python/autoparking_python/e_path.csv"     
        real_path_df = pd.read_csv(directory_real_path)  
        uwb_x = real_path_df['e_x'].tolist()           # 转成 Python 列表
        uwb_y = real_path_df['e_y'].tolist() 
    ##############

    # print(type(directory_ref_path))
    ref_path_df = pd.read_csv(directory_ref_path) 
    # real_path_df = pd.read_csv(directory_real_path) 
    # uwb_path_df = pd.read_csv(uwb_dir)                          # ei

    reference_x = ref_path_df['reference_x'].tolist()           # 转成 Python 列表
    reference_y = ref_path_df['reference_y'].tolist() 
    print(f'参考路径长度:{len(reference_x)}')
    reference_x = reference_x[:-6]
    reference_y = reference_y[:-6]

    # uwb_x = real_path_df['uwb_x'].tolist()           # 转成 Python 列表
    # uwb_y = real_path_df['uwb_y'].tolist() 
    # print(f'跟踪路径长度:{len(uwb_x)}')

    # uwb_x = real_path_df['odom_x'].tolist()           # 转成 Python 列表
    # uwb_y = real_path_df['odom_y'].tolist() 

    # x = uwb_path_df['x'].tolist()                     # ei
    # y = uwb_path_df['y'].tolist()

    # uwb_x = real_path_df['ekf_x'].tolist()           # 转成 Python 列表
    # uwb_y = real_path_df['ekf_y'].tolist() 

    # uwb_x = real_path_df['ukf_x'].tolist()           # 转成 Python 列表
    # uwb_y = real_path_df['ukf_y'].tolist() 
    
    ref_path = np.column_stack([reference_x,reference_y])
    real_path = np.column_stack([uwb_x,uwb_y])
    # uwb_path = np.column_stack([x,y])

    # # test
    # ref_path = np.array([
    #     [0, 0],
    #     [1, 1],
    #     [2, 2],
    #     [3, 3],
    #     [4, 4]
    # ])
    # real_path = ref_path + [0.0,1.0]

    # 计算 DTW 和匹配路径
    distance, path_alignment = fastdtw(ref_path, real_path, dist=euclidean)
    normalized_dtw = distance / len(path_alignment)
    print("DTW 距离:", distance)
    print("DTW 归一化距离:", normalized_dtw)


    # 可视化
    # draw_dtw_alignment(ref_path, real_path, path_alignment,uwb_path)


    # # err
    # timestamps = len(real_path)
    # time_series = np.linspace(0, timestamps,timestamps)
    # # print('real_path',len(real_path))
    # # print('ref_path',len(ref_path))
    # lat_err_list = []
    # along_err_list = []
    # for robot_pos in real_path:
    #     lat_err, along_err, near_pt = compute_lateral_along_error(ref_path, robot_pos)
    #     lat_err_list.append(lat_err)
    #     along_err_list.append(along_err)
    #     # print("横向误差:", lat_err)
    #     # print("沿路径误差:", along_err)
    # # print(lat_err_list)
    # # print(timestamps)
    # plt.figure(figsize=(10, 4))
    # plt.plot(time_series, lat_err_list, label='lat_err', color='blue')
    # plt.plot(time_series, along_err_list, label='along_err', color='green')
    # plt.xlabel("time")
    # plt.ylabel("err (m)")
    # # plt.title("路径追踪误差随时间变化")
    # plt.legend()
    # plt.grid(True)
    # # plt.tight_layout()
    # plt.show()

    # 热力图
    # plot_lateral_error_heatmap(ref_path, real_path, distance, normalized_dtw, date,"coolwarm")
    # 修改后
    plot_euclidean_error_heatmap(ref_path, real_path, distance, normalized_dtw, date,"coolwarm")

if __name__ == '__main__':
    main() 

