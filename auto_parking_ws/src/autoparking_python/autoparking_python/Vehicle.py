"""
记录停车场中所需的汽车信息
名称
开始时间
初始位姿
"""
from geometry_msgs.msg import Pose2D

class Vehicle: 
    def __init__(self, car_name, start_time, initial_pose): 

        self.car_name  = car_name               # 名称
        self.start_time  = start_time           # 开始时间
        self.initial_pose  = initial_pose       # 初始位姿
        self.current_pose  = initial_pose       # 当前位姿
        self.parking_space = 0
        
    # def __str__(self): 
        # return f"车牌号: {self.car_name},  入场时间: {self.start_time},  停车位置: {self.parking_space}"  
    
    def update(self, x, y): 
        current_pose = Pose2D()
        current_pose.x = x
        current_pose.y = y
        self.current_pose = current_pose
        # print("this is test")

