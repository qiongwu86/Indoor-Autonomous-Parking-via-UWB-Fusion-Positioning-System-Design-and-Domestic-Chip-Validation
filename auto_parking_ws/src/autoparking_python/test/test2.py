"""
记录停车场中所需的汽车信息
"""
from geometry_msgs.msg import Pose2D

class Vehicle: 
    def __init__(self, car_name, start_time, initial_pose): 

        self.car_name  = car_name 
        self.start_time  = start_time 
        self.initial_pose  = initial_pose 
        self.current_pose  = initial_pose 

    # def __str__(self): 
        # return f"车牌号: {self.license_plate},  入场时间: {self.entry_time},  停车位置: {self.parking_space}"  
    
    def update(self, x, y): 
        current_pose = Pose2D()
        current_pose.x = x
        current_pose.y = y
        self.current_pose = current_pose
        # print("this is test")


# parking_car = {} 
# pose1 = Pose2D()
# pose2 = Pose2D()
# pose1.x = 1.0
# pose2.y = 2.0

# vehicle1 = Vehicle("ABC123", "2025-03-03 10:00:00", pose1) 
# vehicle2 = Vehicle("DEF456", "2025-03-03 11:30:00", pose2) 

# parking_car["ABC123"] = vehicle1
# parking_car["DEF456"] = vehicle2

# print(parking_car["ABC123"].current_pose) 

# pose = Pose2D()
# pose.x=6.6
# pose.y=9.9
# # vehicle1.current_pose = pose
# parking_car["ABC123"].update(6.6,9.6)

# print(parking_car["ABC123"].current_pose) 

