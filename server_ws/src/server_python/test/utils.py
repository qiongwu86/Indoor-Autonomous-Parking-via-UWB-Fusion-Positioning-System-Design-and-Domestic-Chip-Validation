import math
import numpy as np
import os
import matplotlib.pyplot as plt
import matplotlib.font_manager as font_manager

class DataLogger:
    def __init__(self):
        self.path = []
        self.car_state = []
        self.u = []
        self.yaw = []
        self.linear = []
        self.angular = []
   
    # def log(self, point, my_car, acc, delta):# 原始
    #     self.path.append(point*10)
    #     self.car_state.append([my_car.x*10, my_car.y*10, my_car.v*10, my_car.psi])
    #     self.u.append([acc, delta])
    
    # 记录车辆运动的数据：当前x，y，yaw，linear，angular
    def log(self, point, yaw, linear, angular):
        self.path.append(point)
        self.yaw.append(yaw)
        self.linear.append(linear)
        self.angular.append(angular)

    def save_data(self):
        os.makedirs('log results', exist_ok=True)
        t = np.arange(0,len(self.path),1)
        self.path = np.array(self.path)
        self.car_state = np.array(self.car_state)
        self.u = np.array(self.u)
        font = font_manager.FontProperties(family='Times New Roman', weight='bold',style='normal', size=20)

        # plot x
        plt.figure(figsize=(12,8))
        plt.plot(t, self.path[:,0], color='r', linewidth=5)
        # plt.plot(t, self.car_state[:,0], color='r', linewidth=4)
        plt.title('car\'s x in time',fontsize=20)
        plt.xlabel('time (s)',fontsize=20)
        plt.ylabel('x (m)',fontsize=20)
        plt.grid()
        plt.legend(['reference', 'car\'s x'], prop=font) # using a named size
        plt.savefig('log results/x.png')

        # plot y
        plt.figure(figsize=(12,8))
        plt.plot(t, self.path[:,1], color='r', linewidth=5)
        # plt.plot(t, self.car_state[:,1], color='r', linewidth=4)
        plt.title('car\'s y in time',fontsize=20)
        plt.xlabel('time (s)',fontsize=20)
        plt.ylabel('y (m)',fontsize=20)
        plt.grid()
        plt.legend(['reference', 'car\'s y'], prop=font) # using a named size
        plt.savefig('log results/y.png')

        # plot linear        
        plt.figure(figsize=(12,8))
        plt.plot(t, self.linear, color='r', linewidth=4)
        plt.title('car\'s speed in time',fontsize=20)
        plt.xlabel('time (s)',fontsize=20)
        plt.ylabel('v (m/s)',fontsize=20)
        plt.grid()
        plt.legend(['car speed (m/s)'], prop=font) # using a named size
        plt.savefig('log results/v.png')

        # plot yaw
        plt.figure(figsize=(12,8))
        plt.plot(t, np.rad2deg(self.yaw), color='r', linewidth=4)
        plt.title('car\'s yaw in time',fontsize=20)
        plt.xlabel('time (s)',fontsize=20)
        plt.ylabel('psi (degree)',fontsize=20)
        plt.grid()
        plt.legend(['car yaw (degree)'], prop=font) # using a named size
        plt.savefig('log results/psi.png')

        # plot position
        plt.figure(figsize=(12,12))
        plt.plot(self.path[:,0], self.path[:,1], color='r', linewidth=5)
        # plt.plot(self.car_state[:,0], self.car_state[:,1], color='r', linewidth=4)
        plt.title('car\'s position in time',fontsize=20)
        plt.xlabel('x (cm)',fontsize=20)
        plt.ylabel('y (cm)',fontsize=20)
        plt.grid()
        plt.legend(['reference','car\'s position'], prop=font) # using a named size
        plt.savefig('log results/position.png')

        # plot angular
        plt.figure(figsize=(12,8))
        plt.plot(t, np.rad2deg(self.angular), color='r', linewidth=4)
        plt.title('car\'s angular in time',fontsize=20)
        plt.xlabel('time (s)',fontsize=20)
        plt.ylabel('angular (degree/s)',fontsize=20)
        plt.grid()
        plt.legend(['car angular (degree/s)'], prop=font) # using a named size
        plt.savefig('log results/angular.png')

        print('all data saved on log results ...')
