import numpy as np
import math
from debug import Debugger
from icecream import ic as print
 
class Dwa(object):
#参数设置
    def __init__(self):
        self.V_Min = -3000.0           #最小速度
        self.V_Max = 3000.0             #最大速度
        self.W_Min = -5              #最小角速度
        self.W_Max = 5               #最大角速度
        self.Va = 3000                #加速度
        self.Wa = 5      #角加速度
        self.Vreso = 400           #速度分辨率
        self.Wreso = 0.2    #角速度分辨率
        self.radius = 100 
        self.Dt = 0.2             #时间间隔
        self.Predict_Time = 2     #模拟轨迹的持续时间
        self.alpha = 0.9            #距离目标点的评价函数的权重系数
        self.Belta = 0.1            #速度评价函数的权重系数
        self.Gamma = 100000            #距离障碍物距离的评价函数的权重系数
 
    #距离目标点的评价函数
    def Goal_Cost(Goal,Pos):
        return math.hypot(Pos[-1,0]-Goal[0], Pos[-1,1]-Goal[1])
    
    #速度评价函数
    def Velocity_Cost(self, Pos):
        return self.V_Max-Pos[-1,3]
    
    #距离障碍物距离的评价函数
    def Obstacle_Cost(self, Pos,Obstacle):
        MinDistance = float('Inf')          #初始化时候机器人周围无障碍物所以最小距离设为无穷
        for i in range(len(Pos)):           #对每一个位置点循环
            for j in range(len(Obstacle)):  #对每一个障碍物循环
                Current_Distance = math.hypot(Pos[i,0]-Obstacle[j,0], Pos[i,1]-Obstacle[j,1])
                if Current_Distance < self.radius:            #如果小于机器人自身的半径那肯定撞到障碍物了返回的评价值自然为无穷
                    return float('Inf')
                if Current_Distance < MinDistance:
                    MinDistance=Current_Distance         #得到点和障碍物距离的最小
        return 1/MinDistance
    
    #速度采用
    def V_Range(self, X):
        Vmin_Actual =  X[3]-self.Va*self.Dt          #实际在dt时间间隔内的最小速度
        Vmax_actual =  X[3]+self.Va*self.Dt          #实际载dt时间间隔内的最大速度
        Wmin_Actual = X[4]-self.Wa*self.Dt          #实际在dt时间间隔内的最小角速度
        Wmax_Actual = X[4]+self.Wa*self.Dt          #实际在dt时间间隔内的最大角速度
        VW = [max(self.V_Min,Vmin_Actual),min(self.V_Max,Vmax_actual),max(self.W_Min,Wmin_Actual),min(self.W_Max,Wmax_Actual)]  #因为前面本身定义了机器人最小最大速度所以这里取交集
        return VW
    
    #一条模拟轨迹路线中的位置，速度计算
    def Motion(self, X,u,dt):
        X[0] += u[0] * dt * math.cos(X[2])           #x方向上位置
        X[1] += u[0] * dt * math.sin(X[2])           #y方向上位置
        X[2] += u[1] * dt                     #角度变换
        X[3] = u[0]                         #速度
        X[4] = u[1]                         #角速度
        return X
    
    #一条模拟轨迹的完整计算
    def Calculate_Traj(self, X,u):
        debugger = Debugger()
        Traj=np.array(X)
        Xnew=np.array(X)
        time=0
        while time <= self.Predict_Time:
            # print(Xnew[2])
            Xnew[0] += u[0] * self.Dt * math.cos(Xnew[2])           #x方向上位置
            Xnew[1] += u[0] * self.Dt * math.sin(Xnew[2])           #y方向上位置
            Xnew[2] += u[1] * self.Dt                     #角度变换
            Xnew[3] = u[0]                         #速度
            Xnew[4] = u[1]        #一条模拟轨迹时间
            # Xnew=self.Motion(Xnew,u,self.Dt)
            Traj=np.vstack((Traj,Xnew))   #一条完整模拟轨迹中所有信息集合成一个矩阵
            time=time+self.Dt
        # debugger.draw_dwa(Traj[:,0], Traj[:,1])
        return Traj
    
    #DWA核心计算
    def dwa_Core(self, X,u,goal,obstacles):
        vw=self.V_Range(X)
        best_traj=np.array(X)
        min_score=10000.0                 #随便设置一下初始的最小评价分数
        for v in np.arange(vw[0], vw[1], self.Vreso): 
            # print(v) 
            for w in np.arange(vw[2], vw[3], self.Wreso):     #对每一个角速度循环
                traj=self.Calculate_Traj(X,[v,w])
                goal_score = math.hypot(traj[-1,0]-goal[0], traj[-1,1]-goal[1])
                # print("goal_score")
                # print(goal_score)
                vel_score=self.Velocity_Cost(traj)
                # print("vel_score")
                # print(vel_score)
                obs_score=self.Obstacle_Cost(traj,obstacles)
                # print("obs_score")
                # print(obs_score)
                goal_score *= self.alpha
                vel_score *= self.Belta
                obs_score *= self.Gamma
                # print(goal_score)
                # print(vel_score)
                # print(obs_score)
                score=goal_score + vel_score + obs_score
                if min_score>=score:                    #得出最优评分和轨迹
                    min_score=score
                    u=np.array([v,w])
                    best_traj=traj
        return u,best_traj
