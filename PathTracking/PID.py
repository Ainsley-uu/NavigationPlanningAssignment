import numpy as np
import math
import time
from vision import Vision
from action import Action
from debug import Debugger
import matplotlib.pyplot as plt

class PID:
    def __init__(self, path_x, path_y, path_theta, kp, ki, kd, dt):
        self.dt = dt
        self.set_path(path_x, path_y, path_theta)
        self.set_params(kp, ki, kd)
        self.reset()
        
    def set_path(self, path_x, path_y, path_theta):
        self.path_x = path_x
        self.path_y = path_y
        self.path_theta = path_theta
        
    def set_params(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
    def reset(self):
        last_x = self.path_x.pop(0)
        last_y = self.path_y.pop(0)
        
        self.path_x = np.asarray(self.path_x, dtype = float) / 1000
        self.path_y = np.asarray(self.path_y, dtype = float) / 1000
        self.path_theta = np.asarray(self.path_theta, dtype = float)
        
        self.v = [0]
        self.w = [0]
        
        self.x = [last_x]
        self.y = [last_y]
        
        self.xe = [0]
        self.ye = [0]
        self.te = [0]
        
    def control(self, vision):
        action = Action()
        for i in range(1, len(self.path_x)):
            x, y = vision.my_robot.x / 1000, vision.my_robot.y / 1000
            theta = vision.my_robot.orientation % (2*math.pi)
                
            q = np.array([x, y, theta], dtype=float)
            qr = np.array([self.path_x[i], self.path_y[i], self.path_theta[i]], dtype = float)
            T = np.array([[np.cos(theta), np.sin(theta), 0],
                            [-np.sin(theta), np.cos(theta), 0],
                            [0, 0, 1]])
            ep = T @ (qr - q).T
                
            print(f'qr: {qr}\nq: {q}\nqr - q: {qr - q}\nep: {ep}')
            print('-------------------------')
            self.xe.append(ep[0])
            self.ye.append(ep[1])
            self.te.append(ep[2])
            
            
            v = self.kp[0] * self.xe[-1] + \
                self.ki[0] * self.dt * sum(self.xe) + \
                self.kd[0] * (self.xe[i] - self.xe[i-1]) / self.dt
                
            w1 = self.kp[1] * self.ye[-1] + \
                self.ki[1] * self.dt * sum(self.ye) + \
                self.kd[1] * (self.ye[i] - self.ye[i-1]) / self.dt
            
            w2 = self.kp[2] * self.te[-1] + \
                self.ki[2] * self.dt * sum(self.te) + \
                self.kd[2] * (self.te[i] - self.te[i-1]) / self.dt
                
            w = w1 + w2
             
                    
            # v = self.v[-1] + self.kp[0] * (self.xe[i] - self.xe[i-1]) + self.ki[0] * self.xe[i]
            # w = self.w[-1] + self.kp[1] * (self.ye[i] - self.ye[i-1]) + self.ki[1] * self.ye[i] + \
            #         self.kp[2] * (self.te[i] - self.te[i-1]) + self.ki[2] * self.te[i]
                    
            action.sendCommand(vx = v * 1000, vw = w)
            print(f'v: {v}, w: {w}')
                
            self.v.append(v)
            self.w.append(w)
            self.x.append(x)
            self.y.append(y)
                
            time.sleep(self.dt)
            
        # plt.plot(self.v)
        # plt.show()
        # plt.plot(self.w)
        # plt.show()