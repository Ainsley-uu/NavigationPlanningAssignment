from action import Action
import math
import time
import numpy as np

Kp_rho = 1800
Kp_alpha = 10
Kp_beta = -2
dt = 0.01
          
def move_to_pose(vision, path_x, path_y):
    action = Action()
    sth_lst = [math.pi]
    gth_lst = []
    
    for i in range(1, len(path_x)):
        x_goal, y_goal = path_x[i] / 1000, path_y[i] / 1000
        x_start, y_start = path_x[i-1] / 1000, path_y[i-1] / 1000
        # print(f'sx: {x_start} sy: {y_start} gx: {x_goal} gy: {y_goal}')
        gth = math.atan2((y_goal - y_start), (x_goal - x_start))
        gth_lst.append(gth)
        x, y = x_start, y_start
        theta = sth_lst[-1]

        x_diff = x_goal - x
        y_diff = y_goal - y

        x_traj, y_traj = [], []

        rho = np.hypot(x_diff, y_diff)
        while rho > 0.1:
            x_traj.append(x)
            y_traj.append(y)

            x_diff = x_goal - x
            y_diff = y_goal - y

            rho = np.hypot(x_diff, y_diff)
            alpha = (np.arctan2(y_diff, x_diff)
                 - theta + np.pi) % (2 * np.pi) - np.pi
            beta = (gth_lst[-1] - theta - alpha + np.pi) % (2 * np.pi) - np.pi

            v = Kp_rho * rho
            w = Kp_alpha * alpha + Kp_beta * beta

            if alpha > np.pi / 2 or alpha < -np.pi / 2:
                v = -v
        
            # print(f'v:{v} w:{w}')
            action.sendCommand(vx = v, vw = w)
            time.sleep(dt)

            theta = vision.my_robot.orientation
            x = vision.my_robot.x / 1000
            y = vision.my_robot.y / 1000
            sth_lst.append(gth)
        