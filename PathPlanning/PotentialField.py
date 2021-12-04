from scipy.spatial import KDTree
import numpy as np
import math

class APF(object):
    def __init__(self, katt=1, krep=10000000000, da=100, db=500, step_size=10, enddis=100):
        self.katt = katt
        self.krep = krep
        self.da = da
        self.db = db
        self.step_size = step_size
        self.enddis = enddis

    def potential_field_path_planning(self, vision, start_x, start_y, goal_x, goal_y):
        obstacle_x = []
        obstacle_y = []
        for robot_blue in vision.blue_robot:
            if robot_blue.visible and robot_blue.id > 0:
                obstacle_x.append(robot_blue.x)
                obstacle_y.append(robot_blue.y)
        for robot_yellow in vision.yellow_robot:
            if robot_yellow.visible:
                obstacle_x.append(robot_yellow.x)
                obstacle_y.append(robot_yellow.y)
        current_x = start_x
        current_y = start_y
        # print('当前位置:')
        # print([current_x, current_y])
        road_points_x = [current_x]
        road_points_y = [current_y]
        while True:
            force_x, force_y = self.calculate_force(current_x, current_y, goal_x,
                                                    goal_y, obstacle_x, obstacle_y)
            # print('得到的force:')
            # print([force_x, force_y])
            current_x = current_x + force_x * self.step_size
            current_y = current_y + force_y * self.step_size
            # print('当前位置:')
            # print([current_x,current_y])
            if np.hypot(current_x - goal_x, current_y - goal_y) < self.enddis:
                road_points_x.append(current_x)
                road_points_y.append(current_y)
                road_points_x.append(goal_x)
                road_points_y.append(goal_y)
                break
            road_points_x.append(current_x)
            road_points_y.append(current_y)
        return road_points_x, road_points_y

    def calculate_force(self, current_x, current_y, goal_x, goal_y, obstacle_x, obstacle_y):
        obn = len(obstacle_x)
        force_x = 0
        force_y = 0
        att_force_x, att_force_y = self.calculate_attractive_force(current_x, current_y, goal_x, goal_y)
        # print('att_force为:')
        # print([att_force_x,att_force_y])
        rep_force_x, rep_force_y = self.calculate_repulsive_force(current_x, current_y, obstacle_x, obstacle_y, obn)
        # print('rep_force为:')
        # print([rep_force_x,rep_force_y])
        force_x = force_x + att_force_x + rep_force_x
        force_y = force_y + att_force_y + rep_force_y
        # print('归一化前force:')
        # print([force_x,force_y])
        dis = np.hypot(force_x, force_y)
        force_x = force_x / dis
        force_y = force_y / dis
        # print('归一化后force:')
        # print([force_x, force_y])
        return force_x, force_y

    def calculate_attractive_force(self, current_x, current_y, goal_x, goal_y):
        delta_x = (goal_x - current_x)
        # print(delta_x)
        delta_y = (goal_y - current_y)
        # print(delta_y)
        dis = np.hypot(delta_x, delta_y)
        # print(dis)
        if dis < self.da:
            att_force_x = self.katt * delta_x
            att_force_y = self.katt * delta_y
        else:
            att_force_x = self.katt * self.da * delta_x / dis
            att_force_y = self.katt * self.da * delta_y / dis
        # print(att_force_x)
        # print(att_force_y)
        return att_force_x, att_force_y

    def calculate_repulsive_force(self, current_x, current_y, obstacle_x, obstacle_y, obn):
        obstree = KDTree(np.vstack((obstacle_x, obstacle_y)).T)
        dis, index = obstree.query(np.array([current_x, current_y]), k=obn)
        # print(dis)
        # print(index)
        rep_force_x = 0
        rep_force_y = 0
        for i in range(len(index)):
            if dis[i] < self.db:
                delta_x = current_x - obstacle_x[index[i]]
                delta_y = current_y - obstacle_y[index[i]]
                rep_force_xi = self.krep * (1 / dis[i] - 1 / self.db) * math.pow((1 / dis[i]), 3) * delta_x
                rep_force_yi = self.krep * (1 / dis[i] - 1 / self.db) * math.pow((1 / dis[i]), 3) * delta_y
                rep_force_x += rep_force_xi
                rep_force_y += rep_force_yi
            else:
                break
        return rep_force_x, rep_force_y
