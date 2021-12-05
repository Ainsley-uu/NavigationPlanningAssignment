from scipy.spatial import KDTree
import numpy as np
import copy
import random
import math
import time

class Node(object):
    def __init__(self, x, y, parent = None):
        self.x = x
        self.y = y
        self.parent = parent

class RRT(object):
    def __init__(self, expanDis=900, goalSmapleRate=50, maxIter=200):
        self.expand_dis = expanDis
        self.max_iter = maxIter
        self.goal_sample_rate = goalSmapleRate
        self.minx = -4500
        self.maxx = 4500
        self.miny = -3000
        self.maxy = 3000
        self.robot_size = 200
        self.avoid_dist = 200
        self.node_list = None 

    def plan(self, vision, start_x, start_y, goal_x, goal_y):
        # Obstacles
        obstacle_x = [-999999]
        obstacle_y = [-999999]
        for robot_blue in vision.blue_robot:
            if robot_blue.visible and robot_blue.id > 0:
                obstacle_x.append(robot_blue.x)
                obstacle_y.append(robot_blue.y)
        for robot_yellow in vision.yellow_robot:
            if robot_yellow.visible:
                obstacle_x.append(robot_yellow.x)
                obstacle_y.append(robot_yellow.y)
        
        self.start = Node(start_x, start_y)
        self.goal = Node(goal_x, goal_y)
        self.obstree = KDTree(np.vstack((obstacle_x, obstacle_y)).T)
        self.node_list = [self.start]

        # print(self.start.x)
        # print(self.start.y)
        for i in range(self.max_iter):
            rnd = self.sampling()
            dList = [(node.x - rnd[0]) ** 2 + (node.y - rnd[1]) ** 2 for node in self.node_list ]
            near_index  = dList.index(min(dList))
            # near_index = self.get_nearest_list_index(self.node_list, rnd)
            nearest_node = self.node_list[near_index]

            # steer
            theta = math.atan2(rnd[1] - nearest_node.y, rnd[0] - nearest_node.x)
            new_node = self.get_new_node(theta, near_index, nearest_node)

            collision = self.check_obs(new_node.x, new_node.y, nearest_node.x, nearest_node.y)
            if collision:
                continue

            self.node_list.append(new_node)
            if self.is_near_goal(new_node):
                # if not self.check_obs(new_node.x, new_node.y, self.goal.x, self.goal.y):
                print("Found Path")
                break

        path_x = []
        path_y = []
        index = len(self.node_list) - 1

        while self.node_list[index].parent is not None:
            path_x.append(self.node_list[index].x)
            path_y.append(self.node_list[index].y)
            index = self.node_list[index].parent
        
        path_x.append(self.start.x)
        path_y.append(self.start.y)
                    
        path_x.reverse()
        path_y.reverse()

        return path_x, path_y

    def is_near_goal(self, node):
        d = math.hypot(node.x - self.goal.x, node.y - self.goal.y)
        if d < self.expand_dis:
            return True
        return False

    def get_new_node(self, theta, n_ind, nearestNode):
        newNode = copy.deepcopy(nearestNode)
        newNode.x += self.expand_dis * math.cos(theta)
        newNode.y += self.expand_dis * math.sin(theta)
        newNode.parent = n_ind
        return newNode

    def sampling(self):
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = [random.uniform(self.minx, self.maxx), random.uniform(self.miny, self.maxy)]
        else:
            rnd = [self.goal.x, self.goal.y]
        return rnd

    def check_obs(self, ix, iy, nx, ny):
        x = ix
        y = iy
        dx = nx - ix
        dy = ny - iy
        angle = math.atan2(dy, dx)
        dis = math.hypot(dx, dy)

        step_size = self.robot_size + self.avoid_dist
        steps = round(dis/step_size)
        for i in range(steps):
            distance, index = self.obstree.query(np.array([x, y]))
            if distance <= self.robot_size + self.avoid_dist:
                return True
            x += step_size * math.cos(angle)
            y += step_size * math.sin(angle)

        # check for goal point
        distance, index = self.obstree.query(np.array([nx, ny]))
        if distance <= self.robot_size + self.avoid_dist:
            return True

        return False
        