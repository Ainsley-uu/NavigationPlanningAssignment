from scipy.spatial import KDTree
import numpy as np
import copy
import random
import math
from dwa import Dwa
import vision as VS
from action import Action

class Node(object):
    def __init__(self, x, y, cost = 0.0, parent = None):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent = parent

class RRT_STAR(object):
    def __init__(self, expanDis=200, goalSmapleRate=50, maxIter=200):
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
        self.obstacle_x = [-999999]
        self.obstacle_y = [-999999]
        for robot_blue in vision.blue_robot:
            if robot_blue.visible and robot_blue.id > 0:
                self.obstacle_x.append(robot_blue.x)
                self.obstacle_y.append(robot_blue.y)
        for robot_yellow in vision.yellow_robot:
            if robot_yellow.visible:
                self.obstacle_x.append(robot_yellow.x)
                self.obstacle_y.append(robot_yellow.y)
        
        self.start = Node(start_x, start_y)
        self.goal = Node(goal_x, goal_y)
        self.obstacle = np.vstack((self.obstacle_x, self.obstacle_y)).T
        self.obstree = KDTree(self.obstacle)
        # print(self.obstacle)
        self.node_list = [self.start]

        for i in range(self.max_iter):
            rnd = self.sampling()
            dList = [(node.x - rnd[0]) ** 2 + (node.y - rnd[1]) ** 2 for node in self.node_list ]
            near_index  = dList.index(min(dList))
            nearest_node = self.node_list[near_index]

            # steer
            theta = math.atan2(rnd[1] - nearest_node.y, rnd[0] - nearest_node.x)
            new_node = self.get_new_node(theta, near_index, nearest_node)

            collision = self.check_obs(new_node.x, new_node.y, nearest_node.x, nearest_node.y)
            if collision:
                continue

            nearIndex = self.find_near_nodes(new_node)
            new_node = self.choose_parent(new_node, nearIndex)

            self.node_list.append(new_node)
            self.rewire(new_node, nearIndex)

            if self.is_near_goal(new_node):
                if not self.check_obs(new_node.x, new_node.y, self.goal.x, self.goal.y):
                    print("Found Path")
                    break

        self.path_x = []
        self.path_y = []
        index = len(self.node_list) - 1

        while self.node_list[index].parent is not None:
            self.path_x.append(self.node_list[index].x)
            self.path_y.append(self.node_list[index].y)
            index = self.node_list[index].parent
        
        self.path_x.append(self.start.x)
        self.path_y.append(self.start.y)
                    
        self.path_x.reverse()
        self.path_y.reverse()
        return self.path_x, self.path_y

    def dynamic_window_approach(self):
        for i in range(len(self.path_x)-1):
            start_x = self.path_x[i]
            start_y = self.path_y[i]
            goal = [self.path_x[i+1], self.path_y[i+1]]
            x=np.array([start_x, start_y, -1 * math.pi, 0, 0])
            u=np.array([0,0]) 
            global_tarj=np.array(x)
            for j in range(10):  
                dwa = Dwa()
                action = Action()
                u, current = dwa.dwa_Core(x, u, goal, self.obstacle) 
                x = dwa.Motion(x, u, 0.1)
                print(x)
                action.sendCommand(x[0], x[1], x[4])
                global_tarj=np.vstack((global_tarj,x))
                if math.hypot(x[0]-self.goal.x, x[1]-self.goal.y) <= self.robot_size/2:  #判断是否到达目标点
                    print('Arrived')
                    break       
         
    def rewire(self, newNode, nearIndex):
        n_node = len(self.node_list)
        for i in nearIndex:
            nearNode = self.node_list[i]

            d = math.hypot(nearNode.x - newNode.x, nearNode.y - newNode.y)
            s_cost = newNode.cost + d

            if nearNode.cost > s_cost:
                theta = math.atan2(newNode.y - nearNode.y, newNode.x - nearNode.x)
                if not self.check_obs(newNode.x, newNode.y, nearNode.x, nearNode.y):
                    nearNode.parent = n_node - 1;
                    nearNode.cost = s_cost


    def choose_parent(self, newNode, nearIndex):
        if len(nearIndex) == 0 :
            return newNode
        
        dList = []
        for i in nearIndex:
            dx = newNode.x - self.node_list[i].x
            dy = newNode.y - self.node_list[i].y
            d = math.hypot(dx, dy)
            theta = math.atan2(dy, dx)
            if not self.check_obs(self.node_list[i].x, self.node_list[i].y, newNode.x, newNode.y ):
                dList.append(self.node_list[i].cost + d)
            else:
                dList.append(float('inf'))
        
        minCost = min(dList)
        minIndex = nearIndex[dList.index(minCost)]

        if minCost == float('inf'):
            return newNode
        
        newNode.cost = minCost
        newNode.parent = minIndex

        return newNode

    def find_near_nodes(self, newNode):
        n_node = len(self.node_list)
        r = 800 * math.sqrt((math.log(n_node) / n_node))
        d_list = [(math.hypot( node.x - newNode.x, node.y - newNode.y)) for node in self.node_list]
        near_index = [d_list.index(i) for i in d_list if i < r]
        return near_index

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
        