import math
from random import sample
import numpy as np
from scipy.spatial import cKDTree, Voronoi

from vision import Vision

class Node(object):
    def __init__(self, x, y, cost, parent):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent = parent

class VoronoiMap:
    '''
    智能移动技术 利用Voronoi图法进行路径规划
    '''
    def __init__(self, KNN=10, MAX_EDGE_LEN=5000):
        '''
        初始化
        '''
        self.KNN = KNN
        self.MAX_EDGE_LEN = MAX_EDGE_LEN
        self.minx = -4500
        self.maxx = 4500
        self.miny = -3000
        self.maxy = 3000
        self.robot_size = 200

    def plan(self, vision, start_x, start_y, goal_x, goal_y):
        '''
        进行规划
        '''
        # 障碍物
        obstacle_x = []
        obstacle_y = []
        while len(obstacle_x) == 0:
            for robot_blue in vision.blue_robot:
                if robot_blue.visible and robot_blue.id > 0:
                    obstacle_x.append(robot_blue.x)
                    obstacle_y.append(robot_blue.y)
            for robot_yellow in vision.yellow_robot:
                if robot_yellow.visible:
                    obstacle_x.append(robot_yellow.x)
                    obstacle_y.append(robot_yellow.y)
        obstree = cKDTree(np.vstack((obstacle_x, obstacle_y)).T)
        sample_x, sample_y = self.sampling(start_x, start_y, goal_x, goal_y, obstacle_x, obstacle_y)
        road_map = self.generate_roadmap(sample_x, sample_y, obstree)
        path_x, path_y = self.dijkstra_search(start_x, start_y, goal_x, goal_y, road_map, sample_x, sample_y)
        return path_x, path_y, road_map, sample_x, sample_y

    @staticmethod
    def sampling(start_x, start_y, goal_x, goal_y, obstacle_x, obstacle_y):
        oxy = np.vstack((obstacle_x, obstacle_y)).T
        vor = Voronoi(oxy)
        
        sample_x = [ix for [ix, _] in vor.vertices]
        sample_y = [iy for [_, iy] in vor.vertices]
        
        sample_x.append(start_x)
        sample_y.append(start_y)
        sample_x.append(goal_x)
        sample_y.append(goal_y)
        
        return sample_x, sample_y

    def is_collision(self, start_x, start_y, goal_x, goal_y, obstree):
        x = start_x
        y = start_y
        dx = goal_x - start_x
        dy = goal_y - start_y
        yaw = math.atan2(dx, dy)
        d = math.hypot(dx, dy)
        
        if d > self.MAX_EDGE_LEN:
            return True
        
        n_step = round(d / self.robot_size)
        for _ in range(n_step):
            dist, _ = obstree.query([x, y])
            if dist <= self.robot_size:
                return True
            x += self.robot_size * math.cos(yaw)
            y += self.robot_size * math.sin(yaw)
            
        # goal point check
        dist, _ = obstree.query([goal_x, goal_y])
        if dist <= self.robot_size:
            return True
    
        return False

    def generate_roadmap(self, sample_x, sample_y, obstree):
        road_map = []
        n_sample = len(sample_x)
        node_tree = cKDTree(np.vstack((sample_x, sample_y)).T)
        
        for (i, ix, iy) in zip(range(n_sample), sample_x, sample_y):
            dists, indexes = node_tree.query([ix, iy], k=n_sample)
            edge_id = []
            
            for ii in range(1, len(indexes)):
                nx = sample_x[indexes[ii]]
                ny = sample_y[indexes[ii]]

                if not self.is_collision(ix, iy, nx, ny, obstree):
                    edge_id.append(indexes[ii])

                if len(edge_id) >= self.KNN:
                    break

            road_map.append(edge_id)
        return road_map

    def dijkstra_search(self, start_x, start_y, goal_x, goal_y, road_map, sample_x, sample_y):
        path_x, path_y = [], []
        start = Node(start_x, start_y, 0.0, -1)
        goal = Node(goal_x, goal_y, 0.0, -1)

        openset, closeset = dict(), dict()
        openset[len(road_map)-2] = start

        path_found = True
        while True:
            if not openset:
                print("Cannot find path")
                path_found = False
                break

            c_id = min(openset, key=lambda o: openset[o].cost)
            current = openset[c_id]

            if c_id == (len(road_map) - 1):
                print("Goal is found!")
                goal.cost = current.cost
                goal.parent = current.parent
                break

            del openset[c_id]
            closeset[c_id] = current

            # expand
            for i in range(len(road_map[c_id])):
                n_id = road_map[c_id][i]
                # print(type(sample_x[n_id]))
                # print(type(current))
                dx = sample_x[n_id] - current.x
                dy = sample_y[n_id] - current.y
                d = math.hypot(dx, dy)
                node = Node(sample_x[n_id], sample_y[n_id],
                    current.cost + d, c_id)
                if n_id in closeset:
                    continue
                if n_id in openset:
                    if openset[n_id].cost > node.cost:
                        openset[n_id].cost = node.cost
                        openset[n_id].parent = c_id
                else:
                    openset[n_id] = node

        if path_found:
            path_x.append(goal.x)
            path_y.append(goal.y)
            parent = goal.parent
            while parent != -1:
                path_x.append(closeset[parent].x)
                path_y.append(closeset[parent].y)
                parent = closeset[parent].parent

        return path_x, path_y