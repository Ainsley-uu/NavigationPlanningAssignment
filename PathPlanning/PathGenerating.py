import math
from sys import path
from PathPlanning.cubic_spline_planner import calc_spline_course
from vision import Vision
from action import Action
from debug import Debugger

from PathPlanning.Voronoi import VoronoiMap
from PathPlanning.PotentialField import APF
import PathPlanning.cubic_spline_planner as cubic_spline_planner

def path_generating(vision, start_x, start_y, goal_x, goal_y):
    planner1 = VoronoiMap()
    planner2 = APF()
    
    path_x, path_y, road_map, sample_x, sample_y = planner1.plan(vision, start_x, start_y, goal_x, goal_y)
    path_x_final, path_y_final = [], []
        
    for i in range(len(path_x)-1, 0, -1):
        x_start, y_start = path_x[i], path_y[i]
        x_goal, y_goal = path_x[i-1], path_y[i-1]
        # print(f'sx: {x_start} sy: {y_start} gx: {x_goal} gy: {y_goal}')
        road_points_x, road_points_y = planner2.potential_field_path_planning(vision, x_start, y_start, x_goal, y_goal)
        # print(road_points_x)
        path_x_final += road_points_x
        path_y_final += road_points_y
    
    path_theta = []
    for i in range(1, len(path_x_final)):
        dy = path_y_final[i] - path_y_final[i-1]
        dx = path_x_final[i] - path_x_final[i-1]
        theta0 = math.atan2(dy, dx) + 2*math.pi
        path_theta.append(theta0)
        
    return path_x_final, path_y_final, path_theta