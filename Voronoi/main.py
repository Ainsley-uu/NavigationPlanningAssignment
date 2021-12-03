from vision import Vision
from action import Action
from debug import Debugger
from zss_debug_pb2 import Debug_Msgs
import time
from Voronoi import VoronoiMap
from scipy.spatial import Voronoi, voronoi_plot_2d
import numpy as np
import matplotlib.pyplot as plt

if __name__ == '__main__':
	vision = Vision()
	action = Action()
	debugger = Debugger()
	planner = VoronoiMap()
	i = 0
	while True:
		start_x, start_y = vision.my_robot.x, vision.my_robot.y
		goal_x, goal_y = -2400, -1500

		path_x, path_y, road_map, sample_x, sample_y = planner.plan(vision=vision, start_x=start_x, start_y=start_y, goal_x=goal_x, goal_y=goal_y)
		action.sendCommand(vx = 0, vy = 0, vw = 0)
		package = Debug_Msgs()
		# if len(path_x) > 2:
		debugger.draw_all(sample_x, sample_y, path_x, path_y)
		time.sleep(0.01)
  		# action.sendCommand(vx=0, vy=0, vw=0)
		# debugger.draw_all(sample_x, sample_y, road_map, path_x, path_y)
  		# time.sleep(0.01)