from vision import Vision
from action import Action
from debug import Debugger
from rrt import RRT
import time

if __name__ == '__main__':
    vision = Vision()
    action = Action()
    debugger = Debugger()
    planner = RRT()
    while True:
        # 1. path planning & velocity planning
        start_x, start_y = vision.my_robot.x, vision.my_robot.y
        goal_x, goal_y = -2400, -1500
        path_x, path_y = planner.plan(vision=vision, 
            start_x=start_x, start_y=start_y, goal_x=goal_x, goal_y=goal_y)

        # 2. send command
        action.sendCommand(vx=0, vy=0, vw=0)

        # 3. draw debug msg
        debugger.draw_all(path_x, path_y, vision.my_robot.x, vision.my_robot.y)
        
        time.sleep(0.01)
