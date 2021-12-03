from vision import Vision
from action import Action
from debug import Debugger
from zss_debug_pb2 import Debug_Msgs
import time

# 实例化Vision这个类
vision_module = Vision()

# 读取我的机器人的位置，默认我的机器人为蓝车0号
while True:
    my_robot = vision_module.my_robot
    print('My robot:', my_robot.x, my_robot.y)
    # 读取蓝车0号的位置，其他车号0-15类似
    # blue_robot_0 = vision_module.blue_robot[0]
    # print('Blue robot 0:', blue_robot_0.x, blue_robot_0.y)
    # # 读取黄车0号的位置，其他车号0-15类似
    # yellow_robot_0 = vision_module.yellow_robot[0]
    # print('Yellow robot 0:', yellow_robot_0.x, yellow_robot_0.y)