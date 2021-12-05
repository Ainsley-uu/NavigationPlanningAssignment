from action import Action
from vision import Vision
from icecream import ic as print

vision = Vision()
action = Action()

while True:
    action.sendCommand(0,0,0.2)
    print(vision.my_robot.orientation)
    