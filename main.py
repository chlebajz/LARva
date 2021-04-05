#The main file -> TO RUN THIS PROJECT run this file
from robolab_turtlebot import Turtlebot
import navigator

robot = Turtlebot(rgb=True, depth=True)
navigator = navigator.Navigator(robot)






