#The main file -> TO RUN THIS PROJECT run this file
from robolab_turtlebot import Turtlebot
import pilot
import navigator

robot = Turtlebot(rgb=True, depth=True)
navigator = navigator.Navigator()
pilot = pilot.Pilot(robot, navigator)

pilot.rotate2zero()
pilot.drive()





