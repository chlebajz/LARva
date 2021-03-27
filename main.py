#The main file -> TO RUN THIS PROJECT run this file
from robolab_turtlebot import Turtlebot
import pilot
import navigator
import img_process as img

robot = Turtlebot(rgb=True, depth=True)
navigator = navigator.Navigator()
pilot = pilot.Pilot(robot, navigator)

pilot.rotate2zero()
#pilot.drive()

rgbK = pilot.getRGBKmatrix()
image = robot.get_rgb_image()
depth = robot.get_depth_image()

K = img.process(image, depth, rgbK)

img.print_bannisters(K, pilot.getCurrentPos())




