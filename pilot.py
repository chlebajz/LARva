from robolab_turtlebot import Rate, get_time
import numpy as np

#class responsible for driving the robot along a specified path
class Pilot():
    def __init__(self, robot, navigator):
        self.robot = robot
        self.navigator = navigator
        self.rate = Rate(10)
        self.start = True
        print("INFO: Pilot has been initialized")

    def drive(self):
        path = self.navigator.getPath()
        for step in path:
            angle = self.getBearing(step)
            self.setBearing(angle)
            self.driveTo(step)

    def driveTo(self, point):
        self.start = False
        v = 0.1
        prevDist = float('Inf')
        distance = self.getDistance(point)
        while (distance > 0.02 and distance < prevDist):
            self.robot.cmd_velocity(linear=v)
            prevDist = distance
            self.rate.sleep()
            distance = self.getDistance(point)
        if not (distance < 0.05):
            print("INFO: Adjustment needed to reach: ", point)
            angle = self.getBearing(point)
            self.setBearing(angle)
            self.driveTo(point)

    def getDistance(self, point):
        currentPos = self.getCurrentPos()
        dx = point[0] - currentPos[0]
        dy = point[1] - currentPos[1]
        distance = ((dx ** 2) + (dy ** 2)) ** 0.5
        return distance

    def setBearing(self, angle):
        curAngle = self.getCurrentPos()[2]
        self.start = False
        da = angle - curAngle
        v = np.sign(da)*0.1
        angleInt = angle
        while (v > 0 and curAngle < angleInt) or (v < 0 and curAngle > angleInt):
            self.robot.cmd_velocity(angular=v)
            self.rate.sleep()
            curAngle = self.getCurrentPos()[2]

    def getBearing(self, point):
        currentPos = self.getCurrentPos()
        dx = point[0] - currentPos[0]
        dy = point[1] - currentPos[1]
        if (dx != 0):
            angle = np.arctan(dy/dx)
        else:
            angle = np.pi/2
        if (dx < 0 ):
            angle += np.pi
        if (angle > np.pi):
            angle -=2*np.pi
        return angle

    #At the start, fix the skew of the robot
    def rotate2zero(self):
        if self.start:
            t = get_time()
            v = -0.14
            while ((get_time() - t) < 1):
                self.robot.cmd_velocity(angular=v)
                self.rate.sleep()
            self.robot.reset_odometry()
            print("INFO: The angle is now true 0")
        else:
            print("ERROR: Rotate2zero function can be called only on startup")

    def getCurrentPos(self):
        if self.start:
            return [0, 0, 0]
        else:
            return self.robot.get_odometry()

    def getRGBKmatrix(self):
        return self.robot.get_rgb_K()