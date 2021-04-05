import pilot
import img_process as img

#Class responsible for navigation
class Navigator():
    def __init__(self, robot):
        self.pilot = pilot.Pilot(robot)
        self.cones = [(1, -0.5, 'r')]
        #self.path = [(1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1)]
        self.path = [(1, 0), (1.5, 0.5), (1.5, 1), (1.5, 1.2), (1.5, 1)]

        self.pilot.rotate2zero()
        print("INFO: Navigator has now been initialized")