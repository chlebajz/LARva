
#Class responsible for navigation
class Navigator():
    def __init__(self):
        self.cones = []
        #self.path = [(1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1)]
        self.path = [(0.5, 0), (1, 0), (1.5, -0.25), (1.5, -0.5), (1.5, -1), (1.25, -1.5)]
        print("INFO: Navigator has now been initialized")

    def getPath(self):
        return self.path