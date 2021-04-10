import pilot
import img_process as img
import frontier

#Class responsible for navigation
class Navigator():
    def __init__(self, robot):
        self.pilot = pilot.Pilot(robot)
        self.cones = [(1, -0.5, 'r'), (1, 0.5, 'b'), (2, 0.25, 'g'), (2, -0.75, 'g')]
        self.path = []
        self.goal = None
        self.stateDistance = 0.2
        self.allowedConeDistance = 0.2
        self.acceptedPathLenght = 5
        self.explored = []

        self.pilot.rotate2zero() #only for testing
        print("INFO: Navigator is now initialized")

    def toppleRedCone(self):
        reached = False
        while (not reached):
            self.setGoal()
            self.findPath(self.goal)
            if (len(self.path) > self.acceptedPathLenght):
                self.pilot.drive(self.path[:self.acceptedPathLenght])
                self.scanForCones()
            else:
                self.pilot.drive(self.path)
                reached = True

    def setGoal(self):
        redCone = None
        for x in range(len(self.cones)):
            if (self.cones[x][2] == 'r'):
                redCone = (self.cones[x][0], self.cones[x][1])
                break
        if (redCone is None):
            redCone = self.goal # don't forget to add coordinate transform
        self.goal = redCone

    def findPath(self, goal):
        queue = frontier.queue(goal)  # initialize the frontier
        queue.push([[(0, 0), 0]], [None, 0, None, 0])  # push the first node into the frontier
        self.explored = []

        while True:
            current = queue.pop()

            if (current is None):  # what to do if the frontier is empty
                self.path = None
                break
            elif (self.isCloseToCone(current[0], goal, False)):  # reached the end state
                self.extractPath(current)
                break

            children = self.getNextStates(current[0])  # uncover the current nodes children [[(x1, y1), cost], [(x2, y2), cost], ... ]
            self.explored.append(current[0])
            children = self.removeExpolored(children)
            queue.push(children, current)

    def getNextStates(self, state):
        new = []
        new.append([(state[0] + self.stateDistance, state[1]), self.stateDistance])
        new.append([(state[0] - self.stateDistance, state[1]),self.stateDistance])
        new.append([(state[0], state[1] + self.stateDistance),self.stateDistance])
        new.append([(state[0], state[1] - self.stateDistance),self.stateDistance])
        diagDist = (2*(self.stateDistance**2))**0.5
        new.append([(state[0] + self.stateDistance, state[1] + self.stateDistance), diagDist])
        new.append([(state[0] + self.stateDistance, state[1] - self.stateDistance), diagDist])
        new.append([(state[0] - self.stateDistance, state[1] + self.stateDistance), diagDist])
        new.append([(state[0] - self.stateDistance, state[1] - self.stateDistance), diagDist])
        self.removeBlockedStates(new)
        return new

    def removeBlockedStates(self, states):
        x = 0
        remNum = 0
        while (x < len(states)):
            rem = False
            for cone in self.cones:
                if (self.isCloseToCone(states[x][0], cone, True)):
                    print(cone)
                    states.remove(states[x])
                    remNum += 1
                    rem = True
                    break
            if not rem:
                x += 1

    def isCloseToCone(self, state, cone, excludeGoal):
        dist = (((state[0]-cone[0])**2)+((state[1]-cone[1])**2))**0.5
        if (excludeGoal):
            return dist <= self.allowedConeDistance and not (cone[0] == self.goal[0] and cone[1] == self.goal[1])
        else:
            return dist <= self.allowedConeDistance

    # remove already explored nodes from addition to the frontier
    def removeExpolored(self, children):
        correctionShift = 0
        for x in range(len(children)):
            child = children[x-correctionShift]
            if (child[0] in self.explored):
                children.remove(child)
                correctionShift += 1
        return children

    # generate path to node in the proper format
    def extractPath(self, node):
        path = []
        oldest = node[0]

        while (not (oldest[0] == 0 and oldest[1] == 0)):
            path.append(oldest)
            node = node[2]
            oldest = node[0]

        path.reverse()
        path.append(self.goal)
        self.path = path

    def scanForCones(self):
        self.cones = [(0, -0.1, 'r')]