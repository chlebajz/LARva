
# the implementation of the frontier as a priority queue
class queue():
    def __init__(self, goal):
        self.goal = goal
        self.lineup = []                                            # the queue itself in a [(x, y), value, parent, distanceTraveled] format

    # returns the first node in line or None if the line is empty
    def pop(self):
        try:
            ret = self.lineup.pop(0)
        except:
            ret = None
        return ret

    # adds new nodes to the graph
    def push(self, children, parent):
        children = self.formatChildren(children, parent)
        children = self.removeDuplicates(children)
        children= sorted(children, key=lambda x: x[1])

        if self.lineup != []:
            for node in children:
                inserted = False;
                for x in range(len(self.lineup)):
                    if (node[1] <= self.lineup[x][1]):
                        self.lineup.insert(x, node)
                        inserted = True
                        break
                if (not inserted):
                    self.lineup.append(node)
        else:
            self.lineup = children


    # calculates the values of the children and puts them inte the proper format to be stored in the queue
    def formatChildren(self, children, parent):
        newNodes = []
        for child in children:
            distanceTraveled = child[1]+parent[3]
            value = distanceTraveled + self.getHeuristics(child[0])
            newNodes.append([child[0], value, parent, distanceTraveled])

        return newNodes

    #if newNodes and the queue contain the same node, the more expensive one is removed
    def removeDuplicates(self, newNodes):
        correctionShift = 0
        for x in range(len(newNodes)):
            new = newNodes[x-correctionShift]
            for old in self.lineup:

                if (new[0] == old[0]):
                    if(new[1] >= old[1]):
                        newNodes.remove(new)
                        correctionShift += 1
                        break
                    else:
                        self.lineup.remove(old)
                        break

        return newNodes

    # returns the heuristic value for selected node - the euklidian distance from the goal
    def getHeuristics(self, node):
        x = node[0] - self.goal[0]
        y = node[1] - self.goal[1]
        h = (x ** 2 + y ** 2) ** 0.5

        return h