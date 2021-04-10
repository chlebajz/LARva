
class Cone:
    '''Class to store found cones'''
    def __init__(self, color, coord, standing):
        self.color = color
        self.coord = coord
        self.standing = standing
    def __repr__(self):      # for printing
        stand = "standing" if self.standing else "fallen"
        # Python 3
        # return "position: [{:.3}, {:.3}, {:.3}]".format(self.x, self.y, self.z) + ", color: '" + self.color + "', state: " + stand
        # Python 2
        return "\n{position: " + str(self.coord) + ", color: '" + self.color + "', state: " + stand + "}"