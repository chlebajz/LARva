import cv2
import numpy as np
import math

class Bannister:
    '''Class to store found bannisters'''
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

class Coordinates:
    '''Class to store coordinates'''
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
    def __repr__(self):  # for printing
        # Python 3
        # return "[{:.3}, {:.3}, {:.3}]".format(self.x, self.y, self.z)
        # Python 2
        return "[%.3f, %.3f, %.3f]" % (self.x, self.y, self.z)
    def numpyfy(self):
        return np.array([self.x, self.y, self.z])

def distance(robot_pos, Bannister): # function to get the distance from the bannister
    return math.sqrt(pow((Bannister.x - robot_pos[0]), 2) + pow((Bannister.y - robot_pos[1]), 2) + pow((Bannister.z - robot_pos[2]), 2))

def print_bannisters(bannisters, robot_pos): # for debugging purposes
    for i in range(len(bannisters)):
        print("Bannister {}:".format(i), bannisters[i], "distance:", distance(robot_pos, bannisters[i]))

def process(img, depth_img, K_RGB):
    def distance_from_rgb(x, y, w, h, standing):
        # code from LAR laboratory
        if standing:
            u1_homogeneous = np.array([x, (y + h) / 2, 1])
            u2_homogeneous = np.array([x + w, (y + h) / 2, 1])
        else:
            u1_homogeneous = np.array([(x + w) / 2, y, 1])
            u2_homogeneous = np.array([(x + w) / 2, (y + h), 1])
        x1 = np.matmul(np.linalg.inv(K_RGB), u1_homogeneous)
        x2 = np.matmul(np.linalg.inv(K_RGB), u2_homogeneous)
        cos_alpha = np.dot(x1, x2) / (np.linalg.norm(x1) * np.linalg.norm(x2))
        alpha = np.arccos(cos_alpha)
        return 0.025 / np.sin(alpha / 2)

    def real_position(x, y, z):
        u_mid_homogeneous = np.array([x, y, 1])
        new_x, new_y, new_z = np.matmul(np.linalg.inv(K_RGB), u_mid_homogeneous * z)
        return Coordinates(new_z, new_x, new_y) # swap the axes because the point is in camera coordinates

    HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    color_min = [(0, 35, 25), (49, 30, 25), (107, 30, 25)] # for R, G, B in format (h_min, s_min, v_min)
    color_max = [(15, 255, 255), (77, 255, 255), (135, 255, 255)] # for R, G, B in format (h_max, s_max, v_max)
    bannisters = []

    for i in range(3):
        color = "Red"
        if i == 1: color = "Green"
        elif i == 2: color = "Blue"
        frame_threshold = cv2.inRange(HSV, color_min[i], color_max[i])
        contours, hierarchy = cv2.findContours(frame_threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 500:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)
            if len(approx) > 4: # not a square
                continue
            if float(h) / w > 8: #standing Bannister
                standing = True
            elif float(w) / h > 8: # Bannister on the floor
                standing = False
            else: #invalid ratio
                continue
            mid = (int(x + w / 2), int(y + h / 2))
            z = depth_img[mid[1], mid[0]]
            if z is None:
                z = distance_from_rgb(x, y, w, h, standing)
            else:
                z += 0.025 # convert to distance from the middle of the bannsiter
            pos = real_position(mid[0], mid[1], z) # position in camera coordinates
            new = Bannister(color, pos, standing)
            bannisters.append(new)
    return bannisters

def convert_to_real_coord(robot_x, robot_y, robot_theta, coordinates):
    To = np.array([[math.cos(robot_theta), (-1)*math.sin(robot_theta), robot_x],
                  [math.sin(robot_theta), math.cos(robot_theta), robot_y],
                  [0, 0, 1]])
    return np.matmul(To, np.transpose(coordinates.numpyfy()))

if __name__ == "__main__": # For testing purposes
    img = cv2.imread("Resources/Robot_view_png.png")
    depth = np.full((480, 640), None)
    K_RGB = np.array([[554.25469119, 0, 320.5],
                      [0, 554.25469119, 240.5],
                      [0, 0, 1]])
    bannisters = process(img, depth, K_RGB)
    # c = Coordinates(0, 1, 0)
    # b = Bannister("Green", c, True)
    # bannisters = [b]
    print(bannisters)
    print(convert_to_real_coord(0, 0, 0, bannisters[0].coord))
    print(convert_to_real_coord(0, 0, 0, bannisters[1].coord))
    print(convert_to_real_coord(0, 0, 0, bannisters[2].coord))
    cv2.imshow("Image", img)
    cv2.waitKey(0)