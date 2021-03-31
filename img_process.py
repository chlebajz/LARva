import cv2
import numpy as np
import math

class Bannister:
    '''Class to store found bannisters'''
    def __init__(self, color, x, y, z, standing):
        self.color = color
        self.x = x
        self.y = y
        self.z = z
        self.standing = standing
    def __str__(self):      # for printing
        stand = "standing"if self.standing else "fallen"
        return "position: [{:.3}, {:.3}, {:.3}]".format(self.x, self.y, self.z) + ", color: '" + self.color + "', state: " + stand

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
        return np.matmul(np.linalg.inv(K_RGB), u_mid_homogeneous) * z

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
            b_x, b_y, b_z = real_position(mid[0], mid[1], z)
            new = Bannister(color, b_x, b_y, b_z, standing)
            bannisters.append(new)
    return bannisters