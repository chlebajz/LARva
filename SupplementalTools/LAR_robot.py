import cv2
import math
import numpy as np
robot_position = [0, 0, 0]
K_RGB = np.array([[554.25469119, 0, 320.5],
 [  0, 554.25469119, 240.5],
 [  0, 0, 1]])

class Bannister:
    def __init__(self, color, x, y, z, standing):
        self.color = color
        self.x = x
        self.y = y
        self.z = z
        self.standing = standing
    def __str__(self):      # for printing
        stand = "standing"if self.standing else "fallen"
        return "position: [{:.3}, {:.3}, {:.3}]".format(self.x, self.y, self.z) + ", color: '" + self.color + "', state: " + stand
    def __lt__(self, other): # for comparing two nodes
        return  self.distance() > other.distance()
    def distance(self):
        return math.sqrt(pow((self.x - robot_position[0]),2) + pow((self.y - robot_position[1]),2) + pow((self.z - robot_position[2]),2))

def distance_from_rgb(x, y, w , h):
    # code from LAR laboratory
    u1_homogeneous = np.array([(y+h)/2, x, 1])
    u2_homogeneous = np.array([(y+h)/2, x+w , 1])
    x1 = np.linalg.inv(K_RGB) @ u1_homogeneous
    x2 = np.linalg.inv(K_RGB) @ u2_homogeneous
    cos_alpha = np.dot(x1, x2) / (np.linalg.norm(x1) * np.linalg.norm(x2))
    alpha = np.arccos(cos_alpha)
    return 0.025 / np.sin(alpha / 2)

def real_position(x, y, z):
    u_mid_homogeneous = np.array([y, x, 1])
    return np.linalg.inv(K_RGB) @ u_mid_homogeneous * z

#const_K = robot.get_rgb_K() #to obtain constant K for the robot
#img = cv2.imread("../Data/Robot_view_png.png") #picture 1
#img = cv2.imread("../Data/capture2.png") #picture 2
#img = cv2.imread("../Data/capture3.png") #picture 3
img = cv2.imread("../Data/capture4.png")
HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# robot.get_depth_image()
depth = np.full((640, 480), None) #for depth information use the line above

color_min = [(0, 35, 25), (49, 30, 25), (107, 30, 25)] # for R, G, B in format (h_min, s_min, v_min)
color_max = [(15, 255, 255), (77, 255, 255), (135, 255, 255)] # for R, G, B in format (h_max, s_max, v_max)

imgContour = img.copy()  # for debug
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
            cv2.drawContours(imgContour, cnt, -1, (0, 0, 255), 2)
            continue
        standing = True
        if float(h) / w > 8: #standing Bannister
            imgContour = cv2.drawContours(imgContour, cnt, -1, (255, 0, 0), 2)
            imgContour = cv2.putText(imgContour, color, (x, y - 3), cv2.FONT_HERSHEY_COMPLEX, 0.4, (0, 0, 0), 1)
        elif float(w) / h > 8: # Bannister on the floor
            cv2.drawContours(imgContour, cnt, -1, (0, 0, 255), 2)
            standing = False
        else:
            print(float(h) / w)
            print("Invalid ratio")
            continue
        mid = (int(x + w / 2), int(y + h / 2))
        z = depth[mid[0], mid[1]]
        if z is None:
            z = distance_from_rgb(x, y, w, h)
        b_x, b_y, b_z = real_position(mid[0], mid[1], z)
        new = Bannister(color, b_x, b_y, b_z, standing)
        bannisters.append(new)

#out = cv2.connectedComponentsWithStats(frame_threshold)

cv2.imshow("Image", img) #show the image on screen
#cv2.imshow("Image threshold", frame_threshold) #show the image on screen
cv2.imshow("Image contours", imgContour) #show the image on screen
for b in bannisters:
    print(b)
cv2.waitKey(0)