import cv2
import numpy as np

def empty(value):
    # print(value)
    pass


# create window for trackbars
cv2.namedWindow("TrackBars")  # create window to be able to adjust values in real time
cv2.resizeWindow("TrackBars", 640, 240)  # resize the window

# create a trackbars, in opencv max hue is 179, the trackbar calls function empty on change
cv2.createTrackbar("Hue Min", "TrackBars", 0, 179, empty)
cv2.createTrackbar("Hue Max", "TrackBars", 179, 179, empty)
cv2.createTrackbar("Sat Min", "TrackBars", 0, 255, empty)
cv2.createTrackbar("Sat Max", "TrackBars", 255, 255, empty)
cv2.createTrackbar("Val Min", "TrackBars", 0, 255, empty)
cv2.createTrackbar("Val Max", "TrackBars", 255, 255, empty)

img = cv2.imread("Resources/Robot_view_png.png")
imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # convert colors from RGB to HSV
while True:
    h_min = cv2.getTrackbarPos("Hue Min", "TrackBars")
    h_max = cv2.getTrackbarPos("Hue Max", "TrackBars")
    s_min = cv2.getTrackbarPos("Sat Min", "TrackBars")
    s_max = cv2.getTrackbarPos("Sat Max", "TrackBars")
    v_min = cv2.getTrackbarPos("Val Min", "TrackBars")
    v_max = cv2.getTrackbarPos("Val Max", "TrackBars")

    lower = np.array([h_min, s_min, v_min])
    upper = np.array([h_max, s_max, v_max])
    mask = cv2.inRange(imgHSV, lower, upper) #create a mask

    imgResult = cv2.bitwise_and(img, img, mask=mask) #apply mask

    #cv2.imshow("Original", img)
    #cv2.imshow("HSV", imgHSV)
    cv2.imshow("Mask", mask)
    cv2.imshow("Result", imgResult)
    cv2.waitKey(50)