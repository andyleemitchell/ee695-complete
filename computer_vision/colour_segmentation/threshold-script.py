# from https://stackoverflow.com/a/57469788

import sys

import cv2
import numpy as np


def nothing(x):
    pass


# Create a window
cv2.namedWindow("image")

# create trackbars for color change
cv2.createTrackbar("HMin", "image", 0, 179, nothing)  # Hue is from 0-179 for Opencv
cv2.createTrackbar("SMin", "image", 0, 255, nothing)
cv2.createTrackbar("VMin", "image", 0, 255, nothing)
cv2.createTrackbar("HMax", "image", 0, 179, nothing)
cv2.createTrackbar("SMax", "image", 0, 255, nothing)
cv2.createTrackbar("VMax", "image", 0, 255, nothing)

# Set default value for MAX HSV trackbars.
cv2.setTrackbarPos("HMax", "image", 179)
cv2.setTrackbarPos("SMax", "image", 255)
cv2.setTrackbarPos("VMax", "image", 255)

# Initialize to check if HSV min/max value changes
hMin = sMin = vMin = hMax = sMax = vMax = 0
phMin = psMin = pvMin = phMax = psMax = pvMax = 0

# img = cv2.imread("warped.jpg")
# img = cv2.resize(img, dsize=(0,0), fx=0.5, fy=0.5)
video_capture = cv2.VideoCapture(0)

if not video_capture.isOpened():
    msg = "Could not open the camera."
    raise RuntimeError(msg)

ret, frame = video_capture.read()
if not ret:
    print("Failed to grab frame")
frame = cv2.GaussianBlur(frame, (5, 5), 0)

img = frame
output = img
waitTime = 33

while 1:
    # get current positions of all trackbars
    hMin = cv2.getTrackbarPos("HMin", "image")
    sMin = cv2.getTrackbarPos("SMin", "image")
    vMin = cv2.getTrackbarPos("VMin", "image")

    hMax = cv2.getTrackbarPos("HMax", "image")
    sMax = cv2.getTrackbarPos("SMax", "image")
    vMax = cv2.getTrackbarPos("VMax", "image")

    # Set minimum and max HSV values to display
    lower = np.array([hMin, sMin, vMin])
    upper = np.array([hMax, sMax, vMax])

    # Create HSV Image and threshold into a range.
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)
    output = cv2.bitwise_and(img, img, mask=mask)

    # Print if there is a change in HSV value
    if (
        (phMin != hMin)
        | (psMin != sMin)
        | (pvMin != vMin)
        | (phMax != hMax)
        | (psMax != sMax)
        | (pvMax != vMax)
    ):
        print(
            "(hMin = %d , sMin = %d, vMin = %d), (hMax = %d , sMax = %d, vMax = %d)"
            % (hMin, sMin, vMin, hMax, sMax, vMax)
        )
        phMin = hMin
        psMin = sMin
        pvMin = vMin
        phMax = hMax
        psMax = sMax
        pvMax = vMax

    # Display output image
    cv2.imshow("image", output)

    # Wait longer to prevent freeze for videos.
    if cv2.waitKey(waitTime) & 0xFF == ord("q"):
        break

cv2.destroyAllWindows()
