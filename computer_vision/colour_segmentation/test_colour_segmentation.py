import json
import sys

import cv2
import numpy as np

from connect4_cv.quad_selector import QuadSelector

video_capture = cv2.VideoCapture(0)

if not video_capture.isOpened():
    msg = "Could not open the camera."
    raise RuntimeError(msg)

ret, frame = video_capture.read()
if not ret:
    print("Failed to grab frame")


blur_frame = cv2.GaussianBlur(frame, (5, 5), 0)  # Adjust kernel size (5, 5) as needed
# test_board = cv2.imread("./test-images/c4board.jpg")

test_board = blur_frame
# test_board = cv2.resize(test_board, dsize=(0,0), fx=0.5, fy=0.5)

QS = QuadSelector()
test = QS.select_points(frame=test_board)

if test is None:
    sys.exit()
# print(test)

# test = [(148, 80), (155, 391), (534, 431), (540, 63)]

# transform_mat = cv2.getPerspectiveTransform()


def order_points(pts):
    """
    Order points [topleft, topright, botright, botleft]

    Args:
        pts (list):  of (x,y) points

    Returns:
        np.int32 array: ordered points

    """
    rect = np.zeros((4, 2), dtype=np.float32)

    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]

    diff = np.diff(pts, axis=1)
    rect[1] = pts[np.argmin(diff)]
    rect[3] = pts[np.argmax(diff)]

    return rect

points = order_points(np.array(test, np.int32))

dst = np.array([
    [0,0],
    [700, 0],
    [700, 600],
    [0, 600]
], dtype=np.float32)

w = cv2.getPerspectiveTransform(points, dst)
warped = cv2.warpPerspective(test_board, w, (700, 600))

# for i in range(7):
#     cv2.line(warped, (i*100, 0), (i*100, 600), (0, 0, 0), 2)
# for i in range(6):
#     cv2.line(warped, (0, i*100), (700, i*100), (0, 0, 0), 2)

cv2.imshow("original", test_board)
cv2.imshow("warped", warped)

warped_hsv = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)

# cv2.imwrite("warped.jpg", warped)

# colour masking
# bounds
yellow = [
    [10, 90, 0],
    [50, 255, 255]
]
red = [
    [150, 170, 170],
    [179, 255, 255]
]

RED_COLOUR_MIN = np.array(red[0],np.uint8)
RED_COLOUR_MAX = np.array(red[1],np.uint8)
YELLOW_COLOUR_MIN = np.array(yellow[0],np.uint8)
YELLOW_COLOUR_MAX = np.array(yellow[1],np.uint8)

# Create a mask using the defined color range
red_mask = cv2.inRange(warped_hsv, RED_COLOUR_MIN, RED_COLOUR_MAX)
yellow_mask = cv2.inRange(warped_hsv, YELLOW_COLOUR_MIN, YELLOW_COLOUR_MAX)

cv2.imshow("red mask", red_mask)
cv2.imshow("yellow mask", yellow_mask)
cv2.waitKey(0)
# sys.exit()

NUM_ROWS = 6
NUM_COLUMNS = 7

cell_x = [100*i+50 for i in range(NUM_COLUMNS)]
cell_y = [100*i+50 for i in range(NUM_ROWS)]

board_state = []

for x in cell_x:
    column = []
    for y in cell_y:
        cell_mask = np.zeros(warped.shape[:2], dtype=np.uint8)
        cv2.circle(cell_mask, (x,y), 40, 255, -1)
        red_masked = cv2.bitwise_and(cell_mask, cell_mask, mask=red_mask)
        cv2.namedWindow("cell mask", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("cell mask", 500, 500)
        cv2.imshow("cell mask", cell_mask)
        cv2.imshow("cell mask", red_masked)
        cv2.moveWindow("cell mask", 2000, 1000)
        cv2.waitKey(100)
        cell_mask = np.zeros(warped.shape[:2], dtype=np.uint8)
        cv2.circle(cell_mask, (x,y), 40, 255, -1)
        yellow_masked = cv2.bitwise_and(cell_mask, cell_mask, mask=yellow_mask)
        if red_masked.any():
            column.append("red")
        elif yellow_masked.any():
            column.append("yellow")
        else:
            column.append("empty")
        print(f"red: {cv2.countNonZero(red_masked)} \t yellow: {cv2.countNonZero(yellow_masked)}")
    board_state.append(column)

# print(board_state)

# display state in new image
def display_state(board_state):
    base_image = np.zeros((600, 700, 3), np.uint8)
    for x, column in enumerate(board_state):
        for y, cell in enumerate(column):
            cell_x = 100*x+50
            cell_y = 100*y+50
            if cell == "red":
                cv2.circle(base_image, (cell_x, cell_y), 40, (0,0,255),-1)
            elif cell == "yellow":
                cv2.circle(base_image, (cell_x, cell_y), 40, (0,255,255),-1)
            elif cell == "empty":
                cv2.circle(base_image, (cell_x, cell_y), 40, (255,255,255),-1)
    return base_image


state_image = display_state(board_state=board_state)


cv2.imshow("board state", state_image)
cv2.waitKey(0)

json_state = {"board_state": board_state}

# save state as json
with open("test.json", mode="w", encoding="utf-8") as write_file:
    json.dump(json_state, write_file, indent=4)

# load state as json
with open("test.json", encoding="utf-8") as read_file:
    loaded_json = json.load(read_file)

# print(loaded_json)
state_image = display_state(board_state=loaded_json["board_state"])

cv2.imshow("json loaded board state", state_image)
cv2.waitKey(0)

# cell_mask = np.zeros(warped.shape[:2], dtype=np.uint8)
# cv2.circle(cell_mask, (650,550), 50, 255, -1)

# red_masked = cv2.bitwise_and(cell_mask, cell_mask, mask=red_mask)

# # for i in range (7):
# #     for j in range (6):
# #         cv2.circle(cell_mask, (100*i+50,100*j+50), 50, 255, -1)

# masked = cv2.bitwise_and(warped, warped, mask=cell_mask)

# cv2.imshow("masked", masked)
# cv2.imshow("red masked", red_masked)

# if red_masked.any():
#     print("red in this cell!")

# cv2.waitKey(0)
