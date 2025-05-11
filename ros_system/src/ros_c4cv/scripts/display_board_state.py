#!/usr/bin/env python3

import json
import os
import connect4_cv.state_validator
import numpy as np
import rospy
import cv2
from ros_c4msgs.msg import BoardState, MoveMade
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

WIN_NAME = "Connect Four Board State"
BOARD_WIDTH = 7
BOARD_HEIGHT = 6
RED = (0, 0, 255)
GREEN = (0, 255, 0)
BLUE = (255, 0, 0)
YELLOW = (0, 255, 255)
WHITE = (255, 255, 255)

board_state = [""] * 64

def board_state_from_msg(msg):
    one = 1
    for i in range(64):
        # TODO these are dependent on who moves first
        if (one << i) & msg.player1:
            board_state[i] = "red"
        elif (one << i) & msg.player2:
            board_state[i] = "yellow"
        else:
            board_state[i] = "empty"

def generate_image(msg):
    board_state_from_msg(msg)
    image = np.zeros(
            (100*BOARD_HEIGHT, 100*BOARD_WIDTH, 3), np.uint8
        )
    cv2.rectangle(
        image, (0,0), (100*BOARD_WIDTH, 100*BOARD_HEIGHT),
        BLUE, -1)
    for row in range(0, 6, 1):
        for column in range(0, 7, 1):
            cell = board_state[row + ((column)*7)]
            cell_x = 100*column+50
            cell_y = 100*(BOARD_HEIGHT-row)-50
            if cell == "red":
                cv2.circle(image, (cell_x, cell_y), 40, RED, -1)
            elif cell == "yellow":
                cv2.circle(image, (cell_x, cell_y), 40, YELLOW, -1)
            elif cell == "empty":
                cv2.circle(image, (cell_x, cell_y), 40, WHITE, -1)
    # show_image(image)
    image_message = bridge.cv2_to_imgmsg(image, "bgr8")
    board_image_pub.publish(image_message)
    rate.sleep()

def show_image(image):
    cv2.imshow(WIN_NAME, image)
    cv2.waitKey(10)

def init_window():
    cv2.namedWindow(WIN_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN_NAME, BOARD_WIDTH*100, BOARD_HEIGHT*100)

if __name__ == "__main__":
    rospy.init_node('display_board_state', anonymous=True)
    # init_window()
    rospy.Subscriber("/c4/board_state", BoardState, generate_image)
    board_image_pub = rospy.Publisher("/c4/board_image", Image, queue_size=10)
    bridge = CvBridge()
    rate = rospy.Rate(30)
    rospy.spin()
