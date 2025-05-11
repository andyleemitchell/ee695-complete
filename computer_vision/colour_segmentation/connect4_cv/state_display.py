import json
import sys
from pathlib import Path

import cv2
import numpy as np


class StateDisplay:
    RED = (0, 0, 255)
    GREEN = (0, 255, 0)
    BLUE = (255, 0, 0)
    YELLOW = (0, 255, 255)
    WHITE = (255, 255, 255)

    BOARD_WIDTH = 7
    BOARD_HEIGHT = 6

    WIN_NAME = "State Display"

    DEFAULT_STATE_FILE = ".board_state.json"

    ENTER_KEYCODE = 13
    ESCAPE_KEYCODE = 27

    def __init__(self, json_file=""):
        if json_file:
            self.json_file = json_file
        else:
            self.json_file = self.DEFAULT_STATE_FILE

        if not Path(self.DEFAULT_STATE_FILE).is_file():
            print("JSON output file doesn't exist. Please run the detection first.")
            sys.exit()

    def _read_json(self):
        with open(self.json_file, encoding="utf-8") as read_file:
            self.board_state = json.load(read_file)["board_state"]

    def _create_image(self):
        self.image = np.zeros(
            (100*self.BOARD_HEIGHT, 100*self.BOARD_WIDTH, 3), np.uint8
        )
        cv2.rectangle(
            self.image, (0,0), (100*self.BOARD_WIDTH, 100*self.BOARD_HEIGHT),
            self.BLUE, -1)
        for y, row in enumerate(self.board_state):
            for x, cell in enumerate(row):
                cell_x = 100*x+50
                cell_y = 100*y+50
                if cell == "red":
                    cv2.circle(self.image, (cell_x, cell_y), 40, self.RED, -1)
                elif cell == "yellow":
                    cv2.circle(self.image, (cell_x, cell_y), 40, self.YELLOW, -1)
                elif cell == "empty":
                    cv2.circle(self.image, (cell_x, cell_y), 40, self.WHITE, -1)

    def _display_state(self):
        cv2.imshow(self.WIN_NAME, self.image)
        cv2.waitKey(100)

    def update(self):
        cv2.namedWindow(self.WIN_NAME)
        self._read_json()
        self._create_image()
        self._display_state()



# def display_state(board_state):
#     base_image = np.zeros((600, 700, 3), np.uint8)
#     for x, column in enumerate(board_state):
#         for y, cell in enumerate(column):
#             cell_x = 100*x+50
#             cell_y = 100*y+50
#             if cell == "red":
#                 cv2.circle(base_image, (cell_x, cell_y), 40, (0,0,255),-1)
#             elif cell == "yellow":
#                 cv2.circle(base_image, (cell_x, cell_y), 40, (0,255,255),-1)
#             elif cell == "empty":
#                 cv2.circle(base_image, (cell_x, cell_y), 40, (255,255,255),-1)
#     return base_image
