import sys
from pathlib import Path

import cv2
import numpy as np

from .colour_thresholder import ColourThresholder
from .config_loader import ConfigLoader
from .quad_selector import QuadSelector


class ConnectFourStateDetector:
    """
    Detects the state of a Connect Four board using computer vision.

    Loads a configuration file, named DEFAULT_CONF_FILE.
    This configuration file should have two entries:
        - quad_points (the points defining the area of the board
                       in the image for a fixed camera setup)
        - colour_limits (hsv values for upper and lower thresholds)
            - red
            - yellow

    To perform detection, pass an image frame into the detect() function.
    A representation of the board state will then be output (TODO: how??)
    """

    BOARD_WIDTH = 7
    BOARD_HEIGHT = 6
    DEFAULT_CONF_FILE = ".c4sd_conf.json"
    NOISE_LIM = 1000

    # needed:
    # calibration with quadselector and thresholding for colours.
    # detect function
    # should the detector only handle detection? i.e. leave the frame getting to a script?
    # how to handle outputs? what format? choosable?

    def __init__(self, conf_file=None, noise_lim=None):
        if noise_lim:
            self.NOISE_LIM = noise_lim
        self.conf = None
        # check if have a configuration file
        if conf_file:
            self.DEFAULT_CONF_FILE = conf_file
        config_file = Path(self.DEFAULT_CONF_FILE)
        if config_file.is_file():
            self._load_config()
        else:
            print("No config detected. Please run calibration before any detections.")

    def _load_config(self):
        # we should be calibrated now, so load config
        try:
            self.conf = ConfigLoader.load_json(self.DEFAULT_CONF_FILE)
        except FileNotFoundError as e:
            print(f"{type(e).__name__} at line {e.__traceback__.tb_lineno} of {__file__}: {e}")
            print("Ask the programmer why this didn't work.")
        print(f"config loaded from file: {self.DEFAULT_CONF_FILE}")

        expected_keys = ["colour_limits", "quad_points"]
        if not all(key in self.conf for key in expected_keys):
            print("Expected keys not present.")

        if not self.conf.colour_limits.red:
            print("Red Colour Limits not present in config file.")
        elif not self.conf.colour_limits.yellow:
            print("Yellow Colour Limits not present in config file.")
        else:
            print("Colour limits okay.")

        if not self.conf.quad_points:
            print("Quad Points not present in config file.")
        else:
            print("Quad Points okay.")

        # setup thresholds from config
        self.red_thresh = (
            np.array(self.conf.colour_limits.red[0], np.uint8),
            np.array(self.conf.colour_limits.red[1], np.uint8)
        )
        self.yellow_thresh = (
            np.array(self.conf.colour_limits.yellow[0], np.uint8),
            np.array(self.conf.colour_limits.yellow[1], np.uint8)
        )

    def _calibrate_quad(self):
        qs = QuadSelector()
        return qs.select_points(self.frame)

    def _calibrate_colours(self):
        ct = ColourThresholder()
        return ct.run(self.frame)

    def calibrate(self, frame=None, *, force=False):
        if frame is not None:
            self.frame = frame

        print("Connect Four Board Detector Calibration")
        print(60*"-")
        print("Press <enter> to start (and after each piece of text).")

        print("Select the four corners of the board, ensuring equal borders all around.")
        print("Press enter when you are happy with the values.")
        input()
        quad_points = self._calibrate_quad()
        self._warp_to_points(quad_points)

        print("Move the sliders so only the red counters are visible.")
        print("Press enter when you are happy with the values.")
        input()
        red_hsv = self._calibrate_colours()

        print("Move the sliders so only the yellow counters are visible.")
        print("Press enter when you are happy with the values.")
        input()
        yellow_hsv = self._calibrate_colours()

        conf = {
            "quad_points": quad_points,
            "colour_limits": {
                "red": red_hsv,
                "yellow": yellow_hsv
            }
        }

        conf_file = Path(self.DEFAULT_CONF_FILE)
        if force and conf_file.is_file():
            conf_file.unlink()

        ConfigLoader.write_json(self.DEFAULT_CONF_FILE, conf)
        self._load_config()

    def _order_points(self, points):
        """
        Order points [topleft, topright, botright, botleft]

        Args:
            points (list):  of (x,y) points

        Returns:
            np.float32 array: ordered points

        """
        # convert points into a numpy array first
        pts = np.array(points, np.int32)

        rect = np.zeros((4, 2), dtype=np.float32)

        s = pts.sum(axis=1)
        rect[0] = pts[np.argmin(s)]
        rect[2] = pts[np.argmax(s)]

        diff = np.diff(pts, axis=1)
        rect[1] = pts[np.argmin(diff)]
        rect[3] = pts[np.argmax(diff)]

        self.ordered_points = rect

    def _warp_to_points(self, points=None):
        if points:
            self._order_points(points)
        else:
            self._order_points(self.conf.quad_points)
        output_shape = np.array([
            [0,0],
            [100*self.BOARD_WIDTH, 0],
            [100*self.BOARD_WIDTH, 100*self.BOARD_HEIGHT],
            [0, 100*self.BOARD_HEIGHT]
        ], dtype=np.float32)
        trans_matrix = cv2.getPerspectiveTransform(self.ordered_points, output_shape)
        self.frame = cv2.warpPerspective(
            self.frame, trans_matrix,
            (100*self.BOARD_WIDTH, 100*self.BOARD_HEIGHT)
        )

    def _convert_to_hsv(self):
        self.frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
        shift = 50
        hsv_shifted = self.frame.copy()
        hsv_shifted[:, :, 0] = (hsv_shifted[:, :, 0] + shift) % 180
        self.frame = hsv_shifted.copy()

    def _mask_frame(self):
        self.red_mask = cv2.inRange(self.frame, self.red_thresh[0], self.red_thresh[1])
        self.yellow_mask = cv2.inRange(self.frame, self.yellow_thresh[0], self.yellow_thresh[1])

    def _check_cells(self):
        column_x = [100*i+50 for i in range(self.BOARD_WIDTH)]
        row_y = [100*i+50 for i in range(self.BOARD_HEIGHT)]

        self.board_state = []

        blank_mask = np.zeros(self.frame.shape[:2], dtype=np.uint8)

        for row in row_y:
            row_entries = []
            for column in column_x:
                cell_mask = np.copy(blank_mask)
                cv2.circle(cell_mask, (column, row), 40, 255, -1)
                red = cv2.bitwise_and(cell_mask, cell_mask, mask=self.red_mask)

                # cv2.imshow("cell", red)
                # cv2.waitKey(200)

                cell_mask = np.copy(blank_mask)
                cv2.circle(cell_mask, (column, row), 40, 255, -1)
                yellow = cv2.bitwise_and(cell_mask, cell_mask, mask=self.yellow_mask)

                any_red = red.any()
                any_yellow = yellow.any()
                amount_red = cv2.countNonZero(red)
                amount_yellow = cv2.countNonZero(yellow)

                cell_content = "empty"
                if any_red and any_yellow:
                    if amount_red > any_yellow and amount_red > self.NOISE_LIM:
                        cell_content = "red"
                    elif amount_yellow > amount_red and amount_yellow > self.NOISE_LIM:
                        cell_content = "yellow"
                elif any_red and amount_red > self.NOISE_LIM:
                    cell_content = "red"
                elif any_yellow and amount_yellow > self.NOISE_LIM:
                    cell_content = "yellow"

                row_entries.append(cell_content)

            self.board_state.append(row_entries)

    def _detection(self):
        self._warp_to_points()
        self._convert_to_hsv()
        self._mask_frame()
        self._check_cells()

    def detect(self, frame, output_file=""):
        self.frame = frame

        if self.conf is None:
            print("Please run the calibrate function first.")
            sys.exit()

        self._detection()

        if output_file:
            # so technically it's not a config but this method only does
            # a JSON dump so........ this will work.
            ConfigLoader.write_json(output_file, {"board_state": self.board_state})

        return self.board_state



if __name__ == "__main__":
    detector = ConnectFourStateDetector()

    video_capture = cv2.VideoCapture(-1)
    if not video_capture.isOpened():
        msg = "Could not open the camera."
        raise RuntimeError(msg)
    ret, frame = video_capture.read()
    if not ret:
        print("Failed to grab frame")

    board_state = detector.detect(frame=frame)

    print(board_state)
