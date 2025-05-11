import json
import os
import sys
import time
from argparse import ArgumentParser

import cv2

import connect4_cv

parser = ArgumentParser()
parser.add_argument(
    "-c",
    "--calibrate",
    help="run calibration before doing any detection",
    action="store_true",
    default=False,
)
parser.add_argument(
    "-f", "--file", help="file to store board state in", default=".board_state.json"
)
args = parser.parse_args()

temp_json = args.file + ".tmp"

video_capture = cv2.VideoCapture(-1)
if not video_capture.isOpened():
    msg = "Could not open the camera."
    raise RuntimeError(msg)

detector = connect4_cv.ConnectFourStateDetector()


def process_loop():
    while True:
        ret, frame = video_capture.read()
        board_state = detector.detect(frame=frame)

        json_state = {"board_state": board_state}

        with open(temp_json, mode="w", encoding="utf-8") as write_file:
            json.dump(json_state, write_file, indent=4)

        os.replace(temp_json, args.file)

        time.sleep(1/20)


if args.calibrate:
    _, frame = video_capture.read()
    detector.calibrate(frame=frame, force=True)
    print("saving configuration........")
    time.sleep(0.5)
    sys.exit()


if __name__ == "__main__":
    process_loop()
